#pragma once
#include <zlib.h>
#include <algorithm>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

// One recorder producer, one compression/I/O owner. enqueue never waits for disk.
// A full queue latches a prefix-preserving recording failure, never drops a row
// and then resumes later. The sender is a different process.
class GzipChunks {
 public:
  static std::string frame(const std::string& json,const char* cdr,std::size_t size) {
    if(json.size()>UINT32_MAX||size>UINT32_MAX)throw std::ios_base::failure("Raw frame exceeds uint32 length");
    std::string out;
    for(auto n:{static_cast<std::uint32_t>(json.size()),static_cast<std::uint32_t>(size)})
      for(int i=0;i<4;++i)out+=static_cast<char>((n>>(8*i))&255);
    out+=json;out.append(cdr,size);return out;
  }
  explicit GzipChunks(const std::filesystem::path& root,std::size_t chunk_bytes=4*1024*1024,
                      std::size_t queue_bytes=32*1024*1024):root_(root),chunk_bytes_(chunk_bytes),capacity_(queue_bytes) {
    std::filesystem::create_directory(root_/"chunks");
    lock_=::open((root_/".archive.lock").c_str(),O_CREAT|O_RDWR,0600);
    if(lock_<0)throw std::ios_base::failure("Cannot create archive lock");
    std::ofstream format(root_/"format.json");format.exceptions(std::ios::badbit|std::ios::failbit);
    format<<"{\"schema_version\":\"casbot_raw_gzip_frames_v2\",\"compression\":\"gzip_level_1\","
      "\"framing\":\"CBRAW2\\\\n + repeated uint32_le JSON bytes, uint32_le CDR bytes, JSON, CDR\","
      "\"chunk_bytes\":"<<chunk_bytes_<<",\"chunk_max_s\":1,\"queue_limit_bytes\":"<<capacity_
      <<",\"active_tail\":\"read complete frames; incomplete gzip footer allowed only in .open.gz\"}\n";
    worker_=std::thread([this]{run();});
  }
  ~GzipChunks(){close();if(lock_>=0)::close(lock_);}
  void check() {
    std::lock_guard<std::mutex> guard(mutex_);
    if(!error_.empty())throw std::ios_base::failure(error_);
  }
  void enqueue(std::string& batch) {
    std::lock_guard<std::mutex> guard(mutex_);
    if(!error_.empty())throw std::ios_base::failure(error_);
    if(batch.empty())return;
    if(queued_+batch.size()>capacity_) {
      error_="Raw compression queue full; recording prefix retained";
      throw std::ios_base::failure(error_);
    }
    queued_+=batch.size();queue_.emplace_back();queue_.back().swap(batch);ready_.notify_one();
  }
  void close() {
    {std::lock_guard<std::mutex> guard(mutex_);closing_=true;ready_.notify_one();}
    if(worker_.joinable())worker_.join();
  }
 private:
  void write(const char* data,std::size_t size) {
    while(size) {
      const auto n=static_cast<unsigned>(std::min<std::size_t>(size,1024*1024));
      if(gzwrite(file_,data,n)!=static_cast<int>(n))throw std::ios_base::failure("Raw gzip write failed");
      data+=n;size-=n;
    }
  }
  void open_chunk() {
    std::ostringstream name;name<<std::setw(8)<<std::setfill('0')<<index_++;
    path_=root_/"chunks"/(name.str()+".open.gz");
    file_=gzopen(path_.c_str(),"wb1");
    if(!file_)throw std::ios_base::failure("Cannot open raw gzip chunk");
    write("CBRAW2\n",7);chunk_size_=0;since_=std::chrono::steady_clock::now();
  }
  void seal() {
    if(!file_)return;
    auto file=file_;file_=nullptr;
    if(gzclose(file)!=Z_OK)throw std::ios_base::failure("Raw gzip close failed");
    auto final=path_;final.replace_filename(path_.filename().string().substr(0,8)+".bin.gz");
    if(flock(lock_,LOCK_EX))throw std::ios_base::failure("Cannot lock archive rotation");
    try {std::filesystem::rename(path_,final);}
    catch(...){flock(lock_,LOCK_UN);throw;}
    flock(lock_,LOCK_UN);
  }
  void run() {
    try {
      for(;;) {
        std::string batch;
        {std::unique_lock<std::mutex> guard(mutex_);ready_.wait(guard,[this]{return closing_||!queue_.empty();});
          if(queue_.empty())break;
          batch.swap(queue_.front());queue_.pop_front();queued_-=batch.size();}
        if(!file_)open_chunk();
        write(batch.data(),batch.size());chunk_size_+=batch.size();
        if(gzflush(file_,Z_SYNC_FLUSH)!=Z_OK)throw std::ios_base::failure("Raw gzip flush failed");
        if(chunk_size_>=chunk_bytes_||std::chrono::steady_clock::now()-since_>=std::chrono::seconds(1))seal();
      }
      seal();
    } catch(const std::exception& e) {
      // Do not finalize a failed chunk: its complete frames remain a recoverable
      // .open.gz prefix. No later chunk is written after a failure.
      if(file_){gzclose(file_);file_=nullptr;}
      std::lock_guard<std::mutex> guard(mutex_);error_=e.what();queue_.clear();queued_=0;
    }
  }
  std::filesystem::path root_,path_;std::size_t chunk_bytes_,capacity_,queued_=0,chunk_size_=0,index_=0;
  int lock_=-1;gzFile file_=nullptr;std::chrono::steady_clock::time_point since_;
  std::mutex mutex_;std::condition_variable ready_;std::deque<std::string> queue_;
  std::thread worker_;bool closing_=false;std::string error_;
};
