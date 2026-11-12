#pragma once
#include <spdlog/async_logger.h>
#include <spdlog/details/thread_pool.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/pattern_formatter.h>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <ostream>
#include <sstream>
#include <stdexcept>

// Each stream has one producer. A shared, bounded spdlog worker owns disk I/O;
// backpressure blocks rather than dropping diagnostic records. The manual
// sender may opt into a single-producer nonblocking queue: on full/error, stop
// accepting new rows (never overwrite old ones) and report the preserved prefix.
class AsyncCsv final:public std::ostream {
  // After a write failure, queued messages must not resume later and introduce
  // holes after a partial row. Other files can still drain their valid prefixes.
  class PrefixSink final:public spdlog::sinks::sink {
   public:
    PrefixSink(const std::filesystem::path& path,std::shared_ptr<std::atomic<bool>> failed):failed_(std::move(failed)) {
      file_.open(path,std::ios::binary|std::ios::out|std::ios::trunc);file_.exceptions(std::ios::badbit|std::ios::failbit);
      formatter_=std::make_unique<spdlog::pattern_formatter>("%v");
    }
    void log(const spdlog::details::log_msg& msg) override {
      if(*failed_)return;
      try {spdlog::memory_buf_t text;formatter_->format(msg,text);file_.write(text.data(),text.size());}
      catch(...){*failed_=true;throw;}
    }
    void flush() override {if(*failed_)return;try{file_.flush();}catch(...){*failed_=true;throw;}}
    void set_pattern(const std::string& pattern) override {formatter_=std::make_unique<spdlog::pattern_formatter>(pattern);}
    void set_formatter(std::unique_ptr<spdlog::formatter> formatter) override {formatter_=std::move(formatter);}
   private:
    // This sink belongs to the manual pool's one I/O consumer. Formatters are
    // configured before enqueueing. Checked flush matters: spdlog 1.9's default
    // file helper ignores fflush failures (including a small /dev/full write).
    std::ofstream file_;std::unique_ptr<spdlog::formatter> formatter_;
    std::shared_ptr<std::atomic<bool>> failed_;
  };
  class Buffer final:public std::stringbuf {
   public:
    std::shared_ptr<spdlog::async_logger> logger;
    std::shared_ptr<std::atomic<bool>> failed=std::make_shared<std::atomic<bool>>(false);
    std::string pause_file;
    std::weak_ptr<spdlog::details::thread_pool> pool;
    std::size_t capacity=0;bool full=false;
    int sync() override {
      if(!pause_file.empty()&&std::filesystem::exists(pause_file)){str("");return 0;}
      if(*failed)return -1;
      auto text=str();
      if(text.empty())return 0;
      if(!logger)return -1;
      if(capacity) {
        auto worker=pool.lock();
        // Exactly one producer shares this opt-in pool. Only its consumer can
        // change occupancy between this check and the two enqueue operations.
        if(full||!worker||worker->queue_size()+2>capacity){full=true;str("");return -1;}
      }
      str("");
      if(text.back()=='\n')text.pop_back(); // spdlog supplies the final newline.
      logger->info("{}",text);logger->flush();return 0;
    }
  } buffer_;
  std::ofstream synchronous_;
 public:
  AsyncCsv():std::ostream(nullptr){rdbuf(&buffer_);exceptions(std::ios::badbit|std::ios::failbit);}
  ~AsyncCsv() override {try{flush();}catch(...){} }
  void open(const std::filesystem::path& path,const std::string& header,
            const std::shared_ptr<spdlog::details::thread_pool>& pool,std::size_t single_producer_capacity=0) {
    buffer_.pool=pool;buffer_.capacity=single_producer_capacity;
    std::shared_ptr<spdlog::sinks::sink> sink;
    if(single_producer_capacity)sink=std::make_shared<PrefixSink>(path,buffer_.failed);
    else sink=std::make_shared<spdlog::sinks::basic_file_sink_mt>(path.string(),true);
    buffer_.logger=std::make_shared<spdlog::async_logger>(path.string(),sink,pool,spdlog::async_overflow_policy::block);
    buffer_.logger->set_pattern("%v");
    buffer_.logger->set_error_handler([failed=buffer_.failed](const std::string&){*failed=true;});
    *this<<std::setprecision(17)<<header<<'\n';flush();
  }
  void open_synchronous(const std::filesystem::path& path,const std::string& header) {
    synchronous_.open(path);synchronous_.exceptions(std::ios::badbit|std::ios::failbit);
    rdbuf(synchronous_.rdbuf());*this<<std::setprecision(17)<<header<<'\n';
  }
  void set_pause_file(const std::string& path){buffer_.pause_file=path;}
  void check() const {if(buffer_.full)throw std::runtime_error("Manual CSV archive queue full; recording prefix retained");if(*buffer_.failed)throw std::runtime_error("Asynchronous diagnostic CSV write failed");}
};
