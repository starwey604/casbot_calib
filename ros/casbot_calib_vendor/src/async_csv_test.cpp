#include "async_csv.hpp"
#include <fstream>
#include <iostream>
#include <iterator>
#include <thread>
#include <chrono>
#include <atomic>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>

int main(int argc,char** argv) {
  if(argc!=2)return 2;
  const std::filesystem::path path(argv[1]);
  auto pool=std::make_shared<spdlog::details::thread_pool>(2,1);
  {
    AsyncCsv out;out.open(path,"sequence,value",pool);
    for(int i=0;i<1000;++i){out<<i<<','<<.125<<'\n';out.flush();}
    pool.reset();out.check(); // join/drain must retain every queued row.
  }
  std::ifstream input(path);std::string line;std::getline(input,line);
  if(line!="sequence,value")return 3;
  for(int i=0;i<1000;++i){if(!std::getline(input,line)||line!=std::to_string(i)+",0.125")return 4;}
  if(std::getline(input,line))return 5;
  input.close();std::filesystem::remove(path);
  // A blocked disk/FIFO may not block the opt-in manual producer or overwrite
  // already accepted rows. Drain later and verify the exact retained prefix.
  const auto fifo=path.string()+".fifo."+std::to_string(getpid());
  if(mkfifo(fifo.c_str(),0600))return 6;
  const int reader=open(fifo.c_str(),O_RDONLY|O_NONBLOCK);if(reader<0)return 7;
  std::string retained;std::size_t accepted=0;std::atomic<bool> drained{false};
  {
    auto worker=std::make_shared<spdlog::details::thread_pool>(8,1);
    AsyncCsv bounded;bounded.open(fifo,"sequence,value",worker,8);bool rejected=false;
    while(worker->queue_size())std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const std::string large(1024*1024,'x');bounded<<0<<','<<large<<'\n';bounded.flush();accepted=1;
    // The consumer takes the large record and blocks on the unread FIFO; its
    // flush job remains queued. Only then exercise the foreground producer.
    const auto wait_until=std::chrono::steady_clock::now()+std::chrono::seconds(1);
    while(worker->queue_size()!=1&&std::chrono::steady_clock::now()<wait_until)std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const bool writer_blocked=worker->queue_size()==1;
    const auto start=std::chrono::steady_clock::now();
    for(std::size_t i=1;i<10000;++i) {
      try {bounded<<i<<','<<.125<<'\n';bounded.flush();++accepted;}
      catch(const std::ios_base::failure&){rejected=true;break;}
    }
    if(!rejected||std::chrono::steady_clock::now()-start>std::chrono::seconds(1))return 8;
    std::thread consume([&]{char bytes[8192];while(true){auto n=read(reader,bytes,sizeof(bytes));if(n>0)retained.append(bytes,n);else if(drained)break;else std::this_thread::sleep_for(std::chrono::milliseconds(1));}});
    worker.reset();drained=true;consume.join();
    if(!writer_blocked)return 13;
    bool full=false;try{bounded.check();}catch(const std::runtime_error&){full=true;}if(!full)return 9;
  }
  close(reader);std::filesystem::remove(fifo);
  std::string expected="sequence,value\n0,"+std::string(1024*1024,'x')+"\n";
  for(std::size_t i=1;i<accepted;++i)expected+=std::to_string(i)+",0.125\n";
  if(retained!=expected)return 10;
  {
    auto worker=std::make_shared<spdlog::details::thread_pool>(8,1);
    AsyncCsv full;full.open("/dev/full","sequence,value",worker,8);worker.reset();
    bool failed=false;try{full.check();}catch(const std::runtime_error&){failed=true;}if(!failed)return 11;
  }
  {
    AsyncCsv synchronous;synchronous.open_synchronous(path,"sequence,value");synchronous<<0<<','<<.125<<'\n';synchronous.flush();
  }
  std::ifstream legacy(path);std::string legacy_text((std::istreambuf_iterator<char>(legacy)),{});legacy.close();std::filesystem::remove(path);
  if(legacy_text!="sequence,value\n0,0.125\n")return 12;
  std::cout<<"bounded async CSV drains without drops or extra lines\n";
}
