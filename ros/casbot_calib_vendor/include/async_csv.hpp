#pragma once
#include <spdlog/async_logger.h>
#include <spdlog/details/thread_pool.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <atomic>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <ostream>
#include <sstream>
#include <stdexcept>

// Each stream has one producer. A shared, bounded spdlog worker owns disk I/O;
// backpressure blocks rather than dropping diagnostic records.
class AsyncCsv final:public std::ostream {
  class Buffer final:public std::stringbuf {
   public:
    std::shared_ptr<spdlog::async_logger> logger;
    std::shared_ptr<std::atomic<bool>> failed=std::make_shared<std::atomic<bool>>(false);
    std::string pause_file;
    int sync() override {
      if(!pause_file.empty()&&std::filesystem::exists(pause_file)){str("");return 0;}
      if(*failed)return -1;
      auto text=str();
      if(text.empty())return 0;
      if(!logger)return -1;
      str("");
      if(text.back()=='\n')text.pop_back(); // spdlog supplies the final newline.
      logger->info("{}",text);logger->flush();return 0;
    }
  } buffer_;
 public:
  AsyncCsv():std::ostream(nullptr){rdbuf(&buffer_);exceptions(std::ios::badbit|std::ios::failbit);}
  ~AsyncCsv() override {try{flush();}catch(...){} }
  void open(const std::filesystem::path& path,const std::string& header,
            const std::shared_ptr<spdlog::details::thread_pool>& pool) {
    auto sink=std::make_shared<spdlog::sinks::basic_file_sink_mt>(path.string(),true);
    buffer_.logger=std::make_shared<spdlog::async_logger>(path.string(),sink,pool,spdlog::async_overflow_policy::block);
    buffer_.logger->set_pattern("%v");
    buffer_.logger->set_error_handler([failed=buffer_.failed](const std::string&){*failed=true;});
    *this<<std::setprecision(17)<<header<<'\n';flush();
  }
  void set_pause_file(const std::string& path){buffer_.pause_file=path;}
  void check() const {if(*buffer_.failed)throw std::runtime_error("Asynchronous diagnostic CSV write failed");}
};
