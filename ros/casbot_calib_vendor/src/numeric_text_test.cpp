#include "numeric_text.hpp"
#include "file_stamp.hpp"
#include <filesystem>
#include <fstream>
#include <unistd.h>
#include <sstream>
#include <cmath>
#include <limits>
#include <cstdint>
#include <random>
#include <iostream>
int main() {
  std::mt19937 rng(20261112);
  for(int i=0;i<10000;++i) {
    const double value=i==0?-0.:std::ldexp(double(rng())/4294967296.-.5,int(rng()%1800)-900);
    std::ostringstream out;vendor_text::number(out,value);
    if(std::stod(out.str())!=value||std::signbit(std::stod(out.str()))!=std::signbit(value))return 1;
  }
  std::ostringstream out;vendor_text::number(out,std::numeric_limits<std::uint64_t>::max());
  if(out.str()!="18446744073709551615")return 2;
  const auto file=std::filesystem::temp_directory_path()/("casbot_mailbox_test_"+std::to_string(::getpid()));
  {std::ofstream stream(file);stream<<"first";}
  const auto before=mailbox_stamp(file);const auto stamp=std::filesystem::last_write_time(file);
  const auto pending=file.string()+".pending";
  {std::ofstream stream(pending);stream<<"other";}
  std::filesystem::last_write_time(pending,stamp);std::filesystem::rename(pending,file);
  if(mailbox_stamp(file)==before)return 3;
  std::filesystem::remove(file);
  std::cout<<"17-digit float round trips and full integer precision retained\n";
}
