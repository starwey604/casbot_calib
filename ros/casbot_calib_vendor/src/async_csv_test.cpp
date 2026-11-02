#include "async_csv.hpp"
#include <fstream>
#include <iostream>
#include <iterator>

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
  input.close();std::filesystem::remove(path);std::cout<<"bounded async CSV drains without drops or extra lines\n";
}
