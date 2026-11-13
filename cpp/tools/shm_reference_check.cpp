#include "reference.hpp"
#include <iostream>
#include <iomanip>
int main(int argc,char** argv) {
  if(argc!=2)return 2;
  try {
    auto r=casbot::shm::Reference::load(argv[1]);
    std::cout<<std::setprecision(17);std::int64_t t;
    while(std::cin>>t){auto v=r.at(t);std::cout<<t;for(const auto& row:v.state)for(double x:row)std::cout<<','<<x;for(double x:v.effort)std::cout<<','<<x;std::cout<<'\n';}
    return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
