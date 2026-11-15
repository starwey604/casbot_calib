#include <omp.h>
#include <dlfcn.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <array>
#include <cstdlib>
#include <iostream>
#include <set>

#ifndef _OPENMP
#error OpenMP must be enabled for this probe
#endif

int main(int argc,char** argv) {
  if(argc!=2) return 2;
  const int expected=std::atoi(argv[1]);
  if(expected<1 || expected>64) return 2;
  std::array<long,64> tids{};
  long long sum=0;
  int visits=0;
#pragma omp parallel reduction(+:sum,visits)
  {
    const int id=omp_get_thread_num();
    if(id<64) tids[id]=syscall(SYS_gettid);
    visits+=1;
#pragma omp for schedule(static)
    for(int i=0;i<1000;++i) sum+=i;
  }
  std::set<long> threads;
  for(const long tid:tids) if(tid) threads.insert(tid);
  Dl_info library{};
  if(!dladdr(reinterpret_cast<void*>(&omp_get_thread_num),&library)) return 1;
  std::cout<<"OpenMP="<<_OPENMP<<" threads="<<visits
           <<" tids="<<threads.size()<<" sum="<<sum
           <<" runtime="<<library.dli_fname<<'\n';
  return visits==expected && threads.size()==static_cast<std::size_t>(expected) && sum==499500 ? 0 : 1;
}
