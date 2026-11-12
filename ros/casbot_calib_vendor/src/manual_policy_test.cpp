#include "manual_policy.hpp"
#include <cassert>
#include <limits>
int main() {
  std::vector<double> v(154,1.);v[0]=1;
  for(int j=0;j<14;++j){v[1+j]=-2.;v[1+6*14+j]=-2.;v[1+9*14+j]=0;}
  auto p=ManualPolicy::parse(v);assert(p.revision==1&&p.axis[ManualPolicy::KpMax][2]==1.);
  auto next=p;next.axis[ManualPolicy::ObservedVelocity][0]=4.;assert(next.same_pinned(p));
  next.axis[ManualPolicy::KpMax][0]=2000.;assert(!next.same_pinned(p));
  for(auto index:{0,1,16,127,153}) {
    auto bad=v;bad[index]=std::numeric_limits<double>::quiet_NaN();bool rejected=false;
    try {ManualPolicy::parse(bad);}catch(const std::runtime_error&){rejected=true;}assert(rejected);
  }
  auto high=v;for(int j=0;j<14;++j)high[1+4*14+j]=2000.;assert(ManualPolicy::parse(high).axis[ManualPolicy::KpMax][0]==2000.);
  v[1+9*14]=2;bool rejected=false;try {ManualPolicy::parse(v);}catch(const std::runtime_error&){rejected=true;}assert(rejected);
}
