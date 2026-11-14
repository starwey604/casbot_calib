#include "field_limits.hpp"
#include <stdexcept>
int main() {
  using namespace casbot::shm;
  StatusFrame state{};std::array<double,14> lower{},upper{};lower.fill(-10);upper.fill(10);
  for(int j=0;j<14;++j)arm_status(state,j,0,0,0);
  arm_status(state,0,11,0,0);
  auto limits=field_limits(state,lower,upper,1.5);
  if(limits.size()!=1||limits[0].end_control)throw std::runtime_error("Observed position must only warn");
  arm_status(state,1,0,2,0);limits=field_limits(state,lower,upper,1.5);
  if(limits.size()!=2||!limits[1].end_control)throw std::runtime_error("Observed velocity must end H1 control");
}
