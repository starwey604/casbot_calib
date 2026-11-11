#include "sdk_mock_policy.hpp"
#include <stdexcept>
#include <initializer_list>
int main() {
  using namespace casbot::sdk;
  using Clock=std::chrono::steady_clock;using namespace std::chrono_literals;
  const Clock::time_point origin{};
  if(next_physics_deadline(origin,1ms,origin+300us)!=origin+1ms)
    throw std::runtime_error("Early step lost the periodic deadline");
  if(next_physics_deadline(origin,1ms,origin+5ms)!=origin+5ms)
    throw std::runtime_error("Late physics step inserted an extra idle period");
  if(next_physics_deadline(origin+5ms,1ms,origin+5700us)!=origin+6ms)
    throw std::runtime_error("Periodic pacing did not resume after catch-up");
  if(needs_plant_pose(true,false,false))throw std::runtime_error("Five-field forwarding depends on host feedback");
  if(!needs_plant_pose(false,false,false)||!needs_plant_pose(true,true,false)||!needs_plant_pose(true,false,true))
    throw std::runtime_error("Synthetic hold needs an observed pose");
  for(bool active:{false,true})for(bool first:{false,true})
    if(needs_plant_pose(active,first,false,false))throw std::runtime_error("Manual Mock added model gravity");
  if(!publishes_cached_feedback(true,true)||publishes_cached_feedback(false,true)||publishes_cached_feedback(true,false))
    throw std::runtime_error("Cache presence and explicit disconnect govern publication, not raw acquisition age");
}
