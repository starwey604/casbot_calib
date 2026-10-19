#include "casbot_calib/simulator.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace casbot;
void check(bool condition,const char* message) {if(!condition)throw std::runtime_error(message);}
int main() {
  SimulationConfig c;c.damping=0;c.coulomb=0;c.initial_q=.2;
  auto r=SimulationRunner(c).run(std::vector<JointCommand>{},3000000000LL);
  const double initial=r.truth.front().energy;
  for(const auto& s:r.truth) {
    check(std::abs(s.energy-initial)<1e-9,"energy conservation");
    double expected=.2*std::cos(std::sqrt(c.spring/c.inertia)*s.time_ns*1e-9);
    check(std::abs(s.q-expected)<1e-9,"independent analytic oscillator");
  }
  c.damping=.1;
  r=SimulationRunner(c).run(std::vector<JointCommand>{},3000000000LL);
  for(std::size_t n=1;n<r.truth.size();++n)check(r.truth[n].energy<=r.truth[n-1].energy+1e-12,"energy dissipation");
  c.command_delay_ns=1500000;c.feedback_delay_ns=2500000;
  JointCommand cmd;cmd.torque_ff=.1;
  r=SimulationRunner(c).run({cmd},5000000);
  check(r.observations.front().receive_ns==2500000,"feedback delay");
  check(r.truth[1].current==0 && r.truth[2].current>0,"command event ordering");
  MockJointDevice mock;
  JointDevice& device=mock;
  device.write(cmd);
  auto batch=SimulationRunner(c).run(mock,5000000);
  JointSample sample;std::size_t count=0;
  while(device.read(sample)) ++count;
  check(count==batch.observations.size(),"mock SDK read/write contract");
  bool rejected=false;try{c.inertia=0;SimulationRunner bad(c);}catch(const std::invalid_argument&){rejected=true;}
  check(rejected,"reject invalid physics");
  std::cout<<"Analytic response, energy and timing checks passed\n";
}
