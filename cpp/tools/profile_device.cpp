// Native-only replay through the same device used by both ROS Mock endpoints.
#include <casbot_calib/simulated_device.hpp>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <type_traits>

std::vector<std::string> cells(const std::string& line) {
  std::vector<std::string> out;std::istringstream stream(line);std::string value;
  while(std::getline(stream,value,','))out.push_back(value);
  return out;
}
int main(int argc,char** argv) {
  try {
    if(argc!=7)throw std::invalid_argument("profile_device URDF SETTINGS COMMANDS EPOCH_NS DURATION_NS OUTPUT_PREFIX");
    std::map<std::string,std::vector<std::string>> settings;std::string line;
    std::ifstream params(argv[2]);if(!params)throw std::runtime_error("Missing settings");
    while(std::getline(params,line)) {auto row=cells(line);if(row.size()<2)throw std::invalid_argument("Invalid setting row");settings[row[0]]={row.begin()+1,row.end()};}
    auto declared=casbot::simulated_arm_settings(argv[1],[&](const std::string& name,auto fallback) {
      using T=decltype(fallback);const auto it=settings.find(name);if(it==settings.end())return fallback;
      if constexpr(std::is_same_v<T,std::vector<double>>) {T result;for(const auto& x:it->second)result.push_back(std::stod(x));return result;}
      else if constexpr(std::is_integral_v<T>)return static_cast<T>(std::stoll(it->second.at(0)));
      else return static_cast<T>(std::stod(it->second.at(0)));
    });
    casbot::SimulatedArmDevice device(casbot::ArmDynamics(declared.model),declared.simulation,declared.initial_q);
    const casbot::Nanoseconds epoch=std::stoll(argv[4]),duration=std::stoll(argv[5]);const int n=declared.model.joints.size();
    std::ifstream commands(argv[3]);if(!commands)throw std::runtime_error("Missing commands");std::getline(commands,line);
    std::size_t submitted=0;
    while(std::getline(commands,line)) {
      auto row=cells(line);if(row.size()!=static_cast<std::size_t>(2+5*n))throw std::invalid_argument("Unexpected SDK device command columns");
      const auto t=std::stoll(row[0])-epoch;if(t>=duration)break;
      if(row[1]!="1")throw std::invalid_argument("Device-local PD required");
      Eigen::MatrixXd values(n,5);for(int k=0;k<5;++k)for(int j=0;j<n;++j)values(j,k)=std::stod(row[2+k*n+j]);
      device.submit({t,values.col(0),values.col(1),values.col(2),values.col(3),values.col(4)});++submitted;
    }
    using Clock=std::chrono::steady_clock;using Ns=std::chrono::nanoseconds;
    const casbot::Nanoseconds chunk=1000000;std::vector<std::pair<casbot::Nanoseconds,double>> advances;
    std::vector<casbot::ArmSample> samples;const auto begin=Clock::now();
    for(casbot::Nanoseconds end=chunk;device.time_ns()<duration;end+=chunk) {
      end=std::min(end,duration);const auto before=Clock::now();auto delivered=device.advance_to(end);
      const auto elapsed=std::chrono::duration<double,std::nano>(Clock::now()-before).count();advances.emplace_back(end,elapsed);
      for(auto& sample:delivered)samples.push_back(std::move(sample));
    }
    const double wall=std::chrono::duration<double>(Clock::now()-begin).count();
    std::ofstream timings(std::string(argv[6])+"_advances.csv");timings<<"virtual_ns,compute_ns\n"<<std::setprecision(17);
    std::vector<double> cost;double total=0;for(auto [t,dt]:advances){timings<<t<<','<<dt<<'\n';cost.push_back(dt);total+=dt;}
    std::sort(cost.begin(),cost.end());
    std::ofstream observations(std::string(argv[6])+"_samples.csv");observations<<"sample_ns";
    for(const auto* prefix:{"q","dq","current"})for(int j=0;j<n;++j)observations<<','<<prefix<<j;
    observations<<'\n'<<std::setprecision(17);
    for(const auto& s:samples) {observations<<s.sample_ns;for(const auto* v:{&s.q,&s.dq,&s.current})for(int j=0;j<n;++j)observations<<','<<(*v)[j];observations<<'\n';}
    if(!timings||!observations)throw std::runtime_error("Profiling output write failed");
    const auto percentile=[&](double f){return cost.at(static_cast<std::size_t>(f*(cost.size()-1)))*1e-6;};
    std::cout<<std::setprecision(17)<<"{\"duration_s\":"<<duration*1e-9<<",\"wall_s\":"<<wall
      <<",\"compute_s\":"<<total*1e-9<<",\"real_time_cost_ratio\":"<<wall/(duration*1e-9)
      <<",\"advance_p50_ms\":"<<percentile(.5)<<",\"advance_p99_ms\":"<<percentile(.99)
      <<",\"advance_max_ms\":"<<cost.back()*1e-6<<",\"commands\":"<<submitted<<",\"samples\":"<<samples.size()<<"}\n";
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
