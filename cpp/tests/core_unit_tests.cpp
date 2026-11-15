#include <chrono>
#include <iostream>
int test_arm_core_tests();
int test_simulated_device_tests(int argc,char** argv);
int test_kinematics_tests();
int test_integration_tests();
int test_session_contracts(int argc, char** argv);
namespace {
template<class F> int timed(const char* name, F action) {
  const auto started=std::chrono::steady_clock::now();
  const int result=action();
  std::cout<<"TEST_TIMING "<<name<<" "
    <<std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()<<" s\n";
  return result;
}
}
int main(int argc, char** argv) {
  if (timed("arm_core",test_arm_core_tests)) return 1;
  if (timed("simulated_device",[&]{return test_simulated_device_tests(argc,argv);})) return 1;
  if (timed("kinematics",test_kinematics_tests)) return 1;
  if (timed("integration",test_integration_tests)) return 1;
  return timed("session",[&]{return test_session_contracts(argc,argv);});
}
