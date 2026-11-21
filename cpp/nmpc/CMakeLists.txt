cmake_minimum_required(VERSION 3.22)
project(casbot_nmpc LANGUAGES CXX)
find_package(coal 3.0 REQUIRED CONFIG)
find_package(Eigen3 REQUIRED CONFIG)
add_library(casbot_nmpc_runtime SHARED runtime.cpp)
target_compile_features(casbot_nmpc_runtime PRIVATE cxx_std_17)
target_include_directories(casbot_nmpc_runtime PRIVATE "${MODEL_DIR}" "${MODEL_DIR}/generated"
  "${ACADOS_DIR}/include" "${ACADOS_DIR}/include/blasfeo/include" "${ACADOS_DIR}/include/hpipm/include")
target_link_libraries(casbot_nmpc_runtime PRIVATE coal::coal Eigen3::Eigen
  "${MODEL_DIR}/generated/libacados_ocp_solver_dvt2.so" "${ACADOS_DIR}/lib/libacados.so")
set_target_properties(casbot_nmpc_runtime PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${MODEL_DIR}")
