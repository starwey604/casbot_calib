cmake_minimum_required(VERSION 3.22)
project(casbot_calib VERSION 0.1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CTest)
if(CALIB_ORIN_CROSS)
  include(cmake/OrinCross.cmake)
endif()
option(CALIB_BUILD_CORE "Build dynamics, geometry and simulation core" ON)
option(CALIB_BUILD_PYTHON "Build NumPy bindings" ON)
option(CALIB_BUILD_SHM "Build standalone Linux SHM protocol foundation" OFF)
option(CALIB_BUILD_SHM_MOCK "Build private SHM physical mock" OFF)
if(CALIB_BUILD_SHM_MOCK AND (NOT CALIB_BUILD_CORE OR NOT CALIB_BUILD_SHM))
  message(FATAL_ERROR "SHM physics mock requires CORE and SHM")
endif()
if(NOT CALIB_BUILD_CORE AND CALIB_BUILD_PYTHON)
  message(FATAL_ERROR "Python bindings require CALIB_BUILD_CORE=ON")
endif()

if(CALIB_BUILD_CORE)
  find_package(Eigen3 3.4 REQUIRED NO_MODULE)
  find_package(Boost 1.74 REQUIRED CONFIG)
  find_package(pinocchio 3.9 REQUIRED)
  find_package(coal 3.0 REQUIRED CONFIG)
  if(CALIB_ORIN_CROSS)
    find_package(OpenMP REQUIRED COMPONENTS CXX)
    include(CheckCXXSourceCompiles)
    set(CMAKE_REQUIRED_LIBRARIES OpenMP::OpenMP_CXX)
    check_cxx_source_compiles("#include <omp.h>
#ifndef _OPENMP
#error OpenMP is required
#endif
int main() { return omp_get_max_threads() < 1; }" CALIB_OPENMP_ENABLED)
    unset(CMAKE_REQUIRED_LIBRARIES)
    if(NOT CALIB_OPENMP_ENABLED)
      message(FATAL_ERROR "Orin cross compiler must enable OpenMP")
    endif()
    add_library(casbot_calib::openmp_runtime SHARED IMPORTED)
    set_target_properties(casbot_calib::openmp_runtime PROPERTIES
      IMPORTED_LOCATION "${OpenMP_omp_LIBRARY}")
    calib_install_orin_openmp(Core)
  else()
    find_package(OpenMP QUIET)
  endif()
  add_library(calib_integration INTERFACE)
  add_library(casbot_calib::integration ALIAS calib_integration)
  set_target_properties(calib_integration PROPERTIES EXPORT_NAME integration)
  target_compile_features(calib_integration INTERFACE cxx_std_17)
  target_include_directories(calib_integration INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
  target_link_libraries(calib_integration INTERFACE Eigen3::Eigen Boost::headers)
  add_library(calib_core STATIC cpp/src/arm_dynamics.cpp cpp/src/arm_session.cpp cpp/src/collision.cpp cpp/src/trical.cpp)
  add_library(casbot_calib::core ALIAS calib_core)
  set_target_properties(calib_core PROPERTIES POSITION_INDEPENDENT_CODE ON EXPORT_NAME core)
  target_link_libraries(calib_core PUBLIC calib_integration pinocchio::pinocchio coal::coal)
  if(OpenMP_CXX_FOUND)
    separate_arguments(_calib_omp_flags NATIVE_COMMAND "${OpenMP_CXX_FLAGS}")
    target_compile_options(calib_core PRIVATE ${_calib_omp_flags})
    if(CALIB_ORIN_CROSS)
      target_link_libraries(calib_core PUBLIC casbot_calib::openmp_runtime)
    else()
      target_link_libraries(calib_core PUBLIC ${OpenMP_CXX_LIBRARIES})
    endif()
  endif()
  if(MSVC)
    target_compile_options(calib_core PRIVATE /W4)
  else()
    target_compile_options(calib_core PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  if(CALIB_BUILD_PYTHON)
    find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
    find_package(pybind11 2.13 CONFIG REQUIRED)
    pybind11_add_module(_core cpp/src/bindings.cpp cpp/src/arm_bindings.cpp cpp/src/collision_bindings.cpp)
    target_link_libraries(_core PRIVATE calib_core)
    if(OpenMP_CXX_FOUND)
      target_link_libraries(_core PRIVATE OpenMP::OpenMP_CXX)
    endif()
    if(CALIB_ORIN_CROSS)
      set_target_properties(_core PROPERTIES INSTALL_RPATH "$ORIGIN/.libs;/opt/ros/humble/lib/aarch64-linux-gnu;/opt/ros/humble/lib")
      install(FILES /opt/orin-openmp/lib/libomp.so.5
        DESTINATION casbot_calib/.libs COMPONENT Python)
      install(DIRECTORY /opt/orin-openmp/licenses/
        DESTINATION casbot_calib/.libs/licenses COMPONENT Python)
    else()
      set_target_properties(_core PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
    endif()
    install(TARGETS _core
      LIBRARY DESTINATION casbot_calib COMPONENT Python
      RUNTIME DESTINATION casbot_calib COMPONENT Python)
  endif()
  install(TARGETS calib_core calib_integration EXPORT casbot_calibTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Core)
  install(DIRECTORY cpp/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT Core
    PATTERN "shm" EXCLUDE)
  install(DIRECTORY python/casbot_calib/assets/models/
    DESTINATION ${CMAKE_INSTALL_DATADIR}/casbot_calib/models COMPONENT Models)
  install(EXPORT casbot_calibTargets FILE casbot_calibTargets.cmake NAMESPACE casbot_calib::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/casbot_calib COMPONENT Core)
  if(BUILD_TESTING)
    if(CALIB_ORIN_CROSS)
      add_executable(openmp_probe cpp/tests/openmp_probe.cpp)
      target_link_libraries(openmp_probe PRIVATE OpenMP::OpenMP_CXX ${CMAKE_DL_LIBS})
      install(TARGETS openmp_probe RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT CrossTests)
    endif()
    add_executable(core_unit_tests cpp/tests/core_unit_tests.cpp
      cpp/tests/arm_core_tests.cpp cpp/tests/simulated_device_tests.cpp
      cpp/tests/kinematics_tests.cpp cpp/tests/integration_tests.cpp cpp/tests/session_tests.cpp)
    find_package(Threads REQUIRED)
    target_link_libraries(core_unit_tests PRIVATE calib_core Threads::Threads)
    add_test(NAME core_unit COMMAND core_unit_tests ${CMAKE_CURRENT_SOURCE_DIR}/python/casbot_calib/assets/models/casbot_hl12p1_dvt2/derived/bare_flange_1105_03_gauge_v2/model.urdf)
    if(CALIB_ORIN_CROSS)
      install(TARGETS core_unit_tests RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT CrossTests)
    endif()
  endif()
endif()

if(CALIB_BUILD_SHM)
  if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "The SHM transport targets require Linux; the algorithm core remains portable")
  endif()
  add_subdirectory(cpp/src/shm)
endif()

# A component-only SHM install must remain usable without the dynamics packages.
if(CALIB_BUILD_CORE OR CALIB_BUILD_SHM)
  configure_package_config_file(cmake/casbot_calibConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/casbot_calibConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/casbot_calib
    PATH_VARS CMAKE_INSTALL_DATADIR)
  write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/casbot_calibConfigVersion.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
  foreach(_calib_config_component IN ITEMS Core Shm)
    if((_calib_config_component STREQUAL "Core" AND CALIB_BUILD_CORE) OR
       (_calib_config_component STREQUAL "Shm" AND CALIB_BUILD_SHM))
      install(FILES ${CMAKE_CURRENT_BINARY_DIR}/casbot_calibConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/casbot_calibConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/casbot_calib COMPONENT ${_calib_config_component})
    endif()
  endforeach()
endif()
