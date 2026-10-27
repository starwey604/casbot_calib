cmake_minimum_required(VERSION 3.22)
project(casbot_calib_core VERSION 0.1.0 LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
# Compile the same native source tree; ROS does not introduce another model.
set(CALIB_BUILD_PYTHON OFF CACHE BOOL "Python is installed independently" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Native tests run in build/humble" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../.. ${CMAKE_CURRENT_BINARY_DIR}/native)
ament_package(CONFIG_EXTRAS cmake/casbot_calib_core-extras.cmake)
