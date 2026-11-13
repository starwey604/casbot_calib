find_package(Threads REQUIRED)
add_library(calib_shm STATIC wire35.cpp channel.cpp)
add_library(casbot_calib::shm ALIAS calib_shm)
set_target_properties(calib_shm PROPERTIES POSITION_INDEPENDENT_CODE ON EXPORT_NAME shm)
target_compile_features(calib_shm PUBLIC cxx_std_17)
target_link_libraries(calib_shm PUBLIC Threads::Threads)
target_include_directories(calib_shm PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/cpp/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_options(calib_shm PRIVATE -Wall -Wextra -Wpedantic)

if(CALIB_BUILD_CORE)
  add_executable(shm_joint_test ${PROJECT_SOURCE_DIR}/cpp/tools/shm_joint_test.cpp)
  target_link_libraries(shm_joint_test PRIVATE calib_shm calib_core)
  set_target_properties(shm_joint_test PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
  install(TARGETS shm_joint_test RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT Shm)
endif()

# A real offline ABI diagnostic, not a placeholder motion executor.
add_executable(shm_abi_check ${PROJECT_SOURCE_DIR}/cpp/tools/shm_abi_check.cpp)
target_link_libraries(shm_abi_check PRIVATE calib_shm)
install(TARGETS calib_shm shm_abi_check EXPORT casbot_calibShmTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Shm
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT Shm)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/cpp/include/casbot_calib/shm
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/casbot_calib COMPONENT Shm)
install(EXPORT casbot_calibShmTargets FILE casbot_calibShmTargets.cmake NAMESPACE casbot_calib::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/casbot_calib COMPONENT Shm)

if(BUILD_TESTING)
  if(CALIB_BUILD_CORE)
    add_executable(shm_joint_integration ${PROJECT_SOURCE_DIR}/cpp/tests/shm_joint_integration.cpp)
    target_link_libraries(shm_joint_integration PRIVATE calib_shm)
  endif()
  add_executable(shm_channel_tests ${PROJECT_SOURCE_DIR}/cpp/tests/shm_channel_tests.cpp)
  target_link_libraries(shm_channel_tests PRIVATE calib_shm)
  add_test(NAME shm_channel COMMAND shm_channel_tests)
  add_executable(shm_jog_tests ${PROJECT_SOURCE_DIR}/cpp/tests/shm_jog_tests.cpp)
  target_link_libraries(shm_jog_tests PRIVATE calib_shm)
  add_test(NAME shm_jog COMMAND shm_jog_tests)
  add_executable(shm_wire_tests ${PROJECT_SOURCE_DIR}/cpp/tests/shm_wire_tests.cpp)
  target_link_libraries(shm_wire_tests PRIVATE calib_shm)
  add_test(NAME shm_wire35 COMMAND shm_wire_tests ${PROJECT_SOURCE_DIR}/tests/fixtures/shm)
  add_test(NAME shm_abi_check COMMAND shm_abi_check)
  set_tests_properties(shm_wire35 shm_abi_check shm_channel shm_jog PROPERTIES LABELS "shm")
endif()
