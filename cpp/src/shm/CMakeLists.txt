add_library(calib_shm STATIC wire35.cpp)
add_library(casbot_calib::shm ALIAS calib_shm)
set_target_properties(calib_shm PROPERTIES POSITION_INDEPENDENT_CODE ON EXPORT_NAME shm)
target_compile_features(calib_shm PUBLIC cxx_std_17)
target_include_directories(calib_shm PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/cpp/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_options(calib_shm PRIVATE -Wall -Wextra -Wpedantic)

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
  add_executable(shm_wire_tests ${PROJECT_SOURCE_DIR}/cpp/tests/shm_wire_tests.cpp)
  target_link_libraries(shm_wire_tests PRIVATE calib_shm)
  add_test(NAME shm_wire35 COMMAND shm_wire_tests ${PROJECT_SOURCE_DIR}/tests/fixtures/shm)
  add_test(NAME shm_abi_check COMMAND shm_abi_check)
  set_tests_properties(shm_wire35 shm_abi_check PROPERTIES LABELS "shm")
endif()
