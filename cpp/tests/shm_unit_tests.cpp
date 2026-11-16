int test_shm_channel_tests();
int test_shm_limits_tests();
int test_shm_clock_tests();
int test_shm_jog_tests();
int test_shm_wire_tests(int argc, char** argv);
int test_shm_reference_tests();
int test_shm_trical_tests();
int main(int argc, char** argv) {
  if (test_shm_channel_tests()) return 1;
  if (test_shm_limits_tests()) return 1;
  if (test_shm_clock_tests()) return 1;
  if (test_shm_jog_tests()) return 1;
  if (test_shm_wire_tests(argc, argv)) return 1;
#ifdef CALIB_TEST_REFERENCE
  if (test_shm_reference_tests()) return 1;
  if (test_shm_trical_tests()) return 1;
#endif
}
