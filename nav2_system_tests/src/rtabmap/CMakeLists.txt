nav2_add_test(test_rtabmap
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_rtabmap_launch.py"
  # A passing run takes ~80 s. The headroom is so that a failing run hits the
  # test's own wait_for timeouts and reports which stage broke, rather than
  # being killed by ctest with no diagnosis.
  TIMEOUT 420
  ENV
    TEST_EXECUTABLE=${CMAKE_CURRENT_SOURCE_DIR}/test_rtabmap_node.py
)
