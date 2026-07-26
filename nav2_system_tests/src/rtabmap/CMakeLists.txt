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

nav2_add_test(test_rtabmap_localization
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_rtabmap_localization_launch.py"
  TIMEOUT 420
  ENV
    TEST_EXECUTABLE=${CMAKE_CURRENT_SOURCE_DIR}/test_rtabmap_localization_node.py
)

# Unlike the two above, this one runs in Gazebo rather than nav2_loopback_sim,
# because it needs a rendered camera. It is independent of them: its own world,
# its own robot, its own database, so it is not part of the fixture below.
nav2_add_test(test_rtabmap_visual
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_rtabmap_visual_launch.py"
  # A passing run is ~200 s: about 30 s to bring Gazebo and the bridges up, then
  # three ~44 s laps. The headroom lets a failing run reach its own wait_for
  # timeouts and say which stage broke.
  TIMEOUT 600
  ENV
    TEST_EXECUTABLE=${CMAKE_CURRENT_SOURCE_DIR}/test_rtabmap_visual_node.py
)

# The localization test replays the database the mapping test writes, so it has
# to run after it and only if it passed. A fixture expresses that to ctest;
# plain test ordering would not, and the two must never run concurrently since
# they share the database file.
if(TARGET test_rtabmap OR TEST test_rtabmap)
  set_tests_properties(test_rtabmap
    PROPERTIES FIXTURES_SETUP rtabmap_database)
  set_tests_properties(test_rtabmap_localization
    PROPERTIES FIXTURES_REQUIRED rtabmap_database RUN_SERIAL TRUE)
endif()
