cmake_minimum_required(VERSION 3.8)
project(emcl2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(emcl2_node
  src/Mcl.cpp
  src/ExpResetMcl2.cpp
  src/Particle.cpp
  src/OdomModel.cpp
  src/Pose.cpp
  src/Scan.cpp
  src/LikelihoodFieldMap.cpp
  src/emcl2_node.cpp
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_scan_updates
    test/test_scan_updates.cpp
    src/Mcl.cpp
    src/ExpResetMcl2.cpp
    src/Particle.cpp
    src/OdomModel.cpp
    src/Pose.cpp
    src/Scan.cpp
    src/LikelihoodFieldMap.cpp
  )
  target_include_directories(test_scan_updates PRIVATE include)
  ament_target_dependencies(test_scan_updates rclcpp nav_msgs sensor_msgs)
  add_test(NAME duplicate_scan_preserves_belief COMMAND test_scan_updates duplicate)
  add_test(NAME subsecond_scans_are_distinct COMMAND test_scan_updates subsecond)
  add_test(NAME resampling_resets_weights COMMAND test_scan_updates resampling)
  add_test(NAME initial_covariance_spreads_particles COMMAND test_scan_updates covariance)
  add_test(NAME initial_covariance_preserves_correlation COMMAND test_scan_updates correlated_covariance)
  add_test(NAME initial_covariance_rejects_invalid_input COMMAND test_scan_updates invalid_covariance)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS
  test/test.bash
  test/test_gui.bash
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  test
)
