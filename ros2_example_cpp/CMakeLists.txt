cmake_minimum_required(VERSION 3.8)
project(ros2_example_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# these are needed for mcap
find_package(PkgConfig)
pkg_check_modules(LZ4 REQUIRED)
find_library(LZ4_LIBRARY
  NAMES lz4 liblz4
)

find_package(zstd REQUIRED)

add_executable(minimal_publisher src/minimal_publisher.cpp)
ament_target_dependencies(minimal_publisher rclcpp std_msgs)

add_executable(play_mcap src/play_mcap.cpp)
ament_target_dependencies(play_mcap rclcpp)
# these can't go into ament macro
target_link_libraries(play_mcap
  ${LZ4_LIBRARY}
  zstd::libzstd_shared
)

install(TARGETS
  minimal_publisher
  play_mcap
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
