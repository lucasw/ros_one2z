cmake_minimum_required(VERSION 3.0.2)
project(ros1_example_pkg)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  roslint
  sensor_msgs
)

set(ROSLINT_CPP_OPTS
  "--extensions=cpp,h,hpp" "--filter=-whitespace/braces,-runtime/references,-build/c++11"
)
roslint_cpp()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(generate_image src/generate_image.cpp)
target_link_libraries(generate_image ${catkin_LIBRARIES})

# TODO install scripts

install(TARGETS
  generate_image
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
