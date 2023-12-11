cmake_minimum_required(VERSION 3.16)
project(ros1_example_pkg)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  roslint
  sensor_msgs
  std_msgs
)

# this doesn't get the include path
# need to provide that with catkin build ros1_example_pkg --cmake-args -DCMAKE_CXX_FLAGS=-I$DEST/include
# find_package(zenohcxx REQUIRED COMPONENTS zenohcxx_zenohc_lib)
# find_package(zenohcxx REQUIRED COMPONENTS zenohcxx::zenohc::lib)

# finding zenohc and linking zenohc::lib below works
find_package(zenohc REQUIRED)
# zenohcxx and zenohcxx::zenohc::lib doesn't work
# find_package(zenohcxx REQUIRED)

# find_package(PkgConfig)
# pkg_check_modules(zenohcxx REQUIRED)

set(ROSLINT_CPP_OPTS
  "--extensions=cpp,h,hpp" "--filter=-whitespace/braces,-runtime/references,-build/c++11"
)
roslint_cpp()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(generate_image src/generate_image.cpp)
target_link_libraries(generate_image ${catkin_LIBRARIES})

add_executable(zenoh_generate_image src/zenoh_generate_image.cpp)
target_link_libraries(zenoh_generate_image PUBLIC zenohc::lib ${catkin_LIBRARIES})
# target_link_libraries(zenoh_generate_image PUBLIC zenohcxx::zenohc::lib ${catkin_LIBRARIES})

# TODO install scripts

install(TARGETS
  generate_image
  zenoh_generate_image
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
