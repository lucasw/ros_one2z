cmake_minimum_required(VERSION 3.16)
project(one2z)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  topic_tools
)

set(ROSLINT_CPP_OPTS
  "--extensions=cpp,h,hpp" "--filter=-whitespace/braces,-runtime/references,-build/c++11"
)
roslint_cpp()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# find_package(LZ4 REQUIRED)
find_package(PkgConfig)
pkg_check_modules(LZ4 REQUIRED)
find_library(LZ4_LIBRARY
  NAMES lz4 liblz4
)

find_package(zstd REQUIRED)

add_executable(mcap_recorder src/mcap_recorder.cpp)
target_link_libraries(mcap_recorder
  ${LZ4_LIBRARY}
  ${catkin_LIBRARIES}
  zstd::libzstd_shared
)

# TODO install scripts

install(PROGRAMS
  scripts/ros1_play_mcap.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  mcap_recorder
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
