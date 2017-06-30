cmake_minimum_required(VERSION 2.8.3)
project(downward_target_tracker)
find_package(catkin REQUIRED
  roscpp
  image_transport
  sensor_msgs
  geometry_msgs
  std_msgs
  message_generation
  ascend_msgs # for LineCounter.msg
)
add_message_files(
  FILES
  image.msg
  info.msg
  tracks.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(CATKIN_DEPENDS message_runtime)
add_definitions("-O3 -mavx")
include_directories()

include(FindPkgConfig)
pkg_search_module(SDL2 REQUIRED sdl2)
include_directories(${SDL2_INCLUDE_DIRS})

add_executable(tracker src/main_drone.cpp)
target_link_libraries(tracker ${catkin_LIBRARIES} turbojpeg v4l2)

add_executable(debugger src/main_debugger.cpp)
target_link_libraries(debugger ${catkin_LIBRARIES} turbojpeg ${SDL2_LIBRARIES} GL)
