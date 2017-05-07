cmake_minimum_required(VERSION 2.8.3)
project(downward_target_tracker)
find_package(catkin REQUIRED
  roscpp
  image_transport
  sensor_msgs
  geometry_msgs
  std_msgs
  message_generation
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
add_definitions("-O3")
include_directories()
add_executable(downward_target_tracker asc_tracker_odroid.cpp)
target_link_libraries(downward_target_tracker ${catkin_LIBRARIES} turbojpeg v4l2)
