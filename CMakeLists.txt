# SET THIS TO 1 TO DEBUG THE GRID DETECTOR LOCALLY WITH VDB
# (REQUIRES SDL2)
SET(ASC_GRID_DEBUG 0)

cmake_minimum_required(VERSION 2.8.3)
project(downward_target_tracker)
find_package(catkin REQUIRED
  roscpp
  geometry_msgs
  std_msgs
  message_generation
  ascend_msgs # for LineCounter.msg
  image_transport
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
add_definitions("-O3 -mavx -std=c++11")
include_directories(${catkin_INCLUDE_DIRS})

if (ASC_GRID_DEBUG)

  add_definitions("-DASC_GRID_DEBUG")
  add_executable(tracker src/main_drone.cpp)
  add_dependencies(tracker downward_target_tracker_generate_messages_cpp
  )
  target_link_libraries(tracker ${catkin_LIBRARIES} turbojpeg v4l2 ${SDL2_LIBRARIES} GL)

else (ASC_GRID_DEBUG)

  add_executable(tracker src/main_drone.cpp)
  add_dependencies(tracker downward_target_tracker_generate_messages_cpp
  )
  target_link_libraries(tracker ${catkin_LIBRARIES} turbojpeg v4l2)

endif (ASC_GRID_DEBUG)

SET(SDL2_FOUND 0)
include(FindPkgConfig)
pkg_search_module(SDL2 sdl2)

if (SDL2_FOUND)

  include_directories(${SDL2_INCLUDE_DIRS})
  add_executable(debugger src/main_debugger.cpp)
  add_dependencies(debugger downward_target_tracker_generate_messages_cpp
  )
  target_link_libraries(debugger ${catkin_LIBRARIES} turbojpeg ${SDL2_LIBRARIES} GL)

endif (SDL2_FOUND)

