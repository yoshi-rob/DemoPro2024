cmake_minimum_required(VERSION 3.0.2)
project(joy_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
  audio_player
)

find_package(SFML 2.5 COMPONENTS audio REQUIRED)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(joy_controller src/joy_controller.cpp)
target_link_libraries(joy_controller
  ${catkin_LIBRARIES}
  sfml-audio
)

add_executable(robot_tf_broadcaster src/robot_tf_broadcaster.cpp)
target_link_libraries(robot_tf_broadcaster
  ${catkin_LIBRARIES}
)