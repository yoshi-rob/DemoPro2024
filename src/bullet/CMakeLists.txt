cmake_minimum_required(VERSION 3.0.2)
project(bullet)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
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

add_executable(bullet_launcher src/bullet_launcher.cpp)
target_link_libraries(bullet_launcher
  ${catkin_LIBRARIES}
)

add_executable(death_judge src/death_judge.cpp)
target_link_libraries(death_judge
  ${catkin_LIBRARIES}
  sfml-audio
)
