cmake_minimum_required(VERSION 3.0.2)
project(audio_player)

find_package(catkin REQUIRED)
find_package(SFML 2.5 COMPONENTS audio REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES audio_player
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SFML_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  sfml-audio
)

add_executable(main
  src/main.cpp
)

target_link_libraries(main
  ${PROJECT_NAME}
)