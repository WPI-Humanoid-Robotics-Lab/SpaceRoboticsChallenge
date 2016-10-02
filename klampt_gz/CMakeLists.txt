cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(klampt_gz_player)

find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs
)

set(CMAKE_BUILD_TYPE Debug)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES klampt_gz_player
  CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(include ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(klampt_gz_player src/klampt_gz_player.cc)
target_link_libraries(klampt_gz_player ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_executable(klampt_gz_loader src/klampt_gz_loader.cc)
target_link_libraries(klampt_gz_loader ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
