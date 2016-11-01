cmake_minimum_required(VERSION 2.8.3)
project(pose_pkg)

find_package(catkin REQUIRED COMPONENTS roscpp moveit_core moveit_ros_move_group moveit_planners_ompl moveit_ros_visualization visualization_msgs)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
EIGEN3_INCLUDE_DIR include
)

## Declare a C++ library
 add_library(pose_pkg
   src/Reachability_Matrix.cpp
 )
add_executable(Reachability_Matrix src/Reachability_Matrix.cpp)
target_link_libraries(Reachability_Matrix ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(Reachability_Matrix pose_pkg_generate_messages_cpp)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
