cmake_minimum_required(VERSION 2.8.3)
project(src_task1)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

#set(CMAKE_CXX_FLAGS "-std=c++11x -g -march=native -mno-avx -Werror ${CMAKE_CXX_FLAGS}")
add_definitions(-std=c++11)

# 

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  decision_making
  decision_making_parser
  dynamic_reconfigure
  geometry_msgs
  sensor_msgs
  std_msgs
  moveit_msgs
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  tough_perception_common
  tf
  tf_conversions
  moveit_commander
  moveit_core
  moveit_fake_controller_manager
  moveit_planners_ompl
  moveit_ros_benchmarks
  moveit_ros_benchmarks_gui
  moveit_ros_control_interface
  moveit_ros_manipulation
  moveit_ros_move_group
  moveit_ros_perception
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_robot_interaction
  moveit_ros_visualization
  moveit_ros_warehouse
  moveit_simple_controller_manager
  trac_ik_examples
  trac_ik_kinematics_plugin
  trac_ik_lib
  tough_controller_interface
  val_gui
  tough_footstep
  val_task_common
  navigation_common
  tough_moveit_planners
  tough_control_common
)


## System dependencies are found with CMake's conventions

find_package(OpenCV 2.4 REQUIRED core gpu imgproc calib3d highgui)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7 REQUIRED)


 include_directories(SYSTEM ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

 link_directories(${PCL_LIBRARY_DIRS})
 add_definitions(${PCL_DEFINITIONS})


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/task1_parameters.cfg
 )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES src_task1
  CATKIN_DEPENDS roscpp rospy tough_perception_common dynamic_reconfigure tough_footstep navigation_common
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  include
  cfg/cpp
)

# Declare a C++ library
 add_library(${PROJECT_NAME}
         src/val_task1.cpp
         src/val_task1SM.cpp
         src/handle_detector.cpp
         src/handle_grabber.cpp
         src/move_handle.cpp
         src/pcl_handle_detector.cpp
         src/val_task1_utils.cpp
 )


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(${PROJECT_NAME}_node
            src/val_task1_node.cpp
)

add_dependencies(${PROJECT_NAME}_node
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}_gencfg
 ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}
   ${PROJECT_NAME}
 )

add_executable(handle_detector_node
            src/handle_detector_node.cpp

)



add_dependencies(handle_detector_node
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}_gencfg
 ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(handle_detector_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${PROJECT_NAME}
 )

 add_executable(pcl_handle_detector_node
            src/pcl_handle_detector_node.cpp
)

add_dependencies(pcl_handle_detector_node
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}_gencfg
 ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(pcl_handle_detector_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${PROJECT_NAME}
 )

add_executable(val_circle src/val_circle.cpp)

add_dependencies(val_circle
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS})

target_link_libraries(val_circle
   ${catkin_LIBRARIES}
   val_marker
 )

add_executable(test_next_goal src/test_nextgoal.cpp)

add_dependencies(test_next_goal
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS})

target_link_libraries(test_next_goal
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
 )

add_executable(handle_grabber_node src/handle_grabber_node.cpp)
add_dependencies(handle_grabber_node
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS})

target_link_libraries(handle_grabber_node ${catkin_LIBRARIES} ${PROJECT_NAME})
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above


decision_making_parsing(src/val_task1SM.cpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_val_task1.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


add_executable(test_handle src/test_handle.cpp)
target_link_libraries(test_handle ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(test_move_handle src/test_move_handle.cpp)
target_link_libraries(test_move_handle ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(test_catesian_planner src/test_catesian_planner.cpp)
target_link_libraries(test_catesian_planner ${catkin_LIBRARIES} ${PROJECT_NAME})
