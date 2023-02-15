cmake_minimum_required(VERSION 3.0.2)
project(local_path_planning_tutorial)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

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
#   nav_msgs#   sensor_msgs#   std_msgs#   visualization_msgs
# )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES local_path_planning_tutorial
#  CATKIN_DEPENDS nav_msgs pcl_ros roscpp rospy sensor_msgs std_msgs visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ppac_robot_pose_broadcaster src/virtual_environment/robot_pose_broadcaster.cpp)
target_link_libraries(ppac_robot_pose_broadcaster  ${catkin_LIBRARIES})
# add_dependencies(ppac_robot_pose_broadcaster ${PROJECT_NAME}_generate_messages_cpp)

add_executable(ppac_virtual_environment src/virtual_environment/virtual_environment.cpp)
target_link_libraries(ppac_virtual_environment  ${catkin_LIBRARIES})
# add_dependencies(ppac_virtual_environment ${PROJECT_NAME}_generate_messages_cpp)

add_executable(potential_method_path_generator src/potential_method/potential_method_path_generator.cpp
                                                src/potential_method/potential_method.cpp)
target_link_libraries(potential_method_path_generator  ${catkin_LIBRARIES})
# add_dependencies(potential_method_path_generator ${PROJECT_NAME}_generate_messages_cpp)

add_executable(dynamic_window_approach_path_generator src/dynamic_window_approach/dwa_path_generator.cpp
                                                      src/dynamic_window_approach/dynamic_window_approach.cpp)
target_link_libraries(dynamic_window_approach_path_generator  ${catkin_LIBRARIES})
# add_dependencies(dynamic_window_approach_path_generator ${PROJECT_NAME}_generate_messages_cpp)