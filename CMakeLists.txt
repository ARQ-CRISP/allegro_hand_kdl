cmake_minimum_required(VERSION 2.8.3)
project(allegro_hand_kdl)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  roscpp
  tf2_ros
  kdl_control_tools
  kdl_conversions
  eigen_conversions
  std_msgs
  message_generation
  actionlib_msgs
)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

# For YouCompleteMe
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  PoseRequest.srv
)

add_action_files(
  DIRECTORY action
  FILES
  PoseControl.action
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs actionlib_msgs geometry_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS
  kdl_parser
  roscpp
  tf2_ros
  kdl_conversions
  kdl_control_tools
  eigen_conversions
  message_runtime
  actionlib_msgs
 DEPENDS EIGEN3 orocos_kdl
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/allegro_kdl_config.cpp
  src/kinematics.cpp
  src/inverse_dynamics.cpp
  src/cartesian_force.cpp
  src/joint_position_control.cpp
  src/cartesian_position_control.cpp
  src/joint_trajectory_control.cpp
  src/cartesian_trajectory_control.cpp
  include/allegro_hand_kdl/template_trajectory_control.h
  include/allegro_hand_kdl/pose_control_client.h
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(envelop_force
  src/nodes/envelop_force.cpp
  src/cartesian_force.cpp
  src/allegro_kdl_config.cpp
)

add_executable(fk_publisher
  src/nodes/fk_publisher.cpp
  src/kinematics.cpp
  src/allegro_kdl_config.cpp
)

add_executable(control_force
  src/nodes/force_control.cpp
  src/cartesian_force.cpp
  src/allegro_kdl_config.cpp
)

add_executable(gravity_compensate
  src/nodes/gravity_compensater.cpp
  src/inverse_dynamics.cpp
  src/allegro_kdl_config.cpp
)

add_executable(base_pose_publisher
  src/nodes/base_pose_publisher.cpp
)


add_executable(joint_pose_server
  src/servers/joint_pose_server.cpp
  src/joint_position_control.cpp
  src/allegro_kdl_config.cpp
  src/inverse_dynamics.cpp
)

add_executable(cartesian_pose_server
  src/servers/cartesian_pose_server.cpp
  src/cartesian_position_control.cpp
  src/allegro_kdl_config.cpp
  src/kinematics.cpp
  src/cartesian_force.cpp
)

add_executable(pose_action_server
  src/servers/pose_action_server.cpp
  src/allegro_kdl_config.cpp
  src/kinematics.cpp
  src/cartesian_position_control.cpp
  src/joint_position_control.cpp
  src/cartesian_force.cpp
)

add_executable(fake_control
  src/nodes/fake_control.cpp
)

add_executable(pose_client
  src/nodes/pose_client.cpp
  src/allegro_kdl_config.cpp
)

## Test execulables
add_executable(kinematics_test
  test/kinematics_node.cpp
  src/allegro_kdl_config.cpp
  src/kinematics.cpp
  src/inverse_dynamics.cpp
)

add_executable(inverse_dynamics_test
  test/inverse_dynamics_node.cpp
  src/inverse_dynamics.cpp
  src/allegro_kdl_config.cpp
)

add_executable(single_joint_control_test
  test/single_joint_control_node.cpp
  src/allegro_kdl_config.cpp
  src/inverse_dynamics.cpp
)

# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(control_force ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(joint_pose_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cartesian_pose_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pose_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pose_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(envelop_force
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

target_link_libraries(fk_publisher
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

target_link_libraries(control_force
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

target_link_libraries(gravity_compensate
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

target_link_libraries(base_pose_publisher
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

target_link_libraries(pose_action_server
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

target_link_libraries(cartesian_pose_server
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

target_link_libraries(joint_pose_server
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

target_link_libraries(fake_control
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

target_link_libraries(pose_client
  ${catkin_LIBRARIES}
)

target_link_libraries(kinematics_test
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

target_link_libraries(inverse_dynamics_test
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

target_link_libraries(single_joint_control_test
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

#############
## Install ##
#############



#############
## Testing ##
#############
