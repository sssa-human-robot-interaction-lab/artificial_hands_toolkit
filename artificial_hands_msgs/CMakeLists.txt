cmake_minimum_required(VERSION 3.0.2)
project(artificial_hands_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  cartesian_control_msgs
  trajectory_msgs
  message_generation
  actionlib_msgs
  dynamic_reconfigure
)

add_message_files(
  FILES
  CartesianTrajectoryPointStamped.msg
  FrameKinematics.msg
  FrameKinematicsStamped.msg
  WristDynamics.msg
  WristDynamicsStamped.msg
  Detection.msg
  DetectionStamped.msg
)

add_service_files(
  FILES
  FrameKinematicsCommand.srv
  WristDynamicsCommand.srv
)

add_action_files(
  FILES
  TrajectoryGeneration.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  cartesian_control_msgs
  trajectory_msgs
  actionlib_msgs
)

generate_dynamic_reconfigure_options(
  cfg/ProcessVariance.cfg
  cfg/WristFTProprioception.cfg
)


catkin_package(
   CATKIN_DEPENDS 
   message_runtime
)

include_directories(
${catkin_INCLUDE_DIRS}
)
