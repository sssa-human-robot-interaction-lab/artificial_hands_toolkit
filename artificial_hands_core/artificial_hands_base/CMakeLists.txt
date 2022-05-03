cmake_minimum_required(VERSION 3.0.2)
project(artificial_hands_base)

find_package(catkin REQUIRED COMPONENTS roscpp moveit_ros_planning eigen_conversions artificial_hands_msgs)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES artificial_hands_base alglib
 CATKIN_DEPENDS moveit_ros_planning eigen_conversions artificial_hands_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/BaseAlgorithm.hpp
  include/${PROJECT_NAME}/BaseCommon.hpp
  include/${PROJECT_NAME}/BaseFilters.hpp
  include/${PROJECT_NAME}/BaseFTSensor.hpp
  include/${PROJECT_NAME}/BaseMath.hpp
  include/${PROJECT_NAME}/FrameDynamics.hpp
  include/${PROJECT_NAME}/FrameKinematics.hpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} alglib)
