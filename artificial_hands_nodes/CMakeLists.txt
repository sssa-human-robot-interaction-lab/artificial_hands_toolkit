cmake_minimum_required(VERSION 3.0.2)
project(artificial_hands_nodes)

find_package(catkin REQUIRED COMPONENTS artificial_hands_base)

find_package(Boost REQUIRED COMPONENTS system program_options)

catkin_package(
 CATKIN_DEPENDS artificial_hands_base
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(frame_kinematics_node src/frame_kinematics_node.cpp)
add_executable(wrist_dynamics_node src/wrist_dynamics_node.cpp)
add_executable(onrobot_hex_driver_node src/onrobot_hex_driver_node.cpp)

add_dependencies(frame_kinematics_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(wrist_dynamics_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(onrobot_hex_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(frame_kinematics_node ${catkin_LIBRARIES})
target_link_libraries(wrist_dynamics_node ${catkin_LIBRARIES})
target_link_libraries(onrobot_hex_driver_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/dmp_extended_servo_node.py
  scripts/fake_ft_sensor_node.py
  scripts/key_teleop_servo_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
