cmake_minimum_required(VERSION 3.0.2)
project(artificial_hands_nodes)

find_package(catkin REQUIRED)

catkin_package(
 CATKIN_DEPENDS artificial_hands_py
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/arm_commander_gui_node.py
  scripts/cartesian_trajectory_generator_gui_node.py
  scripts/cartesian_trajectory_generator_node.py
  scripts/cartesian_trajectory_tf_monitor_node.py
  scripts/fake_ft_sensor_node.py
  scripts/fake_handover_module_test_node.py
  scripts/fake_low_level_engine_node.py
  scripts/mia_hand_commander_gui_node.py
  scripts/mia_hand_commander_node.py
  scripts/robot_commander_gui_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
