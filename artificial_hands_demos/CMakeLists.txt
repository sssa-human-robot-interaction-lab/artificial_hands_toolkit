cmake_minimum_required(VERSION 3.0.2)
project(artificial_hands_demos)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS artificial_hands_py
)

catkin_install_python(PROGRAMS
  scripts/ft_calib_node.py
  scripts/handover_release_smach_node.py
  scripts/proprioception_tests_gui_node.py
  scripts/proprioception_tests_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
