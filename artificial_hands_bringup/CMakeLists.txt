cmake_minimum_required(VERSION 3.0.2)
project(artificial_hands_bringup)

find_package(catkin REQUIRED)

catkin_package(
)

include_directories(
)

catkin_install_python(PROGRAMS
  scripts/tty_low_latency.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
