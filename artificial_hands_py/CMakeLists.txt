cmake_minimum_required(VERSION 3.0.2)
project(artificial_hands_py)

add_compile_options(-O3 -std=c++11 -fPIC)

find_package(catkin REQUIRED COMPONENTS
  pybind11_catkin
  artificial_hands_base
  artificial_hands_dynamics
  artificial_hands_msgs
)

catkin_package(
  CATKIN_DEPENDS
  artificial_hands_base
  artificial_hands_dynamics
  artificial_hands_msgs
)

catkin_python_setup()

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/artificial_hands_py_base.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set_target_properties(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION} OUTPUT_NAME pyatk PREFIX "" SUFFIX ".cpython-38-x86_64-linux-gnu.so")

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
