# artificial_hands_py

## cartesian_trajectory_generator

Implementation of a cartesian trajectory generator for point-to-point arm control. Different trajectory profiles are possible. Moreover, through dedicated plugins online trajectory generation is possible (i.e. continuos update of arm target pose).

## low_level_execution_modules

Task oriented low-level modules, each of them has control of the robot (arm + end-effector) and connection to sensors, exploiting the robot_commander interface.

## robot_commander

Interface to control the whole robot trough simple python functions to set pose targets for the manipulator, send end-effector joint commands, manage sensors.