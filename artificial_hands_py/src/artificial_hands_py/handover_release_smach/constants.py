from cmath import pi

from geometry_msgs.msg import Pose

joint_home = [pi/2,-pi/2,pi/2,.0,pi/2,.0]
""" Hardcoded arm joint angles for home position"""

joint_grasp = [pi,-pi/2,pi/2,.0,pi/2,.0]
""" Hardcoded arm joint angles for home position"""

joint_start = joint_grasp
""" Hardcoded arm joint angles for start reach position"""

joint_reach = joint_home
""" Hardcoded arm joint angles for reach position"""

calibration_joints = []
for c in range(0,6):
  calibration_joints.append(joint_home.copy())
calibration_joints[0][5] -= pi
calibration_joints[1][5] -= pi/2
calibration_joints[2][5] += pi/2
calibration_joints[3][3] -= pi/2
calibration_joints[4][3] += pi/2
""" Hardcoded arm joint angles for long calibration of force/torque """