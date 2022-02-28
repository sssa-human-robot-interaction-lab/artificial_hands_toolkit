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

joint_calib = []
joint_calib = joint_home.copy()
joint_calib[3] = -0.6
joint_calib[5] = pi/4
""" Hardcoded arm joint angles for reach position"""

calibration_joints = []
for c in range(0,6):
  calibration_joints.append(joint_home.copy())
calibration_joints[0][5] -= pi
calibration_joints[1][5] -= pi/2
calibration_joints[2][5] += pi/2
calibration_joints[3][3] -= pi/2
calibration_joints[4][3] += pi/2
""" Hardcoded arm joint angles for fast calibration of force/torque """

goal_time = 1.12
""" Goal time for object recognition trajectory """

goal = Pose()
goal.position.y = 0.2
""" Pose delta for object recognition trajectory """

goal_2 = Pose()
goal_2.position.x = 0.032
goal_2.position.z = 0.032
""" Pose delta_2 for object recognition trajectory """