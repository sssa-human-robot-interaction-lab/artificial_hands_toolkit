from cmath import pi

from geometry_msgs.msg import Pose

joint_start = [pi/2,-1.28,2.09,-0.8,pi/2,.0]
""" Hardcoded arm joint angles for start position"""

joint_grasp = joint_start
""" Hardcoded arm joint angles for home position"""

joint_reach = [.0,-pi/2,pi/2,.0,pi/2,.0]
""" Hardcoded arm joint angles for reach position"""

joint_home = joint_reach
""" Hardcoded arm joint angles for home position"""

b = joint_home[0]
s = joint_home[1]
e = joint_home[2]
w1 = joint_home[3]
w2 = joint_home[4]
w3 = joint_home[5]

calibration_joints = [[b,s,e,w1,w2,w3-pi],
                      [b,s,e,w1,w2,w3-pi/2],
                      [b,s,e,w1,w2,w3+pi/2],
                      [b,s,e,w1-pi/2,w2,w3+pi/2],
                      [b,s,e,w1+pi/2,w2,w3+pi/2],
                      [b,s,e,w1,w2,w3]]
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