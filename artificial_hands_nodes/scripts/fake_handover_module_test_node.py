#!/usr/bin/env python3

from time import sleep

import rospy,actionlib, tf.transformations as ts

from artificial_hands_msgs.msg import *
from artificial_hands_py.artificial_hands_py_base import list_to_quat, quat_to_list, pose_copy
from artificial_hands_py.robot_commander.robot_commander import RobotCommander #TO GIVE MIA HAND AN OBJECT

def main():

  rospy.init_node('fake_handover_module_test_node')

  ft_cal_cl = actionlib.SimpleActionClient('/force_torque_sensor_calibration',ForceTorqueSensorCalibrationAction)
  obj_grasp_cl = actionlib.SimpleActionClient('/object_grasp',ObjectGraspAction)
  obj_rec_cl = actionlib.SimpleActionClient('/object_recognition',ObjectRecognitionAction)
  r2h_handv_cl = actionlib.SimpleActionClient('/robot_to_human_handover_reaching',RobotHumanHandoverReachingAction)

  ft_cal_cl.wait_for_server()
  obj_grasp_cl.wait_for_server()
  obj_rec_cl.wait_for_server()
  r2h_handv_cl.wait_for_server()

  # set position for sensor_calibration
  ft_cal_goal = ForceTorqueSensorCalibrationGoal()
  ft_cal_goal.home.position.x = -0.486
  ft_cal_goal.home.position.y = -0.108
  ft_cal_goal.home.position.z = 0.223
  ft_cal_goal.home.orientation = list_to_quat([0.5,0.5,0.5,0.5])
  ft_cal_goal.max_accel = 0.4
  ft_cal_goal.max_angaccel = 0.4

  # grasp the object
  obj_grasp_goal = ObjectGraspGoal()
  obj_grasp_goal.home.position.x = 0.593
  obj_grasp_goal.home.position.y = -0.131
  obj_grasp_goal.home.position.z = -0.066
  obj_grasp_goal.home.orientation.x = -0.255
  obj_grasp_goal.home.orientation.y = 0.674
  obj_grasp_goal.home.orientation.z = -0.695
  obj_grasp_goal.home.orientation.w = 0.253
  obj_grasp_goal.preshape.data = [0.3,0.3,0.0]
  obj_grasp_goal.target = pose_copy(obj_grasp_goal.home)
  obj_grasp_goal.target.position.y = -0.342
  obj_grasp_goal.shape.data = [1.2,1.2,0.6]
  obj_grasp_goal.back = pose_copy(obj_grasp_goal.target)
  obj_grasp_goal.back.position.z = 0.010
  obj_grasp_goal.max_vel = 0.4
  obj_grasp_goal.max_angvel = 0.4
  obj_grasp_goal.alpha = 0.2

  # start object_recognition while positioning for reaching
  obj_rec_goal = ObjectRecognitionGoal()
  obj_rec_goal.home = pose_copy(obj_grasp_goal.back)
  obj_rec_goal.target.position.x = 0.95
  obj_rec_goal.target.position.y = -0.07
  obj_rec_goal.target.position.z = 0.25
  obj_rec_goal.target.orientation.x = 0.017
  obj_rec_goal.target.orientation.y = 0.753
  obj_rec_goal.target.orientation.z = 0.407
  obj_rec_goal.target.orientation.w = 0.516
  obj_rec_goal.max_accel = 0.4
  obj_rec_goal.max_angaccel = 0.2

  # continue with reaching
  r2h_handv_goal = RobotHumanHandoverReachingGoal()
  r2h_handv_goal.home = pose_copy(obj_rec_goal.target)
  r2h_handv_goal.target = pose_copy(r2h_handv_goal.home)
  r2h_handv_goal.target.position.x = 0.66
  r2h_handv_goal.target.position.y = 0.58
  r2h_handv_goal.target.position.z = 0.21
  r2h_handv_goal.back = pose_copy(ft_cal_goal.home)
  r2h_handv_goal.max_accel = 0.4
  r2h_handv_goal.max_angaccel = 0.4
  r2h_handv_goal.stop_time = 0.2
  r2h_handv_goal.sleep = 1

  # high level control loop
  while True:

    # start or check ft sensor calibration
    ft_cal_cl.send_goal_and_wait(ft_cal_goal)
    
    # grasp
    obj_grasp_cl.send_goal_and_wait(obj_grasp_goal)
    
    # do object recognition
    obj_rec_cl.send_goal_and_wait(obj_rec_goal)

    # continue with the handover
    r2h_handv_cl.send_goal_and_wait(r2h_handv_goal)

    break

  rospy.loginfo('bye!')

  rospy.spin()

if __name__ == '__main__':
  main()