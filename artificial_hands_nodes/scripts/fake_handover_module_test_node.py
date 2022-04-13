#!/usr/bin/env python3

import rospy,actionlib

from artificial_hands_msgs.msg import *

def main():

  rospy.init_node('fake_handover_module_test_node')

  ft_cal_cl = actionlib.SimpleActionClient('/force_torque_sensor_calibration',ForceTorqueSensorCalibrationAction)
  obj_rec_cl = actionlib.SimpleActionClient('/object_recognition',ObjectRecognitionAction)
  r2h_handv_cl = actionlib.SimpleActionClient('/robot_to_human_handover_reaching',RobotHumanHandoverReachingAction)

  ft_cal_cl.wait_for_server()
  obj_rec_cl.wait_for_server()
  r2h_handv_cl.wait_for_server()

  cal_goal = ForceTorqueSensorCalibrationGoal()

  # set position for sensor_calibration
  cal_goal.home.position.x = 0.5
  cal_goal.home.position.y = 0.2
  cal_goal.home.position.z = 0.5
  cal_goal.home.orientation.x = 0
  cal_goal.home.orientation.y = 0
  cal_goal.home.orientation.z = 0
  cal_goal.home.orientation.w = 1

  cal_goal.max_accel = 0.2
  cal_goal.max_angaccel = 0.2

  ft_cal_cl.send_goal_and_wait(cal_goal)

  for c in range(0,3):

    obj_rec_goal = ObjectRecognitionGoal()

    # start object_recognition just a little below
    obj_rec_goal.home.position.x = 0.5
    obj_rec_goal.home.position.y = 0.2
    obj_rec_goal.home.position.z = 0.3
    obj_rec_goal.home.orientation.x = 0
    obj_rec_goal.home.orientation.y = 0
    obj_rec_goal.home.orientation.z = 0
    obj_rec_goal.home.orientation.w = 1

    obj_rec_goal.target.position.x = 0.5
    obj_rec_goal.target.position.y = 0.2
    obj_rec_goal.target.position.z = 0.6
    obj_rec_goal.target.orientation.x = 1
    obj_rec_goal.target.orientation.y = 1
    obj_rec_goal.target.orientation.z = 1
    obj_rec_goal.target.orientation.w = 1

    obj_rec_goal.max_accel = 0.2
    obj_rec_goal.max_angaccel = 0.2

    obj_rec_cl.send_goal_and_wait(obj_rec_goal)

    r2h_handv_goal = RobotHumanHandoverReachingGoal()

    # continue with handover reaching
    r2h_handv_goal.home.position.x = 0.5
    r2h_handv_goal.home.position.y = 0.2
    r2h_handv_goal.home.position.z = 0.6
    r2h_handv_goal.home.orientation.x = 1
    r2h_handv_goal.home.orientation.y = 1
    r2h_handv_goal.home.orientation.z = 1
    r2h_handv_goal.home.orientation.w = 1

    r2h_handv_goal.target.position.x = 0.88
    r2h_handv_goal.target.position.y = 0.06
    r2h_handv_goal.target.position.z = 0.35
    r2h_handv_goal.target.orientation.x = 1
    r2h_handv_goal.target.orientation.y = 1
    r2h_handv_goal.target.orientation.z = 1
    r2h_handv_goal.target.orientation.w = 1

    r2h_handv_goal.back.position.x = 0.88
    r2h_handv_goal.back.position.y = 0.06
    r2h_handv_goal.back.position.z = 0.6
    r2h_handv_goal.back.orientation.x = 1
    r2h_handv_goal.back.orientation.y = 1
    r2h_handv_goal.back.orientation.z = 1
    r2h_handv_goal.back.orientation.w = 1

    r2h_handv_goal.max_accel = 0.8
    r2h_handv_goal.max_angaccel = 0.8
    
    r2h_handv_goal.sleep = 1

    r2h_handv_cl.send_goal_and_wait(r2h_handv_goal)

  rospy.loginfo('bye!')

  rospy.spin()

if __name__ == '__main__':
  main()