#!/usr/bin/env python3

from time import sleep

import rospy,actionlib, tf.transformations as ts

from artificial_hands_msgs.msg import *
from artificial_hands_py.artificial_hands_py_base import list_to_quat, quat_to_list
from artificial_hands_py.robot_commander.robot_commander import RobotCommander #TO GIVE MIA HAND AN OBJECT

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

  cal_goal.max_accel = 0.4
  cal_goal.max_angaccel = 0.2

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

  obj_rec_goal.max_accel = 0.4
  obj_rec_goal.max_angaccel = 0.2
  # obj_rec_goal.max_vel = 0.8
  # obj_rec_goal.max_angvel = 0.8
  # obj_rec_goal.alpha = 0.2

  r2h_handv_goal = RobotHumanHandoverReachingGoal()

  # continue with handover reaching
  r2h_handv_goal.home.position.x = 0.375
  r2h_handv_goal.home.position.y = -0.28
  r2h_handv_goal.home.position.z = 0.48
  r2h_handv_goal.home.orientation.x = 0.663
  r2h_handv_goal.home.orientation.y = 0.216
  r2h_handv_goal.home.orientation.z = 0.234
  r2h_handv_goal.home.orientation.w = 0.677

  r2h_handv_goal.target.position.x = 0.67
  r2h_handv_goal.target.position.y = 0.44
  r2h_handv_goal.target.position.z = 0.01
  r2h_handv_goal.target.orientation.x = 0.285
  r2h_handv_goal.target.orientation.y = 0.627
  r2h_handv_goal.target.orientation.z = 0.670
  r2h_handv_goal.target.orientation.w = 0.193

  r2h_handv_goal.back.position.x = 0.5
  r2h_handv_goal.back.position.y = 0.2
  r2h_handv_goal.back.position.z = 0.6
  r2h_handv_goal.back.orientation.x = 0
  r2h_handv_goal.back.orientation.y = 0
  r2h_handv_goal.back.orientation.z = 0
  r2h_handv_goal.back.orientation.w = 1

  r2h_handv_goal.max_accel = 1.2
  r2h_handv_goal.max_angaccel = 1.2

  r2h_handv_goal.stop_time = 0.2
  
  r2h_handv_goal.sleep = 1

  robot = RobotCommander()
  rate = rospy.Rate(30)

  # high level control loop: start with a calibration
  ft_cal_cl.send_goal_and_wait(cal_goal)
  while True:
    
    # open the hand and wait for an object to grasp
    robot.hand.open()
    robot.wrist_dyn.start_node()
    robot.wrist_dyn.do_zero()
    sleep(1)
    robot.wrist_dyn.set_trigger_static()
    robot.wrist_dyn.detection.static_contact = False
    while not robot.wrist_dyn.detection.static_contact:
      if rospy.is_shutdown():
        robot.wrist_dyn.stop_loop()
        break
      rate.sleep()

    if rospy.is_shutdown():
      break
      
    sleep(2)
    robot.hand.close()
    
    # do object recognition
    obj_rec_cl.send_goal_and_wait(obj_rec_goal)

    # continue with the handover
    r2h_handv_cl.send_goal_and_wait(r2h_handv_goal)

  rospy.loginfo('bye!')

  rospy.spin()

if __name__ == '__main__':
  main()