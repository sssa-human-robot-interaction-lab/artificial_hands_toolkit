#!/usr/bin/env python3

import rospy

from artificial_hands_msgs.msg import *
from artificial_hands_py.low_level_execution_modules.force_torque_sensor_calibration_module import ForceTorqueSensorCalibrationModule
from artificial_hands_py.low_level_execution_modules.object_grasp_module import ObjectGraspModule
from artificial_hands_py.low_level_execution_modules.object_recognition_module import ObjectRecognitionModule
from artificial_hands_py.low_level_execution_modules.robot_human_handover_reaching_module import RobotHumanHandoverReachingModule

def main():

  rospy.init_node('fake_low_level_engine_node')

  ft_cal_mod = ForceTorqueSensorCalibrationModule()
  obj_grasp_mod = ObjectGraspModule()
  obj_rev_mod = ObjectRecognitionModule()
  r2h_handv_mod = RobotHumanHandoverReachingModule()

  rospy.loginfo("Low-level control engine ready!")

  rospy.spin()

if __name__ == '__main__':
  main()