from cmath import pi

import rospy, actionlib
import tf.transformations as ts
from geometry_msgs.msg import Pose

from artificial_hands_msgs.msg import *
from artificial_hands_py.artificial_hands_py_base import list_to_quat, quat_to_list
from artificial_hands_py.robot_commander.robot_commander import RobotCommander

class ForceTorqueSensorCalibrationModule(RobotCommander):
  sleep_dur = rospy.Duration(0.5)
  calib_feedback = ForceTorqueSensorCalibrationFeedback()
  calib_result = ForceTorqueSensorCalibrationResult()

  def __init__(self) -> None: 
    super().__init__()
  
    self.calib_as = actionlib.SimpleActionServer('/force_torque_sensor_calibration',ForceTorqueSensorCalibrationAction,execute_cb=self.calibration_cb,auto_start=False)

    self.calib_as.start()

  def calibration_cb(self, goal : ForceTorqueSensorCalibrationGoal):
    self.calib_result.success = True
    self.calib_feedback.calib_ok = False
    self.calib_feedback.percentage = 40

    # initialize wrist_dynamics_module
    self.wrist_dyn.subscribe()
    self.wrist_dyn.set_publish()

    # get hand in open position
    self.hand.open()

    # go to home position
    self.arm.set_max_accel(goal.max_accel)
    self.arm.set_max_angaccel(goal.max_angaccel)
    self.arm.set_harmonic_traj_generator()
    self.arm.switch_to_cartesian_controller('cartesian_eik_position_controller')
    self.arm.set_pose_target(goal.home)

    # check current (if any) calibration
    self.wrist_dyn.apply_calibration()
    self.wrist_dyn.start_loop()
    if self.wrist_dyn.check_calibration():
      self.calib_feedback.calib_ok = True
      self.calib_as.set_succeeded(self.calib_result)
      self.wrist_dyn.stop_loop()
      return

    # unload current calibration and start node again
    self.wrist_dyn.stop_loop()
    self.wrist_dyn.start_node(calib = False)

    # c_pose = Pose()
    # c_pose.position = goal.home.position
    c_pose = self.arm.get_current_frame().pose
    
    # align sensor with base frame
    # c_pose.orientation = list_to_quat([0,0,0,1])
    self.add_calibration_point(c_pose)

    # get current sensor orientation and rotate on its own x-axis
    rot = ts.quaternion_matrix(quat_to_list(c_pose.orientation))
    c_pose.orientation = list_to_quat(ts.quaternion_multiply(ts.quaternion_about_axis(pi/2,rot[:,0]),quat_to_list(c_pose.orientation)))
    self.add_calibration_point(c_pose)

    # now rotate on its own z-axis
    rot = ts.quaternion_matrix(quat_to_list(c_pose.orientation))
    c_pose.orientation = list_to_quat(ts.quaternion_multiply(ts.quaternion_about_axis(-pi/2,rot[:,2]),quat_to_list(c_pose.orientation)))
    self.add_calibration_point(c_pose)
    for c in range(0,3):
      c_pose.orientation = list_to_quat(ts.quaternion_multiply(ts.quaternion_about_axis(pi/2,rot[:,2]),quat_to_list(c_pose.orientation)))
      self.add_calibration_point(c_pose)
    
    # align sensor respect to the base frame (z down)
    rot = ts.quaternion_matrix(quat_to_list(c_pose.orientation))
    c_pose.orientation = list_to_quat(ts.quaternion_multiply(ts.quaternion_about_axis(-pi/2,rot[:,0]),quat_to_list(c_pose.orientation)))
    self.add_calibration_point(c_pose)

    # return to home (z up, in two steps to be sure to move back)
    rot = ts.quaternion_matrix(quat_to_list(c_pose.orientation))
    c_pose.orientation = list_to_quat(ts.quaternion_multiply(ts.quaternion_about_axis(pi/2,rot[:,0]),quat_to_list(c_pose.orientation)))
    self.arm.set_pose_target(c_pose)
    rot = ts.quaternion_matrix(quat_to_list(c_pose.orientation))
    c_pose.orientation = list_to_quat(ts.quaternion_multiply(ts.quaternion_about_axis(pi/2,rot[:,0]),quat_to_list(c_pose.orientation)))
    self.arm.set_pose_target(c_pose)
    rot = ts.quaternion_matrix(quat_to_list(c_pose.orientation))
    for c in range(0,2):
      c_pose.orientation = list_to_quat(ts.quaternion_multiply(ts.quaternion_about_axis(-pi/2,rot[:,2]),quat_to_list(c_pose.orientation)))
      self.arm.set_pose_target(c_pose)

    # estimate calibration and get wrist_dynamics_module to idle
    self.wrist_dyn.estimate_calibration()
    self.wrist_dyn.stop_loop()

    # stop controllers
    self.arm.wait_for_trajectory_monitor()
    self.arm.pause_all_controllers()
    self.calib_as.set_succeeded(self.calib_result)
    
  def add_calibration_point(self, pose : Pose): 
    self.arm.set_pose_target(pose)
    self.wrist_dyn.set_save_calibration()
    rospy.sleep(self.sleep_dur)
    self.wrist_dyn.set_publish()
    self.calib_feedback.percentage += 10
    self.calib_as.publish_feedback(self.calib_feedback)

def main():

  rospy.init_node('force_torque_sensor_calibration_module_node')

  ft_cal_mod = ForceTorqueSensorCalibrationModule()

  rospy.loginfo('FT sensor calibration module ready!')

  rospy.spin()

if __name__ == '__main__':
  main()