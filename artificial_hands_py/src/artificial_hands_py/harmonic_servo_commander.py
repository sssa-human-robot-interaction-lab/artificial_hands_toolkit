from math import floor
import numpy as np
from cmath import pi

import rospy
import moveit_commander
from geometry_msgs.msg import Pose, TwistStamped
from trajectory_msgs.msg import JointTrajectory

from artificial_hands_py.controller_manager_base import ControllerManagerBase

class HarmonicServoCommander(ControllerManagerBase,moveit_commander.MoveGroupCommander):
  """ Purpose of this class is giving access to cartesian planning in the eef frame.
  Using a moveit servo_server node it is possible to move the eef following cycloidal
  or biharmonic motion-laws

  Inherits
  --------
  artificial_hands_py.controller_manager_base.ControllerManagerBase
  
  moveit_commander.MoveGroupCommander

  Attributes
  ----------
  j_traj_ctlr : str
    name of ROS joint controller for trajectory planning and excution
  servo_ctrl : dict
    name of ROS joint controller to feedforward servo command
  dt : float
    servo trajectory time discretization

  Methods
  -------
  servo_delta(goal_time : float, delta_pose : Pose = Pose(), delta_pose_2 : Pose = Pose()):
    move eef from the actual position/orientation by delta_pose, set max allowed displacement in delta_pose_2

  """

  j_traj_ctrl = 'pos_joint_traj_controller'
  servo_ctrl = 'servo_twist_controller'
  dt = 0.002

  def __init__(self,ns=''):
    super().__init__(ns,{self.j_ctrl : JointTrajectory, self.servo_ctrl : TwistStamped})
    super(ControllerManagerBase,self).__init__('manipulator') 

  def servo_delta(self, goal_time : float, delta_pose : Pose = Pose(), delta_pose_2 : Pose = Pose()):

    def harmonic_vel(h : float, ts : float):
      t = np.arange(0,ts,self.dt)
      y = -h/ts*(1 - np.cos(2*pi*t/ts))
      return y

    def biharmonic_vel(h : float, ts : float):
      t = np.arange(0,ts,self.dt)
      y = h/2*pi/ts*(np.sin(pi*t/ts) - 1/2*np.sin(2*pi*t/ts))
      return y

    def biharmonic_vel_2(h : float, ts : float):
        y_ = biharmonic_vel(h,ts/2)
        if np.size(y_) % 2 != 0:
          y = np.append(y_,-1*np.flip(y_[:-1]))
        else:
          y = np.append(y_,-1*np.flip(y_[:-1]))
          y = np.append(y,0)
        return y
    
    self.switch_to_controller(self.j_group_ctrl)

    servo_x = np.ndarray.tolist(harmonic_vel(delta_pose.position.x,goal_time) + biharmonic_vel_2(delta_pose_2.position.x,goal_time))
    servo_y = np.ndarray.tolist(harmonic_vel(delta_pose.position.y,goal_time) + biharmonic_vel_2(delta_pose_2.position.y,goal_time))
    servo_z = np.ndarray.tolist(harmonic_vel(delta_pose.position.z,goal_time) + biharmonic_vel_2(delta_pose_2.position.z,goal_time))

    servo_rate = rospy.Rate(1/self.dt)

    servo_cmd = TwistStamped()
    for c in range(0,len(servo_x)):
      servo_cmd.twist.linear.x = servo_x[c]
      servo_cmd.twist.linear.y = servo_y[c]
      servo_cmd.twist.linear.z = servo_z[c]

      servo_cmd.header.seq = 0
      servo_cmd.header.stamp.secs = 1000000

      self.controller_command(servo_cmd)

      servo_rate.sleep()
    
    self.switch_to_controller(self.j_traj_ctrl)