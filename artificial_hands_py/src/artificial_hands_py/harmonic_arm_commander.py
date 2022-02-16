import numpy as np
from cmath import pi
from time import sleep

import rospy
import tf2_ros, tf2_geometry_msgs, tf.transformations as ts
import moveit_commander, moveit_commander.conversions as cv

from trajectory_msgs.msg import JointTrajectory
from cartesian_control_msgs.msg import CartesianTrajectory, CartesianTrajectoryPoint, FollowCartesianTrajectoryActionGoal

from artificial_hands_py.controller_manager_base import ControllerManagerBase

class HarmonicArmCommander(ControllerManagerBase,moveit_commander.MoveGroupCommander):
  """ Purpose of this class is giving access to cartesian planning in the eef frame 
  (in terms of straight line form pose to pose), moreover the planned biharmonic 
  trajectory is directly forwarded to the ur_hw_interface

  Inherits
  --------
  artificial_hands_py.controller_manager_base.ControllerManagerBase
  
  moveit_commander.MoveGroupCommander
  """

  j_ctrl = 'pos_joint_traj_controller'
  cart_ctrl = 'pose_based_cartesian_traj_controller'
  
  def __init__(self,ns=''):
    super().__init__(ns,{self.j_ctrl : JointTrajectory, self.cart_ctrl : FollowCartesianTrajectoryActionGoal})
    super(ControllerManagerBase,self).__init__('manipulator') 
    self.set_pose_reference_frame('base')
    self.set_end_effector_link('ft_sensor_frame')
    self.dt = 0.01
    self.t_p = 0.0
    self.t_r = 0.0
    self.x_p = 0.0
    self.y_p = 0.0
    self.z_p = 0.0
    self.x_r = 0.0
    self.y_r = 0.0
    self.z_r = 0.0

  def set_estimate_lift(self,tp,tr,xp,yp,zp,xr,yr,zr):
    self.t_p = tp
    self.t_r = tr
    self.x_p = xp
    self.y_p = yp
    self.z_p = zp
    self.x_r = xr
    self.y_r = yr
    self.z_r = zr

  def do_estimate_lift(self):

    def harmonic(h,dt,ts):
      t = np.arange(0,ts+dt,dt)
      y = h*(t/ts - 1/(2*pi)*np.sin(2*pi*t/ts))
      return y,t

    def biharmonic(h,dt,ts):
        t = np.arange(0,ts+dt,dt)
        y = h/2*((1 - np.cos(pi*t/ts)) - 1/4*(1 - np.cos(2*pi*t/ts)))
        return y,t

    def biharmonic2(h,dt,ts):
        t_ = np.arange(0,ts+dt,dt)
        y_ = h/2*((1 - np.cos(pi*t_/ts)) - 1/4*(1 - np.cos(2*pi*t_/ts)))
        t = np.arange(0,2*ts+dt,dt)
        y = np.append(y_,np.flip(y_[:-1]))
        return y,t

    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    pose_ref = self.get_current_pose()
    ref_to_base = tf_buffer.lookup_transform('base',pose_ref.header.frame_id,rospy.Time.now(),timeout=rospy.Duration(1))
    pose = tf2_geometry_msgs.do_transform_pose(pose_ref,ref_to_base)
    pose = cv.pose_to_list(pose.pose)
    pos = pose[0:3]
    quat = pose[3:]
    rot = ts.euler_from_quaternion(quat)
    
    x_pos_arr,t_pos = biharmonic2(self.x_p,self.dt,self.t_p/2)
    y_pos_arr = harmonic(self.y_p,self.dt,self.t_p)[0]
    z_pos_arr = biharmonic2(self.z_p,self.dt,self.t_p/2)[0]
    x_rot_arr,t_rot = biharmonic2(self.x_r,self.dt,self.t_r/2)
    y_rot_arr = biharmonic2(self.y_r,self.dt,self.t_r/2)[0]
    z_rot_arr = biharmonic2(self.z_r,self.dt,self.t_r/2)[0]

    if not (len(x_pos_arr) == len(y_pos_arr) and len(x_pos_arr) == len(z_pos_arr)):
      if not (len(x_rot_arr) == len(y_rot_arr) and len(x_rot_arr) == len(z_rot_arr)):
        rospy.logerr("Error in cartesian points: inconsistent lengths.")
    
    x_pos = np.ndarray.tolist(x_pos_arr)
    y_pos = np.ndarray.tolist(y_pos_arr)
    z_pos = np.ndarray.tolist(z_pos_arr)

    x_rot = np.ndarray.tolist(x_rot_arr)
    y_rot = np.ndarray.tolist(y_rot_arr)
    z_rot = np.ndarray.tolist(z_rot_arr)

    T = ts.quaternion_matrix(quat)
    T[0:3,3] = pos

    for i in range(0,len(x_rot)):
      T_ = T.dot(ts.euler_matrix(x_rot[i],y_rot[i],z_rot[i]))
      x_rot[i],y_rot[i],z_rot[i] = ts.euler_from_matrix(T_)

    for i in range(0,len(x_pos)):
      pos_ = [x_pos[i],y_pos[i],z_pos[i],1] 
      x_pos[i],y_pos[i],z_pos[i] = T.dot((np.array(pos_)))[0:3]

    toolc_to_ft = tf_buffer.lookup_transform('ft_sensor_frame','tool0_controller',rospy.Time.now(),timeout=rospy.Duration(1))
    toolc_pose = cv.transform_to_list(toolc_to_ft.transform)
    toolc_pos = toolc_pose[0:3]
    toolc_quat = toolc_pose[3:]

    x_pos_r = [pos[0]]*len(x_rot)
    y_pos_r = [pos[1]]*len(x_rot)
    z_pos_r = [pos[2]]*len(x_rot)
    for i in range(0,len(x_rot)):
      T0 = ts.euler_matrix(x_rot[i],y_rot[i],z_rot[i])
      T0[0:3,3] = pos
      T1 = ts.quaternion_matrix(toolc_quat)
      T1[0:3,3] = toolc_pos
      T01 = T0.dot(T1)
      x_rot[i],y_rot[i],z_rot[i] = ts.euler_from_matrix(T01)
      x_pos_r[i],y_pos_r[i],z_pos_r[i] = T01[0:3,3]
    
    x_rot_p = [rot[0]]*len(x_pos)
    y_rot_p = [rot[1]]*len(x_pos)
    z_rot_p = [rot[2]]*len(x_pos)
    for i in range(0,len(x_pos)):
      T0 = ts.quaternion_matrix(quat)
      T0[0:3,3] = x_pos[i],y_pos[i],z_pos[i]
      T1 = ts.quaternion_matrix(toolc_quat)
      T1[0:3,3] = toolc_pos
      T01 = T0.dot(T1)
      x_pos[i],y_pos[i],z_pos[i] = T01[0:3,3]
      x_rot_p[i],y_rot_p[i],z_rot_p[i] = ts.euler_from_matrix(T01)
 
    e = 1

    cart_poses = []
    for i in range(e,len(x_rot)-e):
      pose = [x_pos_r[i],y_pos_r[i],z_pos_r[i],x_rot[i],y_rot[i],z_rot[i]]
      cart_poses.append(cv.list_to_pose(pose))
    (plan, fraction) = self.compute_cartesian_path(cart_poses,0.01,0.0)
    if fraction != 1:
      rospy.logerr("ABORTED (rot). Failed to compute a valid path (%.2f %%)",fraction)
      return
    
    cart_poses = []
    for i in range(e,len(x_pos)-e):
      pose = [x_pos[i],y_pos[i],z_pos[i],x_rot_p[i],y_rot_p[i],z_rot_p[i]]
      cart_poses.append(cv.list_to_pose(pose))
    (plan, fraction) = self.compute_cartesian_path(cart_poses,0.01,0.0)
    if fraction != 1:
      rospy.logerr("ABORTED (pos). Failed to compute a valid path (%.2f %%)",fraction)   
      return

    self.switch_to_controller(self.cart_ctrl)

    sleep(1)

    def cart_diff(cart,cart_old):
      cart_ = cart
      cart = cart_.linear.x - cart_old[0]
      cart = cart_.linear.y - cart_old[1]
      cart = cart_.linear.z - cart_old[2]
      cart = cart_.angular.x - cart_old[0]
      cart = cart_.angular.y - cart_old[1]
      cart = cart_.angular.z - cart_old[2]
      cart_old = [cart_.linear.x, cart_.linear.y, cart_.linear.z, cart_.angular.x, cart_.angular.y, cart_.angular.z]
      return cart, cart_old

    vel = [.0,.0,.0,.0,.0,.0]
    acc = [.0,.0,.0,.0,.0,.0]
    cart_goal = FollowCartesianTrajectoryActionGoal()
    for i in range(e,len(x_rot)-e):
      cart_pnt = CartesianTrajectoryPoint() 
      cart_pnt.pose = cv.list_to_pose([x_pos_r[i],y_pos_r[i],z_pos_r[i],x_rot[i],y_rot[i],z_rot[i]])
      cart_pnt.twist, vel = cart_diff(cart_pnt.pose, vel)
      cart_pnt.acceleration, acc = cart_diff(cart_pnt.twist, acc)
      cart_pnt.time_from_start = rospy.Duration.from_sec(t_rot[i])
      cart_goal.goal.trajectory.points.append(cart_pnt)
    self.controller_command(cart_goal)

    sleep(2*self.t_r)

    vel = [.0,.0,.0,.0,.0,.0]
    acc = [.0,.0,.0,.0,.0,.0]
    cart_goal = FollowCartesianTrajectoryActionGoal()
    for i in range(e,len(x_pos)-e):
      cart_pnt = CartesianTrajectoryPoint() 
      cart_pnt.pose = cv.list_to_pose([x_pos[i],y_pos[i],z_pos[i],x_rot_p[i],y_rot_p[i],z_rot_p[i]])
      cart_pnt.twist, vel = cart_diff(cart_pnt.pose, vel)
      cart_pnt.acceleration, acc = cart_diff(cart_pnt.twist, acc)
      cart_pnt.time_from_start = rospy.Duration.from_sec(t_pos[i])
      cart_goal.goal.trajectory.points.append(cart_pnt)
    self.controller_command(cart_goal)

    sleep(2*self.t_p)
    
    self.switch_to_controller(self.j_ctrl) 

