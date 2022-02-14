#! /usr/bin/env python3

from cmath import pi
from time import sleep
from abc import ABC
import numpy as np
from threading import Thread

import rospy, tf2_ros, tf2_geometry_msgs
import tf.transformations as ts
import moveit_commander
import moveit_commander.conversions as cv
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
from rqt_controller_manager.controller_manager import *
from cartesian_control_msgs.msg import *

from artificial_hands_msgs.msg import *
from artificial_hands_msgs.srv import WristCommand

class MiaHandCommander():
  """ Simple commander for Mia hand: grasps using ROS control """

  def __init__(self,ns=''):
    ld_ser = rospy.ServiceProxy(ns+'/controller_manager/load_controller',LoadController)
    ld_ser('j_index_fle_position_controller')
    ld_ser('j_index_fle_velocity_controller')
    ld_ser('j_thumb_fle_position_controller')
    ld_ser('j_mrl_fle_position_controller')
    ld_ser('j_mrl_fle_velocity_controller')

    self.sw_ser = rospy.ServiceProxy(ns+'/controller_manager/switch_controller',SwitchController) 
    self.sw_ser(['j_thumb_fle_position_controller','j_index_fle_velocity_controller','j_mrl_fle_velocity_controller'],['mia_hand_vel_trajectory_controller','mia_hand_pos_trajectory_controller'],1,False,5)
    
    self.thu_pos = rospy.Publisher(ns+'/j_thumb_fle_position_controller/command',Float64,queue_size=1000)
    self.index_pos = rospy.Publisher(ns+'/j_index_fle_position_controller/command',Float64,queue_size=1000)
    self.mrl_pos = rospy.Publisher(ns+'/j_mrl_fle_position_controller/command',Float64,queue_size=1000)

    self.index_vel = rospy.Publisher(ns+'/j_index_fle_velocity_controller/command',Float64,queue_size=1000)
    self.mrl_vel = rospy.Publisher(ns+'/j_mrl_fle_velocity_controller/command',Float64,queue_size=1000)

    self.close_cyl_ser = rospy.Service('/close_cyl',Empty,self.close_cyl_callback)
    self.open_cyl_ser = rospy.Service('/open_cyl',Empty,self.open_cyl_callback)

    self.j = Float64()

    self.switch_to_open()                                        

  def set_thu_pos(self,pos):
    self.j.data = pos
    self.thu_pos.publish(self.j)
  
  def set_index_pos(self,pos):
    self.j.data = pos
    self.index_pos.publish(self.j)
  
  def set_mrl_pos(self,pos):
    self.j.data = pos
    self.mrl_pos.publish(self.j)
  
  def set_index_vel(self,vel):
    self.j.data = vel
    self.index_vel.publish(self.j)
  
  def set_mrl_vel(self,vel):
    self.j.data = vel
    self.mrl_vel.publish(self.j)
  
  def switch_to_open(self):
    self.sw_ser(['j_index_fle_position_controller','j_mrl_fle_position_controller'],['j_index_fle_velocity_controller','j_mrl_fle_velocity_controller'],1,False,5)
  
  def switch_to_close(self):
    self.sw_ser(['j_index_fle_velocity_controller','j_mrl_fle_velocity_controller'],['j_index_fle_position_controller','j_mrl_fle_position_controller'],1,False,5)
  
  def open_pin(self):
    self.set_thu_pos(0.45)
    self.set_index_pos(0.5)
    sleep(2)
    self.switch_to_close()
    
  def close_pin(self):
    self.set_index_vel(1.0)
    sleep(3)
    self.set_index_vel(0.0)
    sleep(1)
    self.switch_to_open()

  def open_cyl(self):
    self.set_thu_pos(0.45)
    self.set_index_pos(0.5)
    self.set_mrl_pos(0.5)
    sleep(2)
    self.switch_to_close()
    
  def close_cyl(self):
    self.set_index_vel(1.0)
    self.set_mrl_vel(0.6)
    sleep(3)
    self.set_index_vel(0.0)
    self.set_mrl_vel(0.0)
    sleep(1)
    self.switch_to_open()
  
  def open_cyl_callback(self,msg):
    self.open_cyl()
    return []

  def close_cyl_callback(self,msg):
    self.close_cyl()
    return []

class MiaHandMoveitCommander(moveit_commander.MoveGroupCommander):
  """ Simple commander for Mia hand: grasps using Moveit """
  def __init__(self,ns=''):
    super().__init__("hand")

  def open_cyl(self):
    self.go([.4,.4,.2],wait=True)
    self.stop()
    sleep(1)

  def close_cyl(self):
    self.go([.99,1.02,.46],wait=True)
    self.stop()
    sleep(1)


class ArmCommander(moveit_commander.MoveGroupCommander):
  """ Simple robitc commander for Mia hand: grasps using Moveit """
  
  def __init__(self,ns=''):
    super().__init__("manipulator")
    self.j_ctrl = ['pos_joint_traj_controller']
    self.cart_ctrl = ['pose_based_cartesian_traj_controller']
    self.sw_ser = rospy.ServiceProxy('/controller_manager/switch_controller',SwitchController) 
    self.traj_pub = rospy.Publisher('/pose_based_cartesian_traj_controller/follow_cartesian_trajectory/goal',FollowCartesianTrajectoryActionGoal,queue_size=1000)
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

  def harmonic(self,h,dt,ts):
    t = np.arange(0,ts+dt,dt)
    y = h*(t/ts - 1/(2*pi)*np.sin(2*pi*t/ts))
    return y,t

  def biharmonic(self,h,dt,ts):
    t = np.arange(0,ts+dt,dt)
    y = h/2*((1 - np.cos(pi*t/ts)) - 1/4*(1 - np.cos(2*pi*t/ts)))
    return y,t

  def biharmonic2(self,h,dt,ts):
    t_ = np.arange(0,ts+dt,dt)
    y_ = h/2*((1 - np.cos(pi*t_/ts)) - 1/4*(1 - np.cos(2*pi*t_/ts)))
    t = np.arange(0,2*ts+dt,dt)
    y = np.append(y_,np.flip(y_[:-1]))
    return y,t

  def do_estimate_lift(self):

    tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    pose_ref = self.get_current_pose()
    ref_to_base = tf_buffer.lookup_transform('base',pose_ref.header.frame_id,rospy.Time.now(),timeout=rospy.Duration(1))
    pose = tf2_geometry_msgs.do_transform_pose(pose_ref,ref_to_base)
    pose = cv.pose_to_list(pose.pose)
    pos = pose[0:3]
    quat = pose[3:]
    rot = ts.euler_from_quaternion(quat)
    
    x_pos_arr,t_pos = self.biharmonic2(self.x_p,self.dt,self.t_p/2)
    y_pos_arr = self.harmonic(self.y_p,self.dt,self.t_p)[0]
    z_pos_arr = self.biharmonic2(self.z_p,self.dt,self.t_p/2)[0]
    x_rot_arr,t_rot = self.biharmonic2(self.x_r,self.dt,self.t_r/2)
    y_rot_arr = self.biharmonic2(self.y_r,self.dt,self.t_r/2)[0]
    z_rot_arr = self.biharmonic2(self.z_r,self.dt,self.t_r/2)[0]

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

    self.sw_ser(self.cart_ctrl,self.j_ctrl,1,False,5) 

    sleep(1)

    cart_goal = FollowCartesianTrajectoryActionGoal()
    for i in range(e,len(x_rot)-e):
      cart_pnt = CartesianTrajectoryPoint() 
      cart_pnt.pose = cv.list_to_pose([x_pos_r[i],y_pos_r[i],z_pos_r[i],x_rot[i],y_rot[i],z_rot[i]])
      cart_pnt.time_from_start = rospy.Duration.from_sec(t_rot[i])
      cart_goal.goal.trajectory.points.append(cart_pnt)
    self.traj_pub.publish(cart_goal)

    sleep(2*self.t_r)

    cart_goal = FollowCartesianTrajectoryActionGoal()
    for i in range(e,len(x_pos)-e):
      cart_pnt = CartesianTrajectoryPoint() 
      cart_pnt.pose = cv.list_to_pose([x_pos[i],y_pos[i],z_pos[i],x_rot_p[i],y_rot_p[i],z_rot_p[i]])
      cart_pnt.time_from_start = rospy.Duration.from_sec(t_pos[i])
      cart_goal.goal.trajectory.points.append(cart_pnt)
    self.traj_pub.publish(cart_goal)

    sleep(2*self.t_p)
    
    self.sw_ser(self.j_ctrl,self.cart_ctrl,1,False,5) 

class RobotCommander(ABC):
  """ Inherite of this abstract class provides a common arm and hand movegroup
  commander, as well a rosservice interface to the handover_wrist_node

  Attributes
  ----------
  arm : moveit_commander.MoveGroupCommander
    manipulator planning group commander
  hand : moveit_commander.MoveGroupCommander
    hand planning group commander

  Methods
  -------
  wristCommand(service_name)
    forward a command to the handover_wrist_node (using its ros services)
  """
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()

  arm = ArmCommander()
  # hand = MiaHandMoveitCommander()                                     # hand commander (useful now only for simulated hw)
  hand = MiaHandCommander("mia_hand")                                 # temp hand commander for real hw (since move_group doesn't work properly for mia hand)

  def wristCommand(self,service_name):
    res = rospy.ServiceProxy(service_name,WristCommand)
    return res().success