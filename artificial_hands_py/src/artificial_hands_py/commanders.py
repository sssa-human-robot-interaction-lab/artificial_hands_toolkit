#! /usr/bin/env python3

from cmath import pi
from time import sleep
from abc import ABC
import numpy as np
from threading import Thread

import rospy
import moveit_commander
import moveit_commander.conversions as cv
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from rqt_controller_manager.controller_manager import *
from std_srvs.srv import Empty

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
    self.sw_ser = rospy.ServiceProxy('/controller_manager/switch_controller',SwitchController) 
    self.pub = rospy.Publisher("/cartesian_motion_controller/command",PoseStamped,queue_size=1000)
    self.dt = 0.01
    self.x_est = 0.0
    self.y_est = 0.0
    self.z_est = 0.0

  def set_estimate_lift(self,x,y,z):
    self.x_est = x
    self.y_est = y
    self.z_est = z

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

  def do_estimate_lift(self,T):
    x_pos = self.biharmonic2(self.x_est,self.dt,T/2)[0]
    y_pos = self.biharmonic2(self.y_est,self.dt,T/2)[0]
    z_pos,t = self.harmonic(self.z_est,self.dt,T)
    x_rot = np.zeros(len(x_pos))
    y_rot = np.zeros(len(x_pos))
    z_rot = np.zeros(len(x_pos))

    if not (len(x_pos) == len(y_pos) and len(x_pos) == len(z_pos)):
      rospy.logerr("Error in cartesian points: inconsistent lengths.")

    pos = cv.pose_to_list(self.get_current_pose().pose)[0:3]

    x_pos = np.ndarray.tolist(pos[0] + x_pos)
    y_pos = np.ndarray.tolist(pos[1] + y_pos)
    z_pos = np.ndarray.tolist(pos[2] + z_pos)

    rot = self.get_current_rpy()
    x_rot = np.ndarray.tolist(rot[0] + x_rot)
    y_rot = np.ndarray.tolist(rot[1] + y_rot)
    z_rot = np.ndarray.tolist(rot[2] + z_rot)

    cart_poses = []
    for i in range(10,len(x_pos)-10):
      pose = [x_pos[i],y_pos[i],z_pos[i],x_rot[i],y_rot[i],z_rot[i]]
      cart_poses.append(cv.list_to_pose(pose))

    (plan, fraction) = self.compute_cartesian_path(cart_poses,0.01,0.0)

    if fraction != 1:
      rospy.logerr("ABORTED. Failed to compute a valid cartesian path (%.2f %%).",100*fraction)
      return

    self.sw_ser(['cartesian_motion_controller'],['pos_joint_traj_controller'],1,False,5) 

    i = 0
    rate = rospy.Rate(1/self.dt)
    while i < len(x_pos):
      pose = cv.list_to_pose_stamped([x_pos[i],y_pos[i],z_pos[i],x_rot[i],y_rot[i],z_rot[i]],'world')
      self.pub.publish(pose)
      i += 1
      rate.sleep()
      
    sleep(1)

    self.sw_ser(['pos_joint_traj_controller'],['cartesian_motion_controller'],1,False,5) 

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
  hand = MiaHandMoveitCommander()                                     # hand commander (useful now only for simulated hw)
  # hand = MiaHandCommander("mia_hand")                                 # temp hand commander for real hw (since move_group doesn't work properly for mia hand)

  def wristCommand(self,service_name):
    res = rospy.ServiceProxy(service_name,WristCommand)
    return res().success