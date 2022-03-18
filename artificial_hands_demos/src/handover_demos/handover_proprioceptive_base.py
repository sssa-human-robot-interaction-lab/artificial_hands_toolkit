from cmath import pi
import numpy as np
import random
from time import sleep
from threading import Thread
from abc import ABC

import rospy, rosbag, tf
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from artificial_hands_py.robot_commander import *
from artificial_hands_py import get_key, quat_to_list, list_to_quat

from artificial_hands_msgs.srv import *
from artificial_hands_msgs.msg import *

calibration_joints = []
for c in range(0,6):
  calibration_joints.append([2.0,-pi/2,pi/2,-pi,-pi/2,-pi].copy())
# calibration_joints[0][5] -= pi
# calibration_joints[1][5] -= pi/2
# calibration_joints[2][5] += pi/2
# calibration_joints[3][3] -= pi/2
# calibration_joints[4][3] += pi/2
calibration_joints[0][5] -= pi/2
calibration_joints[1][5] += pi/2
calibration_joints[2][5] += pi
calibration_joints[3][3] -= pi/2
calibration_joints[4][3] += pi/2

max_disp_accel = 0.5
max_disp = 0.05
p_goal_time = pow(4*9.9*abs(max_disp/max_disp_accel),0.5)
p_goal = Point()
p_goal.x = max_disp
p_goal.y = max_disp
p_goal.z = max_disp

max_rot_accel = 2
max_rot = 0.2
r_goal_time = pow(4*9.9*abs(max_rot/max_rot_accel),0.5)
r_goal = Point()
r_goal.x = max_rot
r_goal.y = max_rot
r_goal.z = max_rot

joint_ini = np.array([114.20,-92.06,97.77,-140.54,-90.05,-201.39])
joint_ini = joint_ini/180*pi

joint_home = np.array([pi/2,-pi/2,pi/2,-pi,-pi/2,pi])

def generate_markers(ref_pose : Pose, num : int) -> MarkerArray:
  
  markers = MarkerArray()
  for c in range(0,num):
    marker = Marker()
    marker.header.frame_id = 'handover_start_frame'
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.01
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    marker.color.g = 1
    marker.color.a = 1
    marker.id = c

    # q = ts.quaternion_multiply(ts.quaternion_about_axis(-pi/2,[0,0,1]),quat_to_list(ref_pose.orientation))
    # q_norm = [float(i)/max(q) for i in q]

    # marker.pose.orientation = list_to_quat(q_norm)
    marker.pose.orientation = ref_pose.orientation

    marker.pose.position.x = 0
    marker.pose.position.y = -0.5
    marker.pose.position.z = 0.1
    # marker.pose.position.z = ref_pose.position.z

    d = 0.2*random.uniform(0,1)
    a = pi/4*random.uniform(-1,0)

    marker.pose.position.y -= d*np.cos(a)
    marker.pose.position.z += d*np.sin(a)

    markers.markers.append(marker)

  return markers

def send_frame(trans : list, quat : Quaternion, child : str, parent : str):
  br = tf.TransformBroadcaster()
  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    br.sendTransform(trans,quat,rospy.Time.now(),child,parent)
    rate.sleep()

class WristInterface:

  def __init__(self) -> None:
    self.calib = False
    self.detection = Detection()
    sub = rospy.Subscriber("/wrist_dynamics_data",WristDynamicsStamped,self.wrist_data_callback)
    det_sub = rospy.Subscriber("/wrist_contact_detection",DetectionStamped,self.wrist_detection_callback)
    self.bag_record = False
    self.recog_preempted = False
    self.near_to_end_pose = Bool()

  def open_bag(self,bag_name : str):
    self.bag = rosbag.Bag(bag_name,'w')

  def close_bag(self):
    self.bag_record = False
    sleep(.1)
    self.bag.close()
    
  def wrist_command(self,service_name) -> bool:
    res = rospy.ServiceProxy(service_name,WristDynamicsCommand)
    return res().success

  def wrist_detection_callback(self,msg : DetectionStamped):
    self.detection = msg.detection
  
  def wrist_data_callback(self,msg : WristDynamicsStamped):
    if self.bag_record:
      self.bag.write('wrist_dynamics_data',msg)
      self.bag.write("wrist_dynamics_detection",self.detection)
      self.bag.write("status",self.near_to_end_pose)

class RobotCommander(ABC):

  twist_servo = TwistServoCommander(ns='',ref='base',eef='ft_sensor_frame')
  arm = CartesianServoCommander(ns='',ref='base',eef='ft_sensor_frame')
  hand = MiaHandCommander('mia_hand')
  wrist = WristInterface()                                                 
 
  def wait_for_enter(self):
    rospy.loginfo("PRESS ENTER TO CONTINUE ('e' TO EXIT) > ")
    c = input()
    if c == "e":
      return False
    return True
  
  def wait_for_touch_enter(self):
    self.wrist.wrist_command("wrist_dynamics_command/subscribe")    
    self.wrist.wrist_command("wrist_dynamics_command/start_loop") 
    sleep(1)
    self.wrist.wrist_command("wrist_dynamics_command/set_zero")                                          
    self.wrist.wrist_command("wrist_dynamics_mode/trigger_static")  
    self.wrist.detection.backtrig = False
    start_time =  rospy.Time.now()             
    while self.wrist.detection.backtrig == False:  
      if (rospy.Time.now() - start_time).to_sec() > 180:             
        rospy.loginfo("Program paused -> PRESS ENTER TO RESUME ('e' TO EXIT)")
        c = input()
        if c == "e":
          return False
        else:
          start_time =  rospy.Time.now()
      sleep(0.05)
    self.wrist.wrist_command("wrist_dynamics_command/stop_loop")  
    return True
  
  def wait_for_release_trigger(self,timeout : float = 1000.0):                                                   
    start_time =  rospy.Time.now()
    self.wrist.detection.trigger = False
    while not self.wrist.detection.trigger and (rospy.Time.now() - start_time).to_sec() < timeout:                                                         
      self.arm.rate.sleep()

  def check_calib(self) -> bool:
    self.wrist.wrist_command("wrist_dynamics_command/subscribe")    
    self.wrist.wrist_command("wrist_dynamics_command/start_loop") 
    self.wrist.wrist_command("wrist_dynamics_command/set_calibration") 
    sleep(2)
    return self.wrist.wrist_command("wrist_dynamics_command/check_calibration") 

  def do_calib(self):
    self.wrist.wrist_command("wrist_dynamics_command/subscribe")    
    self.wrist.wrist_command("wrist_dynamics_command/start_loop")     
    self.arm.set_max_velocity_scaling_factor(.25)                                              
    for joint_target in calibration_joints:
      """ Move arm in joint configuration and save mesurments for further calibration """
      self.arm.go(joint_target, wait=True)                               
      self.arm.stop()                                                    
      sleep(.3)                                                          
      self.wrist.wrist_command("wrist_dynamics_mode/save_calibration")                   
      sleep(.7)                                                          
      self.wrist.wrist_command("wrist_dynamics_mode/publish")                                                         
    self.arm.set_max_velocity_scaling_factor(.1)
    self.wrist.wrist_command("wrist_dynamics_command/estimate_calibration")   
    self.wrist.wrist_command("wrist_dynamics_command/set_calibration")   
    self.wrist.wrist_command("wrist_dynamics_command/stop_loop")   

  def do_object_recognition(self):
    self.wrist.wrist_command("wrist_dynamics_command/subscribe")    
    self.wrist.wrist_command("wrist_dynamics_command/start_loop") 
    self.wrist.wrist_command("wrist_dynamics_command/set_calibration")  
    sleep(.5)   
    self.wrist.wrist_command("wrist_dynamics_mode/save_dynamics")         
    self.twist_servo.servo_delta(r_goal_time,'biharmonic2',delta_rot=r_goal)
    joint_home[5] += pi/2
    self.arm.go(joint_home)
    joint_home[5] -= pi/2
    self.twist_servo.servo_delta(r_goal_time,'biharmonic2',delta_rot=r_goal)
    self.wrist.wrist_command("wrist_dynamics_command/stop_loop")          
    self.wrist.wrist_command("wrist_dynamics_command/build_model")  

  def start_object_recognition(self):
    self.wrist.wrist_command("wrist_dynamics_command/subscribe")    
    self.wrist.wrist_command("wrist_dynamics_command/start_loop") 
    self.wrist.wrist_command("wrist_dynamics_command/set_calibration")  
    sleep(.5)   
    self.wrist.wrist_command("wrist_dynamics_mode/save_dynamics")         
    self.wrist.recog_preempted = True          

  def build_object_model(self):
    self.wrist.wrist_command("wrist_dynamics_command/build_model")
  
  def estimate_wrench(self):
    self.wrist.wrist_command("wrist_dynamics_command/subscribe")    
    self.wrist.wrist_command("wrist_dynamics_command/start_loop") 
    self.wrist.wrist_command("wrist_dynamics_command/set_calibration")  
    sleep(.5)  
    self.wrist.wrist_command("wrist_dynamics_mode/estimate_wrench")

  def eef_distance(self, point : Point):
    eef_point = self.arm.get_eef_frame().pose.position
    return pow(pow(eef_point.x-point.x,2)+pow(eef_point.y-point.y,2)+pow(eef_point.y-point.y,2),0.5)