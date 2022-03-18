from time import sleep

import rospy, rosbag, tf
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from artificial_hands_py.robot_commander import *
from artificial_hands_py import get_key, quat_to_list, list_to_quat

from artificial_hands_msgs.srv import *
from artificial_hands_msgs.msg import *

class WristDynamicsInterface:

  def __init__(self) -> None:
    self.calib = False
    self.detection = Detection()
    sub = rospy.Subscriber("/wrist_dynamics_data",WristDynamicsStamped,self.wrist_data_callback)
    det_sub = rospy.Subscriber("/wrist_contact_detection",DetectionStamped,self.wrist_detection_callback)
    self.bag_record = False
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