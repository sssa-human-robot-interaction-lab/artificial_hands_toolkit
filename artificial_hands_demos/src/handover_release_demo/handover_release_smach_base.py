from time import sleep
from threading import Thread
from abc import ABC

import rospy

from artificial_hands_msgs.srv import *
from artificial_hands_msgs.msg import *
from handover_release_demo.constants import *
from artificial_hands_py import MiaHandCommander, HarmonicServoCommander

class WristInterface:
  """ Purpose of this class is to provide interface to the wrist_node (send command,
  check status, etc.) """

  def __init__(self) -> None:
    self.calib = False
    self.detection = Detection()
    sub = rospy.Subscriber("/wrist_detection",DetectionStamped,self.wristCallback)

  def wristCommand(self,service_name):
    res = rospy.ServiceProxy(service_name,WristCommand)
    return res().success

  def wristCallback(self,msg : DetectionStamped):
    self.detection = msg.detection

class RobotCommander(ABC):
  """ Purpose of this abstract class is to provide a common interface to arm and 
  hand commanders and to wrist_node services and publishers"""

  arm = HarmonicServoCommander()
  hand = MiaHandCommander('mia_hand')
  hand_async_open = Thread(target = hand.open)
  wrist = WristInterface()

  def goHome(self):
    self.arm.go(joint_home,wait=True)
    self.hand.open()                                                   
    self.arm.stop() 
    sleep(1)                                                   

  def waitForTouchEnter(self,state):
    """ Just wait for user to touch the hand"""
    sleep(1)
    self.wrist.wristCommand("wrist_command/set_zero")                                          
    self.wrist.wristCommand("wrist_mode/trigger_static")  
    self.wrist.detection.backtrig = False
    start_time =  rospy.Time.now()             
    while self.wrist.detection.backtrig == False:  
      if (rospy.Time.now() - start_time).to_sec() > 180:             
        rospy.loginfo("State " + state + " paused -> PRESS ENTER TO RESUME ('e' TO EXIT)")
        c = input()
        if c == "e":
          return False
        else:
          start_time =  rospy.Time.now()
      sleep(1/1000)
    self.hand.close_cyl()
    self.wrist.wristCommand("wrist_mode/publish")
    return True
 
  def waitForEnter(self,state):
    """ Just wait for user to press enter"""
    rospy.loginfo("Executing state " + state + " -> PRESS ENTER TO CONTINUE ('e' TO EXIT)")
    c = input()
    if c == "e":
      return False
    return True
  
  def waitForReleaseTrigger(self,timeout : float = 1000.0):
    """ Just wait until user grasps the object to handover or timeout (in sec) occurs """                                                      
    start_time =  rospy.Time.now()
    while not self.wrist.detection.trigger and (rospy.Time.now() - start_time).to_sec() < timeout:                                                         
      pass  

  def longCalib(self):
    for joint_target in calibration_joints:
      """ Move arm in joint configuration and save mesurments for further calibration """
      self.arm.go(joint_target, wait=True)                               
      self.arm.stop()                                                    
      sleep(.3)                                                          
      self.wrist.wristCommand("wrist_mode/save_calibration")                   
      sleep(.7)                                                          
      self.wrist.wristCommand("wrist_mode/publish") 

  def fastCalib(self):
    self.arm.go(joint_calib,wait=True)
    self.wrist.wristCommand("wrist_mode/save_calibration")  
    goal_accel = -0.5
    goal_time = 1 
    goal = Pose()
    goal_2 = Pose()
    goal_2.position.x = goal_accel*pow(goal_time/2,2)/9.9
    goal_2.position.y = goal_2.position.x
    goal_2.position.z = goal_2.position.x
    self.arm.servo_delta(goal_time, delta_pose = goal, delta_pose_2 = goal_2) 
    sleep(.1)                                                                                                        
  