#! /usr/bin/env python3

from cmath import pi
from time import sleep
from threading import Thread

import rospy
import smach
import moveit_commander.conversions as cv

import artificial_hands_py as pyatk
from artificial_hands_msgs.msg import *

joint_start = [pi/2,-1.28,2.09,-0.8,pi/2,.0]
""" Hardcoded arm joint angles for start position"""

joint_grasp = joint_start
""" Hardcoded arm joint angles for home position"""

joint_reach = [.0,-pi/2,pi/2,.0,pi/2,.0]
""" Hardcoded arm joint angles for reach position"""

joint_home = joint_reach
""" Hardcoded arm joint angles for home position"""

b = joint_home[0]
s = joint_home[1]
e = joint_home[2]
w1 = joint_home[3]
w2 = joint_home[4]
w3 = joint_home[5]

calibration_joints = [[b,s,e,w1,w2,w3-pi],
                      [b,s,e,w1,w2,w3-pi/2],
                      [b,s,e,w1,w2,w3+pi/2],
                      [b,s,e,w1-pi/2,w2,w3+pi/2],
                      [b,s,e,w1+pi/2,w2,w3+pi/2],
                      [b,s,e,w1,w2,w3]]
""" Hardcoded arm joint angles for fast calibration of force/torque """


class GoToHome(smach.State,pyatk.RobotCommander):
  """ In this state the robot moves to home position """
  def __init__(self):
    super().__init__(outcomes=['at_home','end'])
    self.arm.set_max_velocity_scaling_factor(.1)                                              # set maximum speed
    self.sub = rospy.Subscriber("wrist_detection",DetectionStamped,self.detectionCallback)    # make use of rostopic to check the backup trigger as start input
    self.trigger = False
  
  def waitForEnter(self):
    rospy.loginfo("Executing state GO_TO_HOME -> PRESS ENTER TO CONTINUE ('e' TO EXIT)")
    c = input()
    if c == "e":
      return 'end'

  def waitForDetection(self):
    sleep(1)
    self.wristCommand("wrist_command/set_zero")                       # set zero of torque/sensor for static trigger                      
    self.wristCommand("wrist_mode/trigger_static")                    # make use of static trigger as go command    
    self.trigger = False
    start_time =  rospy.Time.now()             
    while self.trigger == False:   
      if (rospy.Time.now() - start_time).to_sec() > 200:              # add a timeout (pause smach)
        rospy.loginfo("State GO_TO_HOME paused -> PRESS ENTER TO RESUME ('e' TO EXIT)")
        c = input()
        if c == "e":
          return 'end'
        else:
          start_time =  rospy.Time.now()
      pass
    self.hand.close_cyl()
    self.wristCommand("wrist_mode/publish")

  def detectionCallback(self,msg):
    self.trigger = msg.detection.backtrig

  def execute(self, userdata):
    self.wristCommand("wrist_command/subscribe")                       # subscribe the handover_wrist_node
    self.wristCommand("wrist_command/start_loop")                      # start loop
    # self.waitForEnter()
    self.waitForDetection()                                            # use this function (comment waitForEnter) to start loop by touching the robotic hand
    self.arm.go(joint_home,wait=True)                                  # ensure robot arm and hand strating at home position
    self.hand.open_cyl()                                               # get mia hand in home position
    self.arm.stop()
    rospy.loginfo('Executing state GO_TO_HOME')
    return 'at_home'

class CheckCalib(smach.State,pyatk.RobotCommander):
  """ From the home position, a check on calibration status is required """
  def __init__(self):
    super().__init__(outcomes=['calib','uncalib'])
    self.calib = False                                                  # set to true to ensure calibration at startup

  def execute(self, userdata):
    rospy.loginfo('Executing state CHECK_CALIB')
    # if self.wristCommand("wrist_command/check_calibration"):          # check calibration status (i.e. return 0 if calibration needed, 1 otherwise)
    if not self.calib:
      self.calib = True                                                # do calibration only once
      return 'calib'
    else:
      return 'uncalib'

class DoCalib(smach.State,pyatk.RobotCommander):
  """ Do calibration as needed """
  def __init__(self):
    super().__init__(outcomes=['calib_done'])

  def addCalibrationPoint(self,joint_target):
    self.arm.go(joint_target, wait=True)                               # move the arm
    self.arm.stop()                                                    # prevent residual motion
    sleep(.3)                                                          # wait a bit before starting to record
    self.wristCommand("wrist_mode/save_calibration")                   # set mode to record for force/torque sensor calibration
    sleep(.7)                                                          # average calibration points over .8 seconds
    self.wristCommand("wrist_mode/publish")                            # end of calibration point (stop record)   

  def execute(self, userdata):
    rospy.loginfo('Executing state DO_CALIB')                                          
    for joint_target in calibration_joints:
      self.addCalibrationPoint(joint_target)
    self.wristCommand("wrist_command/set_calibration")                 # estimate offset and calibrate the force/torque sensor
    return 'calib_done'

class GoToGraspPos(smach.State,pyatk.RobotCommander):
  """ Move to grasp position """
  def __init__(self):
    super().__init__(outcomes=['ready_to_grasp'])

  def execute(self, userdata):
    rospy.loginfo('Executing state GO_TO_GRASP_POS')
    self.arm.go(joint_grasp, wait=True)                          # go to the grasp position
    self.arm.stop()
    return 'ready_to_grasp'

class WaitForGrasp(smach.State,pyatk.RobotCommander):
  """ Wait user input to close hand and grasp """
  def __init__(self):
    super().__init__(outcomes=['grasp','home'])

  def execute(self, userdata):
    sleep(3)                                                    # wait a bit before trying to grasp the object
    return 'grasp'

class Grasp(smach.State,pyatk.RobotCommander):
  """ Do grasp action """
  def __init__(self):
    super().__init__(outcomes=['grasped'])

  def execute(self, userdata):
    rospy.loginfo('Executing state GRASP')
    self.hand.close_cyl()                                       # close mia hand with a cylindrical grasp
    return 'grasped'

class DoObjectRecognition(smach.State,pyatk.RobotCommander):
  """ Go to start position for lifting and recognizing the object """
  def __init__(self):
    super().__init__(outcomes=['recog_done','home'])
    self.arm.set_estimate_lift(1.4,1.9,.05,.2,.05,.1,.1,.1)     # ATTENTION displacement (in metres) are expressed in the target frame

  def execute(self, userdata):
    self.arm.go(joint_start ,wait=True)                         # move arm to a start configuration
    self.wristCommand("wrist_mode/save_dynamics")               # start to record dynamics
    self.arm.do_estimate_lift()                                 # lift the object with predefined motion
    self.wristCommand("wrist_command/stop_loop")                # stop node loop
    self.wristCommand("wrist_command/build_model")              # estimate inertial model
    return 'recog_done'

class PrepareToReach(smach.State,pyatk.RobotCommander):
  def __init__(self):
    super().__init__(outcomes=['ready_to_reach'])

  def execute(self, userdata):
    rospy.loginfo('Executing state PREPARE_TO_REACH')
    self.wristCommand("wrist_command/subscribe")                # subscribe to js and ft topics
    self.wristCommand("wrist_command/start_loop")               # start node loop
    self.wristCommand("wrist_mode/save_interaction")            # save estimate error on interaction forces
    return 'ready_to_reach'

class StartToReach(smach.State,pyatk.RobotCommander):
  def __init__(self):
    super().__init__(outcomes=['ready_to_handover','end'])

  def execute(self, userdata):
    rospy.loginfo('Executing state START_TO_REACH')
    self.arm.go(joint_reach,wait=False)                          # start reaching 
    sleep(1)                                                     # wait a bit to be confident on estimate error
    self.wristCommand("wrist_mode/trigger_dynamics")             # enable trigger for dynamic handover
    return 'ready_to_handover'

class ReachToHandover(smach.State,pyatk.RobotCommander):
  def __init__(self):
    super().__init__(outcomes=['released'])
    self.sub = rospy.Subscriber("wrist_detection",DetectionStamped,self.detectionCallback) # make use of a ros subscriber to check trigger
    self.trigger = False 

  def detectionCallback(self,msg):
    self.trigger = msg.detection.trigger

  def execute(self, userdata):
    self.trigger = False                                                                   # reset the trigger
    rospy.loginfo('Executing state REACH_TO_HANDOVER') 
    t = Thread(target = self.hand.open_cyl)                                                # instantiate a separate thread for mia hand open command
    start_time =  rospy.Time.now()
    while not self.trigger and (rospy.Time.now() - start_time).to_sec() < 10:              # set also a timeout                                              
      pass                                                                                 # wait for trigger to occur
    if self.trigger:
      t.start()                                                                            # start the thread 
    self.arm.stop()                                                                        # block arm motion
    self.wristCommand("wrist_command/stop_loop")                                           # stop node loop
    sleep(2)
    return 'released'
     
def main():

  rospy.init_node("handover_release_smach",)

  sm = smach.StateMachine(outcomes=['end'])

  with sm:
    smach.StateMachine.add('GO_TO_HOME', GoToHome(), transitions={'at_home':'CHECK_CALIB'})
    smach.StateMachine.add('CHECK_CALIB', CheckCalib(), transitions={'uncalib':'DO_CALIB','calib':'GO_TO_GRASP_POS'})
    smach.StateMachine.add('DO_CALIB', DoCalib(), transitions={'calib_done':'GO_TO_GRASP_POS'})
    smach.StateMachine.add('GO_TO_GRASP_POS', GoToGraspPos(), transitions={'ready_to_grasp':'WAIT_FOR_GRASP'})
    smach.StateMachine.add('WAIT_FOR_GRASP', WaitForGrasp(), transitions={'grasp':'GRASP','home':'GO_TO_HOME'})
    smach.StateMachine.add('GRASP', Grasp(), transitions={'grasped':'DO_OBJECT_RECOGNITION'})
    smach.StateMachine.add('DO_OBJECT_RECOGNITION', DoObjectRecognition(), transitions={'recog_done':'PREPARE_TO_REACH','home':'GO_TO_HOME'})
    smach.StateMachine.add('PREPARE_TO_REACH', PrepareToReach(), transitions={'ready_to_reach':'START_TO_REACH'})
    smach.StateMachine.add('START_TO_REACH', StartToReach(), transitions={'ready_to_handover':'REACH_TO_HANDOVER'})
    smach.StateMachine.add('REACH_TO_HANDOVER', ReachToHandover(), transitions={'released':'GO_TO_HOME'})

  outcome = sm.execute()
  
if __name__ == '__main__':
  main()