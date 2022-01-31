from cmath import pi
from time import sleep
from abc import ABC, abstractmethod
from threading import Thread

from rospkg import RosPack

import rospy
import smach
import moveit_commander
import moveit_commander.conversions as cv
from std_msgs.msg import Float64
from rqt_controller_manager.controller_manager import *

from artificial_hands_msgs.msg import *
from artificial_hands_msgs.srv import WristCommand

joint_home = [.0,-pi/2,pi/2,.0,pi/2,.0]
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

    self.j = Float64()

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

class atkRobot(ABC):
  """ Inherite of this abstract class provides a common arm and hand movegroup
  commander, as well as to the handover_wrist_node

  Attributes
  ----------
  arm : moveit_commander.MoveGroupCommander
    manipulator planning group commander
  hand : moveit_commander.MoveGroupCommander
    hand planning group commander
  trigger : bool
    boolean representing release trigger from handover_wrist_node

  Methods
  -------
  goHome()
    move arm and hand to home positions
  wristCommand(service_name)
    forward a command to the handover_wrist_node (using its ros services)
  """
  arm = moveit_commander.MoveGroupCommander("manipulator")            # arm commander
  hand = MiaHandCommander("mia_hand")                                 # temp hand commander (since move_group doesn't work properly for mia hand)

  def goHome(self):
    self.arm.go(joint_home, wait=True)                                # get arm in home position
    self.hand.switch_to_open()                                        # prepare mia hand to open grasp
    self.hand.open_cyl()                                              # get mia hand in home position
    self.arm.stop()                                                   # calling stop() ensures that there is no residual movement

  def wristCommand(self,service_name):
    res = rospy.ServiceProxy(service_name,WristCommand)
    return res().code

class GoToHome(smach.State,atkRobot):
  """ In this state the robot moves to home position """
  def __init__(self):
    super().__init__(outcomes=['at_home','end'])
    self.arm.set_max_velocity_scaling_factor(.2)                      # set maximum speed

  def execute(self, userdata):
    rospy.loginfo("Executing state GO_TO_HOME -> PRESS ENTER TO START ('e' TO EXIT)")
    c = input()
    if c == "e":
      return 'end'
    else:
      self.goHome()                                                     # ensure robot arm and hand strating at home position
      self.wristCommand("wrist_command/subscribe")                      # subscribe the handover_wrist_node
      self.wristCommand("wrist_command/start_loop")                     # start loop
      rospy.loginfo('Executing state GO_TO_HOME')
      return 'at_home'

class CheckCalib(smach.State,atkRobot):
  """ From the home position, a check on calibration status is required """
  def __init__(self):
    super().__init__(outcomes=['calib','uncalib'])

  def execute(self, userdata):
    rospy.loginfo('Executing state CHECK_CALIB')
    if self.wristCommand("wrist_command/check_calibration"):          # check calibration status (i.e. return 1 if eef mass estimate has changed, 0 otherwise)
      return 'uncalib'
    else:
      return 'calib'

class DoCalib(smach.State,atkRobot):
  """ Do calibration as needed """
  def __init__(self):
    super().__init__(outcomes=['calib_done'])

  def execute(self, userdata):
    rospy.loginfo('Executing state DO_CALIB')                                          
    for joint_target in calibration_joints:
      self.addCalibrationPoint(joint_target)
    self.wristCommand("wrist_command/set_calibration")                 # estimate offset and calibrate the force/torque sensor
    return 'calib_done'

  def addCalibrationPoint(self,joint_target):
    self.arm.go(joint_target, wait=True)                               # move the arm
    self.arm.stop()                                                    # prevent residual motion
    sleep(.3)                                                          # wait a bit before starting to record
    self.wristCommand("wrist_mode/save_calibration")                   # set mode to record for force/torque sensor calibration
    sleep(.7)                                                          # average calibration points over .8 seconds
    self.wristCommand("wrist_mode/publish")                            # end of calibration point (stop record)   

class GoToGraspPos(smach.State,atkRobot):
  """ Move to grasp position """
  def __init__(self):
    super().__init__(outcomes=['ready_to_grasp'])

  def execute(self, userdata):
    rospy.loginfo('Executing state GO_TO_GRASP_POS')
    self.arm.go([1.57,-1.28,2.09,-0.8,1.57,0.0], wait=True)            # hardcoded grasp position
    self.arm.stop() 
    return 'ready_to_grasp'

class WaitForGrasp(smach.State,atkRobot):
  """ Wait user input to close hand and grasp """
  def __init__(self):
    super().__init__(outcomes=['grasp','home'])

  def execute(self, userdata):
    rospy.loginfo("Executing state WAIT_FOR_GRASP -> PRESS ENTER TO CONTINUE ('h' TO RETURN TO HOME POSITION)")
    c = input()
    if c == "h":
      return 'home'
    else:
      return 'grasp'

class Grasp(smach.State,atkRobot):
  """ Do grasp action """
  def __init__(self):
    super().__init__(outcomes=['grasped'])

  def execute(self, userdata):
    rospy.loginfo('Executing state GRASP')
    self.hand.close_cyl()                                                   # close mia hand with a cylindrical grasp
    return 'grasped'

class GoToStart(smach.State,atkRobot):
  def __init__(self):
    super().__init__(outcomes=['at_start','home'])

  def execute(self, userdata):
    rospy.loginfo("Executing state GO_TO_START -> PRESS ENTER TO CONTINUE ('h' TO RETURN TO HOME POSITION)")
    c = input()
    if c == "h":
      return 'home'
    else:
      self.wristCommand("wrist_mode/save_dynamics")                           # set mode to record dynamics    
      lift_poses = []
      pos = cv.pose_to_list(self.arm.get_current_pose().pose)[0:3]            # get current position
      rot = self.arm.get_current_rpy()                                        # get current orientation
      pose = pos + rot                                        
      for k in range(1,10):           
        pose[0] += 0.005                                                      # little displacement along world x
        pose[1] += 0.005                                                      # little displacement along world y
        pose[2] += 0.03                                                       # consistent lifting along world z
        pose[3] += 0.05                                                       # little change in all orientations
        pose[4] += 0.05                                                 
        pose[5] += 0.05                                                 
        lift_poses.append(cv.list_to_pose(pose))
      (plan, fraction) = self.arm.compute_cartesian_path(lift_poses,0.01,0.0) # compute cartesian path
      self.arm.execute(plan, wait=True)                                       # execute
      return 'at_start'

class PrepareToReach(smach.State,atkRobot):
  def __init__(self):
    super().__init__(outcomes=['ready_to_reach'])

  def execute(self, userdata):
    rospy.loginfo('Executing state PREPARE_TO_REACH')
    self.wristCommand("wrist_command/stop_loop")                # stop node loop
    self.wristCommand("wrist_command/build_model")              # estimate inertial model
    self.wristCommand("wrist_mode/estimate_wrench")             # set mode to estimate forces
    self.wristCommand("wrist_command/subscribe")                # subscribe to js and ft topics
    self.wristCommand("wrist_command/start_loop")               # start node loop
    self.wristCommand("wrist_mode/save_interaction")            # save estimate error
    return 'ready_to_reach'

class StartToReach(smach.State,atkRobot):
  def __init__(self):
    super().__init__(outcomes=['ready_to_handover','end'])

  def execute(self, userdata):
    rospy.loginfo('Executing state START_TO_REACH')
    self.arm.go(joint_home,wait=False)                           # start reaching
    sleep(1)                                                     # wait a bit to be confident on estimate error
    self.wristCommand("wrist_mode/trigger_dynamics")             # enable trigger for dynamic handover
    return 'ready_to_handover'

class ReachToHandover(smach.State,atkRobot):
  def __init__(self):
    super().__init__(outcomes=['released'])
    self.sub = rospy.Subscriber("wrist_detection",DetectionStamped,self.detectionCallback)  # using a ros subscriber to check trigger
    self.trigger = False 

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
    return 'released'
  
  def detectionCallback(self,msg):
    self.trigger = msg.trigger
        
def main():

  rospy.init_node("handover_release_smach",)

  sm = smach.StateMachine(outcomes=['end'])

  with sm:
    smach.StateMachine.add('GO_TO_HOME', GoToHome(), transitions={'at_home':'CHECK_CALIB'})
    smach.StateMachine.add('CHECK_CALIB', CheckCalib(), transitions={'uncalib':'DO_CALIB','calib':'GO_TO_GRASP_POS'})
    smach.StateMachine.add('DO_CALIB', DoCalib(), transitions={'calib_done':'GO_TO_GRASP_POS'})
    smach.StateMachine.add('GO_TO_GRASP_POS', GoToGraspPos(), transitions={'ready_to_grasp':'WAIT_FOR_GRASP'})
    smach.StateMachine.add('WAIT_FOR_GRASP', WaitForGrasp(), transitions={'grasp':'GRASP','home':'GO_TO_HOME'})
    smach.StateMachine.add('GRASP', Grasp(), transitions={'grasped':'GO_TO_START'})
    smach.StateMachine.add('GO_TO_START', GoToStart(), transitions={'at_start':'PREPARE_TO_REACH','home':'GO_TO_HOME'})
    smach.StateMachine.add('PREPARE_TO_REACH', PrepareToReach(), transitions={'ready_to_reach':'START_TO_REACH'})
    smach.StateMachine.add('START_TO_REACH', StartToReach(), transitions={'ready_to_handover':'REACH_TO_HANDOVER'})
    smach.StateMachine.add('REACH_TO_HANDOVER', ReachToHandover(), transitions={'released':'GO_TO_HOME'})

  outcome = sm.execute()
  
if __name__ == '__main__':
  main()