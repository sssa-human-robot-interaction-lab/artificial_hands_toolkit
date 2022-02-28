#! /usr/bin/env python3

import rospy,smach
from handover_release_demo.handover_release_smach_base import *

class GoToHome(smach.State,RobotCommander):
  """ In this state the robot moves to home position """
  def __init__(self):
    super().__init__(outcomes=['at_home','end'])
    self.arm.set_max_velocity_scaling_factor(.1)                  # set maximum speed (be careful exceeding .2)

  def execute(self, userdata):
    rospy.loginfo('Executing state GO_TO_HOME')
    self.wrist.wristCommand("wrist_command/subscribe")            # start subscribers
    self.wrist.wristCommand("wrist_command/start_loop")           # start loop
    # if self.waitForDetection('GO_TO_HOME'):                       # use this function (comment waitForEnter) to start loop by touching the robotic hand
    if self.waitForEnter('GO_TO_HOME'):
      self.goHome()
      return 'at_home'
    return 'end'

class CheckCalib(smach.State,RobotCommander):
  """ From the home position, a check on calibration status is required """
  def __init__(self):
    super().__init__(outcomes=['calib','uncalib'])

  def execute(self, userdata):
    rospy.loginfo('Executing state CHECK_CALIB')
    self.wrist.calib = self.wrist.wristCommand("wrist_command/check_calibration")
    if self.wrist.calib:
      return 'calib'
    return 'uncalib'

class DoCalib(smach.State,RobotCommander):
  """ Do calibration as needed """
  def __init__(self):
    super().__init__(outcomes=['calib_done','return_to_home'])   

  def execute(self, userdata):
    rospy.loginfo('Executing state DO_CALIB')  
    self.arm.set_max_velocity_scaling_factor(.25)                                              
    self.longCalib()                                              # execute fast calibration procedure
    self.arm.set_max_velocity_scaling_factor(.1)
    self.wrist.wristCommand("wrist_command/estimate_calibration") # solve for calibration offsets
    self.wrist.wristCommand("wrist_command/set_calibration")      # make use of calibration result
    return 'return_to_home'

class GoToGraspPos(smach.State,RobotCommander):
  """ Move to grasp position """
  def __init__(self):
    super().__init__(outcomes=['ready_to_grasp'])

  def execute(self, userdata):
    rospy.loginfo('Executing state GO_TO_GRASP_POS')
    self.arm.go(joint_grasp, wait=True)                          # go to the grasp position
    self.arm.stop()
    return 'ready_to_grasp'

class WaitForGrasp(smach.State,RobotCommander):
  """ Wait user input to close hand and grasp """
  def __init__(self):
    super().__init__(outcomes=['grasp','home'])

  def execute(self, userdata):
    sleep(3)                                                     # wait a bit before trying to grasp the object
    return 'grasp'

class Grasp(smach.State,RobotCommander):
  """ Do grasp action """
  def __init__(self):
    super().__init__(outcomes=['grasped'])

  def execute(self, userdata):
    rospy.loginfo('Executing state GRASP')
    self.hand.close_cyl()                                        # close mia hand with a cylindrical grasp
    return 'grasped'

class DoObjectRecognition(smach.State,RobotCommander):
  """ Go to start position for lifting and recognizing the object """
  def __init__(self):
    super().__init__(outcomes=['recog_done','home'])

  def execute(self, userdata):
    self.arm.go(joint_start ,wait=True)                          # move arm to the start configuration
    self.wrist.wristCommand("wrist_mode/save_dynamics")          # start to record dynamics
    sleep(1)
    # self.arm.servo_delta(goal_time,goal,goal_2)                  # lift the object enforcing linear acceleration
    self.wrist.wristCommand("wrist_command/stop_loop")           # stop node loop
    self.wrist.wristCommand("wrist_command/build_model")         # estimate inertial model
    return 'recog_done'

class PrepareToReach(smach.State,RobotCommander):
  def __init__(self):
    super().__init__(outcomes=['ready_to_reach'])

  def execute(self, userdata):
    rospy.loginfo('Executing state PREPARE_TO_REACH')
    self.wrist.wristCommand("wrist_command/subscribe")           # subscribe to js and ft topics
    self.wrist.wristCommand("wrist_command/start_loop")          # start node loop
    self.wrist.wristCommand("wrist_command/set_calibration")     # make use of previous calibration
    self.wrist.wristCommand("wrist_mode/save_interaction")       # save estimate error on interaction forces
    return 'ready_to_reach'

class StartToReach(smach.State,RobotCommander):
  def __init__(self):
    super().__init__(outcomes=['ready_to_handover'])

  def execute(self, userdata):
    rospy.loginfo('Executing state START_TO_REACH')
    self.arm.go(joint_reach,wait=False)                         # start reaching 
    sleep(1)                                                    # wait a bit to be confident on estimate error
    self.wrist.wristCommand("wrist_mode/trigger_dynamics")      # enable trigger for dynamic handover
    return 'ready_to_handover'

class ReachToHandover(smach.State,RobotCommander):
  def __init__(self):
    super().__init__(outcomes=['return_to_home'])

  def execute(self, userdata):                                                                   
    rospy.loginfo('Executing state REACH_TO_HANDOVER')                                                     
    self.waitForReleaseTrigger(timeout = 180)                   # wait for trigger or until timeout occurs                                                          
    if self.wrist.detection.trigger:                            # check trigger
      self.hand_async_open.start()                              # eventually open hand in separate thread
    self.arm.stop()                                             # then stop the arm motion                                                                                                           
    sleep(2)                                                    # wait a bit before restart
    return 'return_to_home'
     
def main():

  rospy.init_node("handover_release_smach",)

  sm = smach.StateMachine(outcomes=['end'])

  with sm:
    smach.StateMachine.add('GO_TO_HOME', GoToHome(), transitions={'at_home':'CHECK_CALIB'})
    smach.StateMachine.add('CHECK_CALIB', CheckCalib(), transitions={'uncalib':'DO_CALIB','calib':'GO_TO_GRASP_POS'})
    smach.StateMachine.add('DO_CALIB', DoCalib(), transitions={'calib_done':'GO_TO_GRASP_POS','return_to_home':'GO_TO_HOME'})
    smach.StateMachine.add('GO_TO_GRASP_POS', GoToGraspPos(), transitions={'ready_to_grasp':'WAIT_FOR_GRASP'})
    smach.StateMachine.add('WAIT_FOR_GRASP', WaitForGrasp(), transitions={'grasp':'GRASP','home':'GO_TO_HOME'})
    smach.StateMachine.add('GRASP', Grasp(), transitions={'grasped':'DO_OBJECT_RECOGNITION'})
    smach.StateMachine.add('DO_OBJECT_RECOGNITION', DoObjectRecognition(), transitions={'recog_done':'PREPARE_TO_REACH','home':'GO_TO_HOME'})
    smach.StateMachine.add('PREPARE_TO_REACH', PrepareToReach(), transitions={'ready_to_reach':'START_TO_REACH'})
    smach.StateMachine.add('START_TO_REACH', StartToReach(), transitions={'ready_to_handover':'REACH_TO_HANDOVER'})
    smach.StateMachine.add('REACH_TO_HANDOVER', ReachToHandover(), transitions={'return_to_home':'GO_TO_HOME'})

  outcome = sm.execute()
  
if __name__ == '__main__':
  main()