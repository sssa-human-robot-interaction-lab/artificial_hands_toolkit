from time import sleep
import rospy, actionlib

from geometry_msgs.msg import Pose
from cartesian_control_msgs.msg import CartesianTrajectoryPoint

from artificial_hands_msgs.msg import *
from artificial_hands_py.robot_commander.robot_commander import RobotCommander
from artificial_hands_py.low_level_execution_modules.wrist_dynamics_module import WristDynamicsModule

class RobotHumanHandoverReachingModule(WristDynamicsModule,RobotCommander):
  r2h_handv_feedback = RobotHumanHandoverReachingFeedback()
  r2h_handv_result = RobotHumanHandoverReachingActionResult()

  def __init__(self) -> None:
    super().__init__()
  
    self.r2h_handv_as = actionlib.SimpleActionServer('robot_to_human_handover',RobotHumanHandoverReachingAction,execute_cb=self.r2h_handover_cb,auto_start=False)

    det_sub = rospy.Subscriber('wrist_dynamics_detection',DetectionStamped,callback=self.detection_cb)
    trg_sub = rospy.Subscriber("handover_target_point",CartesianTrajectoryPoint,callback=self.handover_target_cb)

    self.detection = Detection()
    self.target = Pose()

    self.r2h_handv_as.start()

  def handover_target_cb(self, msg : CartesianTrajectoryPoint):
    self.target = msg.pose

  def detection_cb(self, msg : DetectionStamped):
    self.detection = msg.detection   

  def r2h_handover_cb(self, goal : RobotHumanHandoverReachingGoal):
    self.r2h_handv_result.result.success = True
    self.r2h_handv_feedback.percentage = 100

    # initialize wrist_dynamics_module making use of previous calibration (assumed valid)
    self.start_node(calib=True)
    self.set_estimate_wrench()
    
    # move to start position
    self.arm.set_max_accel(goal.max_accel)
    self.arm.set_max_angaccel(goal.max_angaccel)
    self.arm.set_harmonic_traj_generator()
    self.arm.switch_to_cartesian_controller('cartesian_motion_position_controller')
    self.arm.set_pose_target(goal.home)

    # change to dmp
    self.arm.set_dmp_traj_generator()

    # go to target pose
    self.arm.set_pose_target(goal.target)
    sleep(3)

    # update dmp target according to human hand pose
    # rate = rospy.Rate(50)
    # self.detection.trigger = False
    # while not self.detection.trigger:
    #   self.arm.set_pose_target(self.target)
    #   rate.sleep()
      
    # stop arm immediately after trigger
    self.arm.pause_all_controllers()
    
    # get wrist_dynamics_module to idle
    self.stop_loop()

    self.r2h_handv_as.set_succeeded(self.r2h_handv_result)

def main():

  rospy.init_node('robot_human_handover_reaching_module_node')

  r2h_handv_reach_mod = RobotHumanHandoverReachingModule()

  rospy.spin()

if __name__ == '__main__':
  main()