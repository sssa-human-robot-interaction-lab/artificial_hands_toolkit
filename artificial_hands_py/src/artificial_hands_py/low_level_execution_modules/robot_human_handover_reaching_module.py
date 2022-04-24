from time import sleep
import rospy, actionlib

from geometry_msgs.msg import Pose
from cartesian_control_msgs.msg import CartesianTrajectoryPoint

from artificial_hands_msgs.msg import *
from artificial_hands_py.robot_commander.robot_commander import RobotCommander
from artificial_hands_py.low_level_execution_modules.wrist_dynamics_module import WristDynamicsModule

class RobotHumanHandoverReachingModule(WristDynamicsModule,RobotCommander):
  sleep_dur = rospy.Duration(0.5)
  r2h_handv_feedback = RobotHumanHandoverReachingFeedback()
  r2h_handv_result = RobotHumanHandoverReachingResult()

  def __init__(self) -> None:
    super().__init__()
  
    self.r2h_handv_as = actionlib.SimpleActionServer('/robot_to_human_handover_reaching',RobotHumanHandoverReachingAction,execute_cb=self.r2h_handover_cb,auto_start=False)

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
    self.r2h_handv_result.success = True
    self.r2h_handv_feedback.percentage = 100

    # initialize wrist_dynamics_module making use of previous calibration (assumed valid)
    # self.start_node(calib=True)

    # start to store proprioceptive information to further thresholding, but first wait a bit to get filter adapt to calib offset
    rospy.sleep(self.sleep_dur)
    # self.set_save_interaction()
    
    # move to start position
    self.arm.set_stop_time(goal.stop_time)
    self.arm.set_stop_factor(goal.stop_factor)
    self.arm.set_max_accel(goal.max_accel)
    self.arm.set_max_angaccel(goal.max_angaccel)
    self.arm.set_harmonic_traj_generator()
    self.arm.switch_to_cartesian_controller('cartesian_motion_position_controller')
    self.arm.set_pose_target(goal.home)

    # change to dmp
    # self.arm.set_dmp_traj_generator()

    # go to target pose and monitor execution
    self.arm.set_pose_target(goal.target,False)
    self.arm.update_trajectory_monitor()

    # wait at least for half of the nominal trajectory, then start to trigger
    rate = rospy.Rate(20)
    while self.arm.percentage < 50:
      rate.sleep()  
    # self.set_trigger_dynamics()

    # update dmp target according to human hand pose, break on detection
    while True:
      if self.arm.percentage > 70:
        break
      # self.arm.set_pose_target(self.target)
      # self.arm.update_trajectory_monitor()
      rate.sleep()
      
    # stop arm immediately at the end of the loop
    self.arm.stop()

    # wait a bit before set wrist_dynamics_module to idle
    rospy.sleep(rospy.Duration(goal.sleep))
    # self.stop_loop()

    # wait a bit and retire to back position
    # rospy.sleep(rospy.Duration(goal.sleep))
    # self.arm.set_pose_target(goal.back)

    # stop controllers
    self.arm.pause_all_controllers()
    self.r2h_handv_as.set_succeeded(self.r2h_handv_result)

def main():

  rospy.init_node('robot_human_handover_reaching_module_node')

  r2h_handv_reach_mod = RobotHumanHandoverReachingModule()

  rospy.loginfo('R2H handover reaching module ready!')

  rospy.spin()

if __name__ == '__main__':
  main()