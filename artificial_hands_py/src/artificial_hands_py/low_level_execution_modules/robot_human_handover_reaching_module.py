from threading import Thread

import rospy, actionlib

from geometry_msgs.msg import Pose
from cartesian_control_msgs.msg import CartesianTrajectoryPoint

from artificial_hands_msgs.msg import *
from artificial_hands_py.robot_commander.robot_commander import RobotCommander

class RobotHumanHandoverReachingModule(RobotCommander):
  sleep_dur = rospy.Duration(0.5)
  r2h_handv_feedback = RobotHumanHandoverReachingFeedback()
  r2h_handv_result = RobotHumanHandoverReachingResult()

  def __init__(self) -> None:
    super().__init__()
  
    self.r2h_handv_as = actionlib.SimpleActionServer('/robot_to_human_handover_reaching',RobotHumanHandoverReachingAction,execute_cb=self.r2h_handover_cb,auto_start=False)

    trg_sub = rospy.Subscriber("handover_target_point",CartesianTrajectoryPoint,callback=self.handover_target_cb)

    self.target = Pose()

    self.r2h_handv_as.start()

  def handover_target_cb(self, msg : CartesianTrajectoryPoint):
    self.target = msg.pose 

  def r2h_handover_cb(self, goal : RobotHumanHandoverReachingGoal):
    self.r2h_handv_result.success = True
    self.r2h_handv_feedback.percentage = 100

    # initialize wrist_dynamics_module making use of previous calibration (assumed valid)
    self.wrist_dyn.start_node(calib=True)

    # start to store proprioceptive information to further thresholding, but first wait a bit to get filter adapt to calib offset
    rospy.sleep(self.sleep_dur)
    self.wrist_dyn.set_save_interaction()
    
    # move to start position
    self.arm.set_dmp_ratio(1) #just to make the dmp following faster the current target
    self.arm.set_stop_time(goal.stop_time)
    self.arm.set_max_accel(0.2) # use lower accelration to reach the home position
    self.arm.set_max_angaccel(0.2)
    self.arm.set_harmonic_traj_generator()
    self.arm.switch_to_cartesian_controller('cartesian_motion_position_controller')
    self.arm.set_pose_target(goal.home)

    # prepare hand to open in a separate thread
    self.hand.switch_to_controller('mia_hand_joint_group_vel_controller')
    open_thread = Thread(target=self.hand.open,args=[False,3])

    # change to dmp
    # self.arm.set_dmp_traj_generator()
    # self.arm.set_dmp_ratio(.2) #just to set a reasonable ratio for reaching
    self.arm.set_max_accel(goal.max_accel) # not rqeuired for dmp
    self.arm.set_max_angaccel(goal.max_angaccel)
    rospy.sleep(self.sleep_dur)

    # go to target pose and monitor execution
    self.arm.set_pose_target(goal.target,False)
    self.arm.update_trajectory_monitor()
    
    # wait at least for half of the nominal trajectory, then start to trigger
    rate = rospy.Rate(40)
    while self.arm.percentage < 50:
      rate.sleep()  
    self.wrist_dyn.set_trigger_dynamics()

    # update dmp target according to human hand pose, break on detection
    self.wrist_dyn.detection.dynamic_contact = False
    # while self.arm.percentage < 100:
    while not self.wrist_dyn.detection.dynamic_contact:
      # self.arm.set_pose_target(self.target)
      # self.arm.update_trajectory_monitor()
      rate.sleep()
      
    # open the hand, stop arm immediately at the end of the loop
    open_thread.start()
    self.arm.stop()

    # wait a bit before set wrist_dynamics_module to idle
    rospy.sleep(rospy.Duration(goal.sleep))
    self.wrist_dyn.stop_loop()

    # wait a bit and retire to back position
    rospy.sleep(rospy.Duration(goal.sleep))
    self.arm.set_max_accel(0.2) # use lower accelration to reach the back position
    self.arm.set_max_angaccel(0.2)
    self.arm.set_harmonic_traj_generator()
    self.arm.switch_to_cartesian_controller('cartesian_motion_position_controller')
    self.arm.set_pose_target(goal.back)

    # join the open thread
    open_thread.join()

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