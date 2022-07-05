from time import sleep

import rospy, actionlib
from cartesian_control_msgs.msg import CartesianTrajectoryPoint
from trajectory_msgs.msg import JointTrajectoryPoint

from artificial_hands_msgs.msg import *
from artificial_hands_py.robot_commander.robot_commander import RobotCommander

class RobotMoveWaypointsModule(RobotCommander):
  sleep_dur = rospy.Duration(0.2)
  move_feedback = RobotMoveWaypointsFeedback()
  move_result = RobotMoveWaypointsResult()

  def __init__(self) -> None:
    super().__init__()
    
    self.move_as = actionlib.SimpleActionServer('/robot_move_waypoints',RobotMoveWaypointsAction,execute_cb=self.move_cb,auto_start=False)
    self.move_as.start()

  def move_cb(self, goal : RobotMoveWaypointsGoal):
    self.move_result.success = True
    self.move_feedback.percentage = 100

    # iterate through all waypoints only if ntheir number is consistent
    if len(goal.arm_waypoints.points) != len(goal.hand_waypoints.points):
      rospy.logerr('Inconsistent number of arm and hand waypoints')
      self.move_result.success = False
      return

    # go to home pose
    self.arm.set_max_accel(goal.max_accel)
    self.arm.set_max_angaccel(goal.max_angaccel)
    self.arm.set_harmonic_traj_generator()
    self.arm.switch_to_cartesian_controller('cartesian_motion_position_controller')

    for (arm_wp, hand_wp) in zip(goal.arm_waypoints.points, goal.hand_waypoints.points):

      # check for negative time from start (mantain current configuration)
      if arm_wp.time_from_start.secs >= 0:
        self.arm.set_pose_target(arm_wp.pose,wait=False)
      if hand_wp.time_from_start.secs >= 0:
        self.hand.set_joint_positions(hand_wp.positions)

      # wait for arm trajectory completion
      self.arm.wait_for_trajectory_monitor()
      rospy.sleep(self.sleep_dur)

    # stop controllers
    self.arm.pause_all_controllers()
    self.move_as.set_succeeded(self.move_result)

def main():

  rospy.init_node('robot_move_waypoints_module_node')

  obj_rec_mod = RobotMoveWaypointsModule()

  rospy.loginfo('Robot move wayoints odule ready!')

  rospy.spin()

if __name__ == '__main__':
  main()