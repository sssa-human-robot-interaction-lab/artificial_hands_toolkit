from time import sleep
import rospy, actionlib

from artificial_hands_msgs.msg import *
from artificial_hands_py.robot_commander.robot_commander import RobotCommander

class ObjectGraspModule(RobotCommander):
  sleep_dur = rospy.Duration(0.5)
  grasp_feedback = ObjectGraspFeedback()
  grasp_result = ObjectGraspResult()

  def __init__(self) -> None:
    super().__init__()
    
    self.grasp_as = actionlib.SimpleActionServer('/object_grasp',ObjectGraspAction,execute_cb=self.grasp_cb,auto_start=False)

    self.grasp_as.start()

  def grasp_cb(self, goal : ObjectGraspGoal):
    self.grasp_result.success = True
    self.grasp_feedback.percentage = 100

    # get hand in open position
    self.hand.set_joint_positions([0.4,0.0,0.0])

    # go to home pose
    self.arm.set_max_accel(goal.max_accel)
    self.arm.set_max_angaccel(goal.max_angaccel)
    self.arm.set_harmonic_traj_generator()
    self.arm.switch_to_cartesian_controller('cartesian_motion_position_controller')
    self.arm.set_pose_target(goal.home)

    # get hand in preshape
    self.hand.set_joint_positions(goal.preshape.data)

    # go to grasp pose with trapz
    self.arm.set_alpha(goal.alpha)
    self.arm.set_max_vel(goal.max_vel)
    self.arm.set_max_angvel(goal.max_angvel)
    self.arm.set_mod_trapz_traj_generator()
    self.arm.set_pose_target(goal.target)

    # get hand in closed shape
    self.hand.set_joint_positions(goal.shape.data)
    sleep(3)

    # go to back position
    self.arm.set_pose_target(goal.back)

    # stop controllers
    self.arm.pause_all_controllers()
    self.grasp_as.set_succeeded(self.grasp_result)

def main():

  rospy.init_node('object_grasp_module_node')

  obj_rec_mod = ObjectGraspModule()

  rospy.loginfo('Object grasp module ready!')

  rospy.spin()

if __name__ == '__main__':
  main()