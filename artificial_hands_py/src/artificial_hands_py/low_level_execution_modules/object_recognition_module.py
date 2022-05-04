import rospy, actionlib

from artificial_hands_msgs.msg import *
from artificial_hands_py.robot_commander.robot_commander import RobotCommander

class ObjectRecognitionModule(RobotCommander):
  sleep_dur = rospy.Duration(0.5)
  recog_feedback = ObjectRecognitionFeedback()
  recog_result = ObjectRecognitionResult()

  def __init__(self) -> None:
    super().__init__()
    
    self.recog_as = actionlib.SimpleActionServer('/object_recognition',ObjectRecognitionAction,execute_cb=self.recognition_cb,auto_start=False)

    self.recog_as.start()

  def recognition_cb(self, goal : ObjectRecognitionGoal):
    self.recog_result.success = True
    self.recog_feedback.percentage = 100

    # initialize wrist_dynamics_module making use of previous calibration (assumed valid)
    self.wrist_dyn.start_node(calib=True)
    self.wrist_dyn.set_publish()
    
    # set parameters for the trapz trajectory
    # self.arm.set_alpha(goal.alpha)
    # self.arm.set_max_vel(goal.max_vel)
    # self.arm.set_max_angvel(goal.max_angvel)

    self.arm.set_max_accel(goal.max_accel)
    self.arm.set_max_angaccel(goal.max_angaccel)
    self.arm.set_harmonic_traj_generator()
    self.arm.switch_to_cartesian_controller('cartesian_motion_position_controller')
    self.arm.set_pose_target(goal.home)

    # start to fill wrist_dynamics matrix
    self.wrist_dyn.set_save_dynamics()
    rospy.sleep(self.sleep_dur)

    # go to target pose with a trapz trajectory
    # self.arm.set_mod_trapz_traj_generator()
    self.arm.set_pose_target(goal.target)
    rospy.sleep(self.sleep_dur)

    # get wrist_dynamics_module to idle and build the object inertial model
    self.wrist_dyn.stop_loop()
    self.wrist_dyn.build_model()

    # stop controllers
    self.arm.pause_all_controllers()
    self.recog_as.set_succeeded(self.recog_result)

def main():

  rospy.init_node('object_recognition_module_node')

  obj_rec_mod = ObjectRecognitionModule()

  rospy.loginfo('Object recognition module ready!')

  rospy.spin()

if __name__ == '__main__':
  main()