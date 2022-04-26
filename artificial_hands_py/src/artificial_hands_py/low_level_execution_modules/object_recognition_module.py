import rospy, actionlib

from artificial_hands_msgs.msg import *
from artificial_hands_py.robot_commander.robot_commander import RobotCommander, RobotManager

class ObjectRecognitionModule:
  sleep_dur = rospy.Duration(0.5)
  recog_feedback = ObjectRecognitionFeedback()
  recog_result = ObjectRecognitionResult()

  def __init__(self) -> None:

    self.robot : RobotCommander
    self.robot = RobotManager.get()
  
    self.recog_as = actionlib.SimpleActionServer('/object_recognition',ObjectRecognitionAction,execute_cb=self.recognition_cb,auto_start=False)

    self.recog_as.start()

  def recognition_cb(self, goal : ObjectRecognitionGoal):
    self.recog_result.success = True
    self.recog_feedback.percentage = 100

    # initialize wrist_dynamics_module making use of previous calibration (assumed valid)
    self.robot.wrist_dyn.start_node(calib=True)
    self.robot.wrist_dyn.set_publish()
    
    self.robot.arm.set_max_accel(goal.max_accel)
    self.robot.arm.set_max_angaccel(goal.max_angaccel)
    self.robot.arm.set_harmonic_traj_generator()
    self.robot.arm.switch_to_cartesian_controller('cartesian_motion_position_controller')
    self.robot.arm.set_pose_target(goal.home)

    # start to fill wrist_dynamics matrix
    self.robot.wrist_dyn.set_save_dynamics()
    rospy.sleep(self.sleep_dur)

    # go to target pose
    self.robot.arm.set_pose_target(goal.target)
    rospy.sleep(self.sleep_dur)

    # get wrist_dynamics_module to idle and build the object inertial model
    self.robot.wrist_dyn.stop_loop()
    self.robot.wrist_dyn.build_model()

    # stop controllers
    self.robot.arm.pause_all_controllers()
    self.recog_as.set_succeeded(self.recog_result)

def main():

  rospy.init_node('object_recognition_module_node')

  obj_rec_mod = ObjectRecognitionModule()

  rospy.loginfo('Object recognition module ready!')

  rospy.spin()

if __name__ == '__main__':
  main()