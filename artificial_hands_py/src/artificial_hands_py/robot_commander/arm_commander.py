import rospy, actionlib
import tf2_ros
import moveit_commander.conversions as cv

from geometry_msgs.msg import Pose, PoseStamped, Point
from trajectory_msgs.msg import JointTrajectory
from cartesian_control_msgs.msg import CartesianTrajectoryPoint

from artificial_hands_msgs.msg import *
from artificial_hands_py.artificial_hands_py_base import singleton
from artificial_hands_py.robot_commander.controller_manager_base  import ControllerManagerBase
from artificial_hands_py.cartesian_trajectory_generator.cartesian_publishers import CartesianTrajectoryPointPublisher, PoseStampedPublisher

@singleton
class ArmCommander(ControllerManagerBase):

  j_traj_pos_ctrl = 'scaled_pos_joint_traj_controller'
  cart_mot_pos_ctrl = 'cartesian_motion_position_controller'
  cart_mot_eik_ctrl = 'cartesian_eik_position_controller'

  ctrl_dict = {j_traj_pos_ctrl : JointTrajectory,
               cart_mot_pos_ctrl : CartesianTrajectoryPoint,
               cart_mot_eik_ctrl : CartesianTrajectoryPoint}

  c_gen_cl = actionlib.SimpleActionClient('/cartesian_trajectory_plugin_manager',TrajectoryGenerationAction)
  c_traj_cl = actionlib.SimpleActionClient('/cartesian_trajectory_generator',TrajectoryGenerationAction)
  c_mon_cl = actionlib.SimpleActionClient('/cartesian_trajectory_tf_monitor',TrajectoryGenerationAction)

  tf_buffer = tf2_ros.Buffer(rospy.Duration(1))

  def __init__(self, ns: str = '', ref: str = 'base', eef: str = 'tool0') -> None:
    super().__init__(ns, self.ctrl_dict)

    self.boot = True

    self.ref_frame = ref
    self.ee_frame = eef 

    self.percentage = 0

    self.goal = TrajectoryGenerationGoal()
    self.goal.header.frame_id = ref
    self.goal.controlled_frame = eef

    cart_mot_pos_pub = CartesianTrajectoryPointPublisher(self.cart_mot_pos_ctrl+'/command')
    cart_mot_eik_pub = CartesianTrajectoryPointPublisher(self.cart_mot_eik_ctrl+'/command')

    tf_listener = tf2_ros.TransformListener(self.tf_buffer)   

  def switch_to_cartesian_controller(self, ctrl_name : str):
    if self.boot:
      rospy.loginfo('Booting cartesian trajectory generator...')
    self.pause_all_controllers()
    self.set_pose_target(self.get_current_frame().pose)
    self.switch_to_controller(ctrl_name)
    if self.boot:
      rospy.loginfo('Arm commander ready!')
      self.boot = False
    
  def get_current_frame(self,ref : str = None, frame : str = None) -> PoseStamped:
    if frame is None:
      frame = self.ee_frame
    if ref is None:
      ref = self.ref_frame
    ref_to_frame = self.tf_buffer.lookup_transform(ref,frame,rospy.Time(0),timeout=rospy.Duration(1))
    return cv.list_to_pose_stamped(cv.transform_to_list(ref_to_frame.transform),ref)
  
  def set_pose_target(self, pose_target : Pose, wait : bool = True):
    self.goal.traj_target.pose = pose_target
    self.c_traj_cl.send_goal(self.goal)
    if wait:
      self.c_traj_cl.wait_for_result()
  
  def set_position_target(self, position_target : Point, wait : bool = True):
    self.goal.traj_target.pose.position = position_target
    self.c_traj_cl.send_goal(self.goal)
    if wait:
      self.c_traj_cl.wait_for_result()
  
  def stop(self):
    self.goal.traj_type = self.goal.STOP
    self.c_traj_cl.send_goal_and_wait(self.goal)
  
  def cancel_pose_target(self):
    self.c_traj_cl.cancel_all_goals()
  
  def set_track_ratio(self,track_ratio : float):
    self.goal.track_ratio = track_ratio
    self.c_traj_cl.send_goal_and_wait(self.goal)
  
  def set_track_t_go(self,track_ratio : float):
    self.goal.track_ratio = track_ratio
    self.c_traj_cl.send_goal_and_wait(self.goal)
  
  def set_stop_time(self,stop_time : float):
    self.goal.stop_time = stop_time

  def set_forward_traj_point(self):
    self.goal.traj_type = self.goal.FORWARD
    self.c_gen_cl.send_goal_and_wait(self.goal)
  
  # def set_dmp_traj_generator(self):
  #   self.goal.traj_type = self.goal.DMP
  #   self.c_gen_cl.send_goal_and_wait(self.goal)
  
  def set_mj_traj_generator(self):
    self.goal.traj_type = self.goal.MJ
    self.c_gen_cl.send_goal_and_wait(self.goal)

  def set_harmonic_traj_generator(self):
    self.goal.traj_type = self.goal.HARMONIC
    self.c_gen_cl.send_goal_and_wait(self.goal)
  
  def set_poly_345_traj_generator(self):
    self.goal.traj_type = self.goal.POLY345
    self.c_gen_cl.send_goal_and_wait(self.goal)
  
  def set_poly_567_traj_generator(self):
    self.goal.traj_type = self.goal.POLY567
    self.c_gen_cl.send_goal_and_wait(self.goal)

  def set_mod_trapz_traj_generator(self):
    self.goal.traj_type = self.goal.TRAPZMOD
    self.c_gen_cl.send_goal_and_wait(self.goal)

  def set_alpha(self, alpha : float): 
    self.goal.trapz_alpha = alpha
  
  def set_max_vel(self, max_vel : float):
    self.goal.trapz_max_vel = max_vel
  
  def set_max_angvel(self, max_angvel : float):
    self.goal.trapz_max_angvel = max_angvel

  def set_max_accel(self,max_accel : float):
    self.goal.traj_max_accel = max_accel
  
  def set_max_angaccel(self,max_angaccel : float):
    self.goal.traj_max_angaccel = max_angaccel

  def update_trajectory_monitor(self):
    self.percentage = 0
    self.c_mon_cl.send_goal(self.goal,feedback_cb=self.trajectory_monitor_feedback_cb,done_cb=self.trajectory_monitor_result_cb)
  
  def wait_for_trajectory_monitor(self):
    self.update_trajectory_monitor()
    rate = rospy.Rate(30)
    while self.percentage < 100:
      rate.sleep()

  def trajectory_monitor_feedback_cb(self, feedback : TrajectoryGenerationFeedback):
    self.percentage = feedback.percentage

  def trajectory_monitor_result_cb(self, goal : TrajectoryGenerationGoal, result : TrajectoryGenerationResult):
    if result.success:
      self.percentage = 100

def main():

  rospy.init_node('arm_commander_node')

  arm = ArmCommander()

  rospy.loginfo('Arm commander ready!')

  rospy.spin()

if __name__ == '__main__':
  main()
