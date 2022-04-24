import rospy,actionlib
import tf2_ros
import moveit_commander.conversions as cv

from geometry_msgs.msg import Pose, PoseStamped, Point

from artificial_hands_msgs.msg import *
from artificial_hands_py.robot_commander.controller_manager_base  import ControllerManagerBase

class ArmCommander(ControllerManagerBase):

  c_gen_cl = actionlib.SimpleActionClient('/cartesian_trajectory_plugin_manager',TrajectoryGenerationAction)
  c_traj_cl = actionlib.SimpleActionClient('/cartesian_trajectory_generator',TrajectoryGenerationAction)
  c_mon_cl = actionlib.SimpleActionClient('/cartesian_trajectory_tf_monitor',TrajectoryGenerationAction)

  tf_buffer = tf2_ros.Buffer(rospy.Duration(1))

  def __init__(self, ns: str = '', ref: str = 'base', eef: str = 'tool0', move_group: str = 'manipulator', ctrl_dict: dict = None) -> None:
    super().__init__(ns, ctrl_dict)
    
    self.ref_frame = ref
    self.ee_frame = eef 

    self.percentage = 0

    self.goal = TrajectoryGenerationGoal()
    self.goal.header.frame_id = ref
    self.goal.controlled_frame = eef

    tf_listener = tf2_ros.TransformListener(self.tf_buffer)    

  def switch_to_cartesian_controller(self, ctrl_name : str):
    self.pause_all_controllers()
    self.set_pose_target(self.get_current_frame().pose)
    self.switch_to_controller(ctrl_name)
    
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
  
  def set_stop_time(self,stop_time):
    self.goal.stop_time = stop_time

  def set_stop_factor(self,stop_factor):
    self.goal.stop_factor = stop_factor

  def set_forward_traj_point(self):
    self.goal.traj_type = self.goal.FORWARD
    self.c_gen_cl.send_goal_and_wait(self.goal)
  
  def set_dmp_traj_generator(self):
    self.goal.traj_type = self.goal.DMP
    self.c_gen_cl.send_goal_and_wait(self.goal)

  def set_harmonic_traj_generator(self):
    self.goal.traj_type = self.goal.HARMONIC
    self.c_gen_cl.send_goal_and_wait(self.goal)
  
  def set_poly_345_traj_generator(self):
    self.goal.traj_type = self.goal.POLY345
    self.c_gen_cl.send_goal_and_wait(self.goal)

  def set_max_accel(self,max_accel : float):
    self.goal.traj_max_accel = max_accel
  
  def set_max_angaccel(self,max_angaccel : float):
    self.goal.traj_max_angaccel = max_angaccel

  def update_trajectory_monitor(self):
    self.percentage = 0
    self.c_mon_cl.send_goal(self.goal,feedback_cb=self.trajectory_monitor_cb)
  
  def wait_for_trajectory_monitor(self):
    self.update_trajectory_monitor()
    rate = rospy.Rate(30)
    while self.percentage < 100:
      rate.sleep()

  def trajectory_monitor_cb(self, feedback : TrajectoryGenerationFeedback):
    self.percentage = feedback.percentage
