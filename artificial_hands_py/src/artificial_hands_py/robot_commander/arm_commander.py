import rospy,actionlib
import tf2_ros
import moveit_commander.conversions as cv

from geometry_msgs.msg import Pose, PoseStamped

from artificial_hands_msgs.msg import *
from artificial_hands_py.robot_commander.controller_manager_base  import ControllerManagerBase

class ArmCommander(ControllerManagerBase):

  c_gen_cl = actionlib.SimpleActionClient('/trajectory_plugin_manager',TrajectoryGenerationAction)
  c_traj_cl = actionlib.SimpleActionClient('/cartesian_trajectory_generator',TrajectoryGenerationAction)

  tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))

  def __init__(self, ns: str = '', ref: str = 'base', eef: str = 'tool0', move_group: str = 'manipulator', ctrl_dict: dict = None) -> None:
    super().__init__(ns, ctrl_dict)
    self.c_traj_type = 2
    self.ref_frame = ref
    self.ee_frame = eef 
    tf_listener = tf2_ros.TransformListener(self.tf_buffer)

  def switch_to_trajectory_generator(self, gen_index : int):
    goal = TrajectoryGenerationGoal()
    
    if gen_index == 0:
      goal.traj_type = goal.FORWARD
    elif gen_index == 1:
      goal.traj_type = goal.DMP
    elif gen_index == 2:
      goal.traj_type = goal.HARMONIC

    self.c_traj_type = goal.traj_type
    self.c_gen_cl.send_goal(goal)

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
    goal = TrajectoryGenerationGoal()
    goal.traj_type = self.c_traj_type
    goal.traj_target.pose = pose_target
    self.c_traj_cl.send_goal(goal)
    if wait:
      self.c_traj_cl.wait_for_result()
  
  def cancel_pose_target(self):
    self.c_traj_cl.cancel_all_goals()
