import rospy,actionlib
import tf2_ros, tf2_geometry_msgs
import moveit_commander

from geometry_msgs.msg import Pose, PoseStamped
from trajectory_msgs.msg import JointTrajectory, MultiDOFJointTrajectoryPoint

from artificial_hands_msgs.msg import *
from artificial_hands_py.robot_commander.controller_manager_base  import ControllerManagerBase

class RobotCommander(ControllerManagerBase,moveit_commander.MoveGroupCommander):
  
  j_traj_ctrl = 'pos_joint_traj_controller'
  c_traj_ctrl = 'cartesian_motion_controller'

  c_traj_cl = actionlib.SimpleActionClient('/cartesian_trajectory_generator',TrajectoryGenerationAction)

  tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))

  def __init__(self, ns: str = '', ref: str = 'base', eef: str = 'tool0', move_group: str = 'manipulator') -> None:
    ctrl_dict = {self.j_traj_ctrl : JointTrajectory, self.c_traj_ctrl : PoseStamped}
    super().__init__(ns, ctrl_dict)
    super(ControllerManagerBase,self).__init__(move_group) 
    self.ref_frame = ref
    self.ee_frame = eef 
    tf_listener = tf2_ros.TransformListener(self.tf_buffer)

  def switch_to_joint_trajectory(self):
    self.switch_to_controller(self.j_traj_ctrl)
  
  def switch_to_cartesian_trajectory(self):
    self.switch_to_controller(self.c_traj_ctrl)
  
  def get_current_frame(self,ref : str = None, frame : str = None) -> PoseStamped:
    if frame is None:
      frame = self.ee_frame
    if ref is None:
      ref = self.ref_frame
    pose_ref : PoseStamped = self.get_current_pose(frame)
    ref_to_base = self.tf_buffer.lookup_transform(ref,pose_ref.header.frame_id,rospy.Time.now(),timeout=rospy.Duration(1))
    print(pose_ref)
    print(tf2_geometry_msgs.do_transform_pose(pose_ref,ref_to_base))
    return tf2_geometry_msgs.do_transform_pose(pose_ref,ref_to_base)

  def set_joint_target(self, joint_target : list, wait : bool = True, auto_switch : bool = True):
    self.go(joint_target,wait) 
  
  def set_pose_target(self, pose_target : Pose, wait : bool = True):
    goal = TrajectoryGenerationGoal()
    goal.traj_target.pose = pose_target
    self.c_traj_cl.send_goal(goal)
    if wait:
      self.c_traj_cl.wait_for_result()
  
  def cancel_pose_target(self):
    self.c_traj_cl.cancel_all_goals()
