import rospy

from std_msgs.msg import Float64
from geometry_msgs.msg import QuaternionStamped
from cartesian_control_msgs.msg import CartesianTrajectoryPoint
from dmp_extended.msg import MJerkTrackTarget, DesiredTrajectory
from minimum_jerk_se3_tracker.msg import MJTarget, TaskSpaceTrajectory
from minimum_jerk_se3_tracker.srv import *

from artificial_hands_py.artificial_hands_py_base import norm_quat, pose_copy

"""@fraiori minimum_jerk_se3_tracker"""
class MJTrajectoryPlugin:
  lin_vel_tolerance = 1e-02
  lin_accel_tolerance = 1e-02
  ang_vel_tolerance = 1e-02
  ang_accel_tolerance = 1e-02

  def __init__(self) -> None:
    
    self.target_pub = rospy.Publisher('/mjtracker/target',MJTarget,queue_size=1000)
    self.ratio_pub = rospy.Publisher('/mjtracker/desired_ratio',Float64,queue_size=1000)
    sub = rospy.Subscriber("/mjtracker/traj",TaskSpaceTrajectory,callback=self.target_cb)
    self.set_ser = rospy.ServiceProxy("/mj_trajectory_tracker/set_pose",SetPoseMJTracker)

    self.plugin_target = CartesianTrajectoryPoint()
    self.target = MJTarget()
    self.target.t_go.data = 1
    self.target_ratio = Float64()
  
  def target_cb(self, msg : TaskSpaceTrajectory):
    self.plugin_target.pose.position = msg.se3.position
    self.plugin_target.pose.orientation = msg.se3.orientation
    self.plugin_target.twist.linear = msg.se3d.linear
    self.plugin_target.twist.angular = msg.se3d.angular
    self.plugin_target.acceleration.linear = msg.se3dd.linear
    self.plugin_target.acceleration.angular = msg.se3dd.angular
  
  def set_plugin_target(self, target : CartesianTrajectoryPoint, ratio : float = 0.2):

    self.target_ratio.data = ratio

    self.ratio_pub.publish(self.target_ratio)
    
    self.target.se3 = target.pose
    self.target.se3d = target.twist
    self.target.se3dd = target.acceleration

    self.target_pub.publish(self.target)
  
  def set_plugin_state(self, target : CartesianTrajectoryPoint):

    set_req = SetPoseMJTrackerRequest()
    set_req.pose = pose_copy(target.pose)
    res = self.set_ser(set_req)
  
  def is_active(self) -> bool:
    active = abs(self.plugin_target.twist.linear.x) + abs(self.plugin_target.twist.linear.y) + abs(self.plugin_target.twist.linear.z) > self.lin_vel_tolerance
    active = active or abs(self.plugin_target.acceleration.linear.x) + abs(self.plugin_target.acceleration.linear.y) + abs(self.plugin_target.acceleration.linear.z) > self.lin_accel_tolerance
    active = active or abs(self.plugin_target.twist.angular.x) + abs(self.plugin_target.twist.angular.y) + abs(self.plugin_target.twist.angular.z) > self.ang_vel_tolerance
    active = active or abs(self.plugin_target.acceleration.angular.x) + abs(self.plugin_target.acceleration.angular.y) + abs(self.plugin_target.acceleration.angular.z) > self.ang_accel_tolerance
    return active

"""@fraiori dmp_extended"""
class DMPTrajectoryPlugin:
  lin_vel_tolerance = 1e-02
  lin_accel_tolerance = 1e-02
  ang_vel_tolerance = 1e-02
  ang_accel_tolerance = 1e-02

  def __init__(self) -> None:
    
    self.pos_pub = rospy.Publisher('/dmp/target_position_state',MJerkTrackTarget,queue_size=1000)
    self.rot_pub = rospy.Publisher('/dmp/target_orientation',QuaternionStamped,queue_size=1000)
    self.ratio_pub = rospy.Publisher('/dmp/desired_ratio',Float64,queue_size=1000)
    sub = rospy.Subscriber("/target_traj",DesiredTrajectory,callback=self.target_cb)

    self.plugin_target = CartesianTrajectoryPoint()
    self.target_pos = MJerkTrackTarget()
    self.target_rot = QuaternionStamped()
    self.target_ratio = Float64()
  
  def target_cb(self, msg : DesiredTrajectory):
    self.plugin_target.pose = msg.pose
    self.plugin_target.twist = msg.twist
    self.plugin_target.acceleration = msg.twd
  
  def set_plugin_target(self, target : CartesianTrajectoryPoint, ratio : float = 0.2):

    self.target_ratio.data = ratio

    self.ratio_pub.publish(self.target_ratio)
    
    self.target_pos.pos = target.pose.position
    self.target_pos.vel = target.twist.linear
    self.target_pos.acc = target.acceleration.linear

    self.target_rot.quaternion = norm_quat(target.pose.orientation)

    self.pos_pub.publish(self.target_pos)
    self.rot_pub.publish(self.target_rot)
  
  def is_active(self) -> bool:
    active = abs(self.plugin_target.twist.linear.x) + abs(self.plugin_target.twist.linear.y) + abs(self.plugin_target.twist.linear.z) > self.lin_vel_tolerance
    active = active or abs(self.plugin_target.acceleration.linear.x) + abs(self.plugin_target.acceleration.linear.y) + abs(self.plugin_target.acceleration.linear.z) > self.lin_accel_tolerance
    active = active or abs(self.plugin_target.twist.angular.x) + abs(self.plugin_target.twist.angular.y) + abs(self.plugin_target.twist.angular.z) > self.ang_vel_tolerance
    active = active or abs(self.plugin_target.acceleration.angular.x) + abs(self.plugin_target.acceleration.angular.y) + abs(self.plugin_target.acceleration.angular.z) > self.ang_accel_tolerance
    return active
    