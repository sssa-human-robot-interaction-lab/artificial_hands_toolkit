from artificial_hands_py.artificial_hands_py_base import norm_quat
import rospy, actionlib

from geometry_msgs.msg import Quaternion, QuaternionStamped
from cartesian_control_msgs.msg import CartesianTrajectoryPoint
from dmp_extended.msg import MJerkTrackTarget, DesiredTrajectory

class DMPTrajectoryPlugin:
  lin_vel_tolerance = 1e-02
  ang_vel_tolerance = 1e-02

  def __init__(self) -> None:
    
    self.pos_pub = rospy.Publisher('/dmp/target_position_state',MJerkTrackTarget,queue_size=1000)
    self.rot_pub = rospy.Publisher('/dmp/target_orientation',QuaternionStamped,queue_size=1000)
    sub = rospy.Subscriber("/target_traj",DesiredTrajectory,callback=self.target_cb)

    self.plugin_target = CartesianTrajectoryPoint()
    self.target_pos = MJerkTrackTarget()
    self.target_rot = QuaternionStamped()
  
  def target_cb(self, msg : DesiredTrajectory):
    self.plugin_target.pose = msg.pose
    self.plugin_target.twist = msg.twist
    self.plugin_target.acceleration = msg.twd

  def set_plugin_target(self, target : CartesianTrajectoryPoint):
    
    self.target_pos.pos = target.pose.position
    self.target_pos.vel = target.twist.linear
    self.target_pos.acc = target.acceleration.linear

    # self.target_rot.quaternion = norm_quat(target.pose.orientation)

    self.target_rot.quaternion = target.pose.orientation

    self.pos_pub.publish(self.target_pos)
    self.rot_pub.publish(self.target_rot)
  
  def is_active(self) -> bool:
    active = self.plugin_target.twist.linear.x + self.plugin_target.twist.linear.y + self.plugin_target.twist.linear.z > self.lin_vel_tolerance
    active = active and self.plugin_target.twist.angular.x + self.plugin_target.twist.angular.y + self.plugin_target.twist.angular.z > self.ang_vel_tolerance
    return active