import rospy

from geometry_msgs.msg import QuaternionStamped
from cartesian_control_msgs.msg import CartesianTrajectoryPoint
from dmp_extended.msg import MJerkTrackTarget, DesiredTrajectory

class DMPTrajectoryPlugin:
  lin_vel_tolerance = 1e-02
  lin_accel_tolerance = 1e-02
  ang_vel_tolerance = 1e-02
  ang_accel_tolerance = 1e-02

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

    self.target_rot.quaternion = target.pose.orientation

    self.pos_pub.publish(self.target_pos)
    self.rot_pub.publish(self.target_rot)
  
  def is_active(self) -> bool:
    active = abs(self.plugin_target.twist.linear.x) + abs(self.plugin_target.twist.linear.y) + abs(self.plugin_target.twist.linear.z) > self.lin_vel_tolerance
    active = active or abs(self.plugin_target.acceleration.linear.x) + abs(self.plugin_target.acceleration.linear.y) + abs(self.plugin_target.acceleration.linear.z) > self.lin_accel_tolerance
    active = active or abs(self.plugin_target.twist.angular.x) + abs(self.plugin_target.twist.angular.y) + abs(self.plugin_target.twist.angular.z) > self.ang_vel_tolerance
    active = active or abs(self.plugin_target.acceleration.angular.x) + abs(self.plugin_target.acceleration.angular.y) + abs(self.plugin_target.acceleration.angular.z) > self.ang_accel_tolerance
    return active
    