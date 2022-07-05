import rospy

from geometry_msgs.msg import Transform, Twist, PoseStamped
from cartesian_control_msgs.msg import CartesianTrajectoryPoint
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint

from artificial_hands_msgs.msg import CartesianTrajectoryPointStamped

class CartesianMDOFPointPublisher:

  def __init__(self, target : str = '/target_traj_mdof_pnt') -> None:

    self.pub = rospy.Publisher(target,MultiDOFJointTrajectoryPoint,queue_size=1000)
    sub = rospy.Subscriber("/target_traj_pnt",CartesianTrajectoryPointStamped,callback=self.target_traj_point_cb)

    self.transform = Transform()
    self.twist = Twist()

    self.mdof_pnt = MultiDOFJointTrajectoryPoint()
    self.mdof_pnt.transforms.append(self.transform)
    self.mdof_pnt.velocities.append(self.twist)

  def target_traj_point_cb(self, msg : CartesianTrajectoryPointStamped):

    self.transform.translation = msg.point.pose.position
    self.transform.rotation = msg.point.pose.orientation
    self.twist = msg.point.twist
    if not rospy.is_shutdown():
      self.pub.publish(self.mdof_pnt)

class PoseStampedPublisher:

  def __init__(self, target : str = '/target_pose_stamped') -> None:

    self.pub = rospy.Publisher(target,PoseStamped,queue_size=1000)
    sub = rospy.Subscriber("/target_traj_pnt",CartesianTrajectoryPointStamped,callback=self.target_traj_point_cb)

    self.target_pose = PoseStamped()

  def target_traj_point_cb(self, msg : CartesianTrajectoryPointStamped):

    self.target_pose.header = msg.header
    self.target_pose.header.frame_id = 'base'
    self.target_pose.pose = msg.point.pose
    if not rospy.is_shutdown():
      self.pub.publish(self.target_pose)

class CartesianTrajectoryPointPublisher:

  def __init__(self, target : str = '/target_pose_stamped') -> None:

    self.pub = rospy.Publisher(target,CartesianTrajectoryPoint,queue_size=1000)
    sub = rospy.Subscriber("/target_traj_pnt",CartesianTrajectoryPointStamped,callback=self.target_traj_point_cb)

    self.target_pose = CartesianTrajectoryPoint()

  def target_traj_point_cb(self, msg : CartesianTrajectoryPointStamped):

    self.target_pose.pose = msg.point.pose
    self.target_pose.twist = msg.point.twist
    if not rospy.is_shutdown():
      self.pub.publish(self.target_pose)

