import rospy

from geometry_msgs.msg import Transform, Twist
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

    self.pub.publish(self.mdof_pnt)

