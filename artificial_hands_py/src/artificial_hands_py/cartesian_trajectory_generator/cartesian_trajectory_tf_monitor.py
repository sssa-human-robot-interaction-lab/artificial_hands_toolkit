import rospy, actionlib, tf2_ros
import tf.transformations as ts
import moveit_commander.conversions as cv
from geometry_msgs.msg import Quaternion
from cartesian_control_msgs.msg import CartesianTrajectoryPoint

from artificial_hands_msgs.msg import *
from artificial_hands_py.artificial_hands_py_base import list_to_quat, quat_to_list
from artificial_hands_py.cartesian_trajectory_generator.cartesian_trajectory_base import *
from artificial_hands_py.cartesian_trajectory_generator.cartesian_trajectory_plugins import *

class CartesianTrajectoryTFMonitor:
  angle_tol = 1e-02
  position_tol = 1e-04
  
  mon_rate = 40
  mon_feedback = TrajectoryGenerationFeedback()
  mon_result = TrajectoryGenerationResult()

  tf_buffer = tf2_ros.Buffer(rospy.Duration(1))

  def __init__(self) -> None:
    
    self.mon_as = actionlib.SimpleActionServer('cartesian_trajectory_tf_monitor', TrajectoryGenerationAction, execute_cb=self.trajectory_monitor_cb, auto_start = False)
    
    self.rate = rospy.Rate(self.mon_rate)

    tf_listener = tf2_ros.TransformListener(self.tf_buffer)    

    self.mon_as.start()

  def trajectory_monitor_cb(self, goal : TrajectoryGenerationGoal):

    self.mon_feedback.percentage = 0

    self.mon_result.success = True

    ref_to_frame = self.tf_buffer.lookup_transform(goal.header.frame_id,goal.controlled_frame,rospy.Time(0),timeout=rospy.Duration(1))
  
    c_target = cv.list_to_pose_stamped(cv.transform_to_list(ref_to_frame.transform),goal.header.frame_id)

    inv_c_quat = Quaternion()
    inv_c_quat.x = c_target.pose.orientation.x
    inv_c_quat.y = c_target.pose.orientation.y
    inv_c_quat.z = c_target.pose.orientation.z
    inv_c_quat.w = -c_target.pose.orientation.w

    delta_target = CartesianTrajectoryPoint()
    delta_target.pose.position.x = goal.traj_target.pose.position.x - c_target.pose.position.x
    delta_target.pose.position.y = goal.traj_target.pose.position.y - c_target.pose.position.y
    delta_target.pose.position.z = goal.traj_target.pose.position.z - c_target.pose.position.z
    delta_target.pose.orientation = list_to_quat(ts.quaternion_multiply(quat_to_list(goal.traj_target.pose.orientation),quat_to_list(inv_c_quat)))
    
    delta_rot = ts.quaternion_matrix(quat_to_list(delta_target.pose.orientation))
    delta_angle,delta_axis,delta_point = ts.rotation_from_matrix(delta_rot)

    delta_position = pow(pow(delta_target.pose.position.x,2)+pow(delta_target.pose.position.y,2)+pow(delta_target.pose.position.z,2),.5)

    self.mon_feedback.delta_angle = delta_angle
    self.mon_feedback.delta_position = delta_position
    
    if delta_angle < self.angle_tol and delta_position < self.position_tol:
      self.mon_feedback.percentage = 100
      self.mon_as.publish_feedback(self.mon_feedback)
      self.mon_as.set_succeeded(self.mon_result)
      return

    while self.mon_feedback.percentage < 99.9 and not self.mon_as.is_preempt_requested() and not rospy.is_shutdown():

      a_ref_to_frame = self.tf_buffer.lookup_transform(goal.header.frame_id,goal.controlled_frame,rospy.Time(0),timeout=rospy.Duration(1))

      a_target = cv.list_to_pose_stamped(cv.transform_to_list(a_ref_to_frame.transform),goal.header.frame_id)

      a_delta_target = CartesianTrajectoryPoint()
      a_delta_target.pose.position.x = a_target.pose.position.x - c_target.pose.position.x
      a_delta_target.pose.position.y = a_target.pose.position.y - c_target.pose.position.y
      a_delta_target.pose.position.z = a_target.pose.position.z - c_target.pose.position.z
      a_delta_target.pose.orientation = list_to_quat(ts.quaternion_multiply(quat_to_list(a_target.pose.orientation),quat_to_list(inv_c_quat)))
      
      a_delta_rot = ts.quaternion_matrix(quat_to_list(a_delta_target.pose.orientation))
      a_delta_angle,a_delta_axis,a_delta_point = ts.rotation_from_matrix(a_delta_rot)

      a_delta_position = pow(pow(a_delta_target.pose.position.x,2)+pow(a_delta_target.pose.position.y,2)+pow(a_delta_target.pose.position.z,2),.5)

      if delta_angle < self.angle_tol:
        self.mon_feedback.percentage = 100*(a_delta_position/delta_position)
      elif delta_position < self.position_tol:
        self.mon_feedback.percentage = 100*abs(a_delta_angle/delta_angle)
      elif delta_angle < self.angle_tol and delta_position < self.position_tol:
        self.mon_feedback.percentage = 100
      else:
        self.mon_feedback.percentage = 50*(a_delta_position/delta_position+abs(a_delta_angle/delta_angle))
      
      self.mon_as.publish_feedback(self.mon_feedback)
      self.rate.sleep()

    self.mon_feedback.percentage = 100
    self.mon_as.publish_feedback(self.mon_feedback)
    self.rate.sleep()

    if self.mon_result.success:
      self.mon_as.set_succeeded(self.mon_result)     

def main():

  rospy.init_node('cartesian_trajectory_tf_monitor_node')

  cart_traj_tf_mon = CartesianTrajectoryTFMonitor()

  rospy.loginfo('Cartesian trajectory tf monitor ready!')

  rospy.spin()

if __name__ == '__main__':
  main()