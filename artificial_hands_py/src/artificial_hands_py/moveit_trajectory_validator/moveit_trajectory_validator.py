from threading import Thread

import rospy
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest, GetStateValidityResponse
from geometry_msgs.msg import Transform
from std_msgs.msg import Bool

from artificial_hands_msgs.msg import CartesianTrajectoryPointStamped

class MoveitTrajectoryValidator:
  
  def __init__(self) -> None:
    rospy.wait_for_service('/check_state_validity')
    traj_sub = rospy.Subscriber('target_traj_pnt',CartesianTrajectoryPointStamped,self.trajectory_point_cb)

    self.valid_pub = rospy.Publisher('moveit_trajectory_validator/status',Bool,queue_size=1000)

    self.target = CartesianTrajectoryPointStamped()

    valid_thread = Thread(target=self.trajectory_point_validation)
    valid_thread.start()

  def trajectory_point_cb(self, msg : CartesianTrajectoryPointStamped):
    self.target = msg

  def trajectory_point_validation(self):
    valid = Bool()
    
    valid_rate = rospy.Rate(20)
    dt = 1/20
    
    valid_ser = rospy.ServiceProxy('/check_state_validity',GetStateValidity)
    valid_req = GetStateValidityRequest()
    valid_res = GetStateValidityResponse()
   
    while not rospy.is_shutdown():
      
      future_target = Transform()
      future_target.translation.x = self.target.point.pose.position.x + self.target.point.twist.linear.x*dt
      future_target.translation.y = self.target.point.pose.position.y + self.target.point.twist.linear.y*dt
      future_target.translation.z = self.target.point.pose.position.z + self.target.point.twist.linear.z*dt
      
      valid_req.robot_state.multi_dof_joint_state.transforms.append(future_target)
      
      valid_res = valid_ser(valid_req)
      
      valid.data = valid_res.valid
      
      self.valid_pub.publish(valid)

      valid_rate.sleep()

def main():
  rospy.init_node('moveit_trajectory_validator_node')
  mov_traj_val = MoveitTrajectoryValidator()
  rospy.spin()

if __name__ == '__main__':
  main()
