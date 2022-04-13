from math import floor
from threading import Thread, Lock

import rospy, actionlib
import tf.transformations as ts
from geometry_msgs.msg import Quaternion
from cartesian_control_msgs.msg import CartesianTrajectoryPoint

from artificial_hands_msgs.msg import *
from artificial_hands_py.artificial_hands_py_base import list_to_quat, quat_to_list
from artificial_hands_py.cartesian_trajectory_generator.cartesian_trajectory_base import *
from artificial_hands_py.cartesian_trajectory_generator.cartesian_trajectory_plugins import *

class CartesianTrajectoryGenerator:
  traj_rate = 125
  traj_feedback = TrajectoryGenerationFeedback()
  traj_result = TrajectoryGenerationResult()

  dmp_plugin = DMPTrajectoryPlugin()

  lock = Lock()

  def __init__(self) -> None:
    
    self.gen_as = actionlib.SimpleActionServer('cartesian_trajectory_plugin_manager', TrajectoryGenerationAction, execute_cb=self.trajectory_plugin_cb, auto_start = False)
    self.traj_as = actionlib.SimpleActionServer('cartesian_trajectory_generator', TrajectoryGenerationAction, execute_cb=self.trajectory_generation_cb, auto_start = False)
    
    self.traj_pnt_pub = rospy.Publisher('target_traj_pnt',CartesianTrajectoryPointStamped,queue_size=1000)

    self.target = CartesianTrajectoryPoint()
    self.target.pose.orientation.w = 1
    self.target.time_from_start = rospy.Duration.from_sec(1/self.traj_rate)
    self.rate = rospy.Rate(self.traj_rate)

    target_timer = rospy.Timer(rospy.Duration.from_sec(1/self.traj_rate),self.update)

    self.plugin_thread = Thread()
    self.plugin_running = False

    self.gen_as.start()
    self.traj_as.start()

  def copy_plugin_target(self, plugin_target : CartesianTrajectoryPoint):
    self.plugin_running = True
    while self.plugin_running:
      self.target.pose = plugin_target.pose
      self.target.twist = plugin_target.twist
      self.target.acceleration = plugin_target.acceleration
      self.rate.sleep()

  def trajectory_plugin_cb(self, goal : TrajectoryGenerationGoal):
    
    self.traj_result.success = True

    if self.plugin_thread.is_alive():
      self.plugin_running = False
      self.plugin_thread.join()

    if goal.traj_type == goal.DMP:
      self.plugin_thread = Thread(target=self.copy_plugin_target,args=(self.dmp_plugin.plugin_target,))
      self.plugin_thread.start()
    
    self.gen_as.set_succeeded(self.traj_result)

  def trajectory_generation_cb(self, goal : TrajectoryGenerationGoal):

    self.traj_result.success = True

    self.dmp_plugin.set_plugin_target(goal.traj_target)

    if goal.traj_type == goal.FORWARD:
      self.target = goal.traj_target
      self.traj_as.set_succeeded(self.traj_result)
      return
    elif goal.traj_type == goal.DMP:   
      self.traj_as.set_succeeded(self.traj_result)
      return

    dt = 1/self.traj_rate

    c_target = CartesianTrajectoryPoint()
    c_target.pose.position.x = self.target.pose.position.x
    c_target.pose.position.y = self.target.pose.position.y
    c_target.pose.position.z = self.target.pose.position.z
    c_target.pose.orientation.x = self.target.pose.orientation.x
    c_target.pose.orientation.y = self.target.pose.orientation.y
    c_target.pose.orientation.z = self.target.pose.orientation.z
    c_target.pose.orientation.w = self.target.pose.orientation.w

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

    if goal.traj_max_accel < 0:
      goal.traj_max_accel = -goal.traj_max_accel
    elif goal.traj_max_accel == 0:
      goal.traj_max_accel = 0.5
    elif goal.traj_max_accel > 1.5:
      goal.traj_max_accel = 1.5
    if goal.traj_max_angaccel < 0:
      goal.traj_max_angaccel = -goal.traj_max_angaccel
    elif goal.traj_max_angaccel == 0:
      goal.traj_max_angaccel = 1.5
    elif goal.traj_max_angaccel > 1:
      goal.traj_max_angaccel = 1

    if goal.traj_type == goal.HARMONIC:
      
      h = max(abs(delta_target.pose.position.x),abs(delta_target.pose.position.y),abs(delta_target.pose.position.z))
      h_rot = abs(delta_angle)

      if h == 0 and h_rot == 0:
        self.traj_as.set_succeeded(self.traj_result)
        return

      goal_time = max(pow(2*pi*h/goal.traj_max_accel,.5),pow(2*pi*h_rot/goal.traj_max_angaccel,.5))

      c_max = goal_time/dt
      if floor(c_max) != c_max:
        c_max = floor(c_max) + 1
        goal_time = c_max*dt

      self.traj_feedback.goal_time = goal_time
      self.traj_feedback.max_vel.x = 2*delta_target.pose.position.x/goal_time
      self.traj_feedback.max_vel.y = 2*delta_target.pose.position.y/goal_time
      self.traj_feedback.max_vel.z = 2*delta_target.pose.position.z/goal_time
      self.traj_feedback.max_accel.x = 2*pi*delta_target.pose.position.x/pow(goal_time,2)
      self.traj_feedback.max_accel.y = 2*pi*delta_target.pose.position.y/pow(goal_time,2)
      self.traj_feedback.max_accel.z = 2*pi*delta_target.pose.position.z/pow(goal_time,2)

      for c in range(0,c_max+1):
        
        self.target.pose.position.x = c_target.pose.position.x + harmonic_pos(delta_target.pose.position.x,c*dt,goal_time)
        self.target.pose.position.y = c_target.pose.position.y + harmonic_pos(delta_target.pose.position.y,c*dt,goal_time)
        self.target.pose.position.z = c_target.pose.position.z + harmonic_pos(delta_target.pose.position.z,c*dt,goal_time)
        self.target.twist.linear.x = harmonic_vel(delta_target.pose.position.x,c*dt,goal_time)
        self.target.twist.linear.y = harmonic_vel(delta_target.pose.position.y,c*dt,goal_time)
        self.target.twist.linear.z = harmonic_vel(delta_target.pose.position.z,c*dt,goal_time)
        self.target.acceleration.linear.x = harmonic_accel(delta_target.pose.position.x,c*dt,goal_time)
        self.target.acceleration.linear.y = harmonic_accel(delta_target.pose.position.y,c*dt,goal_time)
        self.target.acceleration.linear.z = harmonic_accel(delta_target.pose.position.z,c*dt,goal_time)

        angle = harmonic_pos(delta_angle,c*dt,goal_time)
        self.target.pose.orientation = list_to_quat(
          ts.quaternion_multiply(ts.quaternion_from_matrix(ts.rotation_matrix(angle,delta_axis,delta_point)),quat_to_list(c_target.pose.orientation)))

        self.traj_feedback.percentage = 100*c/c_max
        self.traj_as.publish_feedback(self.traj_feedback)

        self.rate.sleep()
      
      if self.traj_result.success:
        self.traj_as.set_succeeded(self.traj_result)
  
  def update(self,event):
    msg = CartesianTrajectoryPointStamped()
    msg.header.stamp = rospy.Time.now()
    msg.point = self.target
    self.traj_pnt_pub.publish(msg) 
  
  def trajectory_generator_target(self):   
    msg = CartesianTrajectoryPointStamped()
    while not rospy.is_shutdown():
      msg.header.stamp = rospy.Time.now()
      msg.point = self.target
      self.traj_pnt_pub.publish(msg) 
      self.rate.sleep()     

def main():

  rospy.init_node('cartesian_trajectory_generator_node')

  cart_traj_gen = CartesianTrajectoryGenerator()

  rospy.loginfo('Cartesian trajectory generator ready!')

  rospy.spin()

if __name__ == '__main__':
  main()