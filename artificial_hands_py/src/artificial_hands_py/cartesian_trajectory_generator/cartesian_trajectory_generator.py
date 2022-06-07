from math import floor
from threading import Thread, Lock

import rospy, actionlib
import tf.transformations as ts
from geometry_msgs.msg import Quaternion, Twist
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


  def trajectory_plugin_cb(self, goal : TrajectoryGenerationGoal):

    self.traj_result.success = True

    if self.plugin_thread.is_alive():
      self.plugin_running = False
      self.plugin_thread.join()

    if goal.traj_type == goal.DMP:
      while self.dmp_plugin.is_active():
        self.rate.sleep()
      self.plugin_thread = Thread(target=self.copy_plugin_target,args=(self.dmp_plugin.plugin_target,))
      self.plugin_thread.start()

    self.gen_as.set_succeeded(self.traj_result)

  def trajectory_generation_cb(self, goal : TrajectoryGenerationGoal):

    self.traj_result.success = True

    dt = 1/self.traj_rate

    if goal.stop_time < 0.1 or goal.stop_time > 0.5:
      goal.stop_time = 0.1
    
    if goal.dmp_ratio < 0.1 or goal.dmp_ratio > 1:
      goal.dmp_ratio = 0.1

    if goal.traj_type == goal.STOP:
      if self.plugin_running:
        self.stop_plugin_target(goal.stop_time,dt)
      self.traj_as.set_succeeded(self.traj_result)
      return

    self.dmp_plugin.set_plugin_target(goal.traj_target,goal.dmp_ratio)

    if goal.traj_type == goal.FORWARD:
      self.target = goal.traj_target
      self.traj_as.set_succeeded(self.traj_result)
      return
    elif goal.traj_type == goal.DMP:
      self.traj_as.set_succeeded(self.traj_result)
      return

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

    h = max(abs(delta_target.pose.position.x),abs(delta_target.pose.position.y),abs(delta_target.pose.position.z))
    h_rot = abs(delta_angle)

    if h == 0 and h_rot == 0:
      self.traj_as.set_succeeded(self.traj_result)
      return

    if goal.traj_max_accel <= 0:
      goal.traj_max_accel = 0.5
    elif goal.traj_max_accel > 1.5:
      goal.traj_max_accel = 1.5

    if goal.traj_max_angaccel <= 0:
      goal.traj_max_angaccel = 0.5
    elif goal.traj_max_angaccel > 1:
      goal.traj_max_angaccel = 1    

    if goal.traj_type == goal.HARMONIC:

      goal_time = max(pow(2*pi*h/goal.traj_max_accel,.5),pow(2*pi*h_rot/goal.traj_max_angaccel,.5))

      c_max = floor(goal_time/dt) + 1
      goal_time = c_max*dt

      self.traj_feedback.goal_time = goal_time
      self.traj_feedback.max_vel.x = 2*delta_target.pose.position.x/goal_time
      self.traj_feedback.max_vel.y = 2*delta_target.pose.position.y/goal_time
      self.traj_feedback.max_vel.z = 2*delta_target.pose.position.z/goal_time
      self.traj_feedback.max_accel.x = 2*pi*delta_target.pose.position.x/pow(goal_time,2)
      self.traj_feedback.max_accel.y = 2*pi*delta_target.pose.position.y/pow(goal_time,2)
      self.traj_feedback.max_accel.z = 2*pi*delta_target.pose.position.z/pow(goal_time,2)

      for c in range(1,c_max+1):

        if self.traj_as.is_preempt_requested():

          c_max = floor(goal.stop_time/dt) + 1
          goal.stop_time = c_max*dt
          
          stop_twist = Twist()
          for k in range(0,c_max+1):
            stop_twist.linear.x = harmonic_vel(delta_target.pose.position.x,(c+k)*dt,goal_time)*down_scaling_hann(k,c_max)
            stop_twist.linear.y = harmonic_vel(delta_target.pose.position.y,(c+k)*dt,goal_time)*down_scaling_hann(k,c_max)
            stop_twist.linear.z = harmonic_vel(delta_target.pose.position.z,(c+k)*dt,goal_time)*down_scaling_hann(k,c_max)
            self.target_from_twist(stop_twist,dt)
            self.rate.sleep()
    
          self.stop_target()
          self.traj_result.success = False
          self.traj_as.set_preempted()
          break

        self.lock.acquire()

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

        self.lock.release()

        self.traj_feedback.percentage = 100*c/c_max
        self.traj_as.publish_feedback(self.traj_feedback)

        self.rate.sleep()

      if self.traj_result.success:
        self.traj_as.set_succeeded(self.traj_result)

    elif goal.traj_type == goal.POLY345:

      goal_time = max(pow(5.8*h/goal.traj_max_accel,.5),pow(5.8*h_rot/goal.traj_max_angaccel,.5))

      c_max = floor(goal_time/dt) + 1
      goal_time = c_max*dt

      x_345 = poly_345(delta_target.pose.position.x,goal_time,self.target.twist.linear.x,self.target.acceleration.linear.x,goal.traj_target.twist.linear.x,goal.traj_target.acceleration.linear.x)
      y_345 = poly_345(delta_target.pose.position.y,goal_time,self.target.twist.linear.y,self.target.acceleration.linear.y,goal.traj_target.twist.linear.y,goal.traj_target.acceleration.linear.y)
      z_345 = poly_345(delta_target.pose.position.z,goal_time,self.target.twist.linear.z,self.target.acceleration.linear.z,goal.traj_target.twist.linear.z,goal.traj_target.acceleration.linear.z)

      rot_345 = poly_345(delta_angle,goal_time)

      for c in range(0,c_max+1):

        if self.traj_as.is_preempt_requested():

          c_max = floor(goal.stop_time/dt) + 1
          goal.stop_time = c_max*dt
          
          stop_twist = Twist()
          for k in range(1,c_max+1):
            stop_twist.linear.x = poly_vel(x_345,c*dt)*down_scaling_hann(k,c_max)
            stop_twist.linear.y = poly_vel(y_345,c*dt)*down_scaling_hann(k,c_max)
            stop_twist.linear.z = poly_vel(z_345,c*dt)*down_scaling_hann(k,c_max)
            self.target_from_twist(stop_twist,dt)
            self.rate.sleep()
    
          self.stop_target()
          self.traj_result.success = False
          self.traj_as.set_preempted()
          break

        self.lock.acquire()

        self.target.pose.position.x = c_target.pose.position.x + poly_pos(x_345,c*dt)
        self.target.pose.position.y = c_target.pose.position.y + poly_pos(y_345,c*dt)
        self.target.pose.position.z = c_target.pose.position.z + poly_pos(z_345,c*dt)
        self.target.twist.linear.x = poly_vel(x_345,c*dt)
        self.target.twist.linear.y = poly_vel(y_345,c*dt)
        self.target.twist.linear.z = poly_vel(z_345,c*dt)
        self.target.acceleration.linear.x = poly_accel(x_345,c*dt)
        self.target.acceleration.linear.y = poly_accel(y_345,c*dt)
        self.target.acceleration.linear.z = poly_accel(z_345,c*dt)
        self.target.jerk.linear.x = poly_jerk(x_345,c*dt)
        self.target.jerk.linear.y = poly_jerk(y_345,c*dt)
        self.target.jerk.linear.z = poly_jerk(z_345,c*dt)

        angle = poly_pos(rot_345,c*dt)
        self.target.pose.orientation = list_to_quat(
          ts.quaternion_multiply(ts.quaternion_from_matrix(ts.rotation_matrix(angle,delta_axis,delta_point)),quat_to_list(c_target.pose.orientation)))

        self.lock.release()
        
        self.traj_feedback.percentage = 100*c/c_max
        self.traj_as.publish_feedback(self.traj_feedback)

        self.rate.sleep()

      if self.traj_result.success:
        self.traj_as.set_succeeded(self.traj_result)

    elif goal.traj_type == goal.POLY567:

      goal_time = max(pow(5.8*h/goal.traj_max_accel,.5),pow(5.8*h_rot/goal.traj_max_angaccel,.5))

      c_max = floor(goal_time/dt) + 1
      goal_time = c_max*dt

      x_567 = poly_567(delta_target.pose.position.x,goal_time,self.target.twist.linear.x,self.target.acceleration.linear.x,goal.traj_target.twist.linear.x,goal.traj_target.acceleration.linear.x)
      y_567 = poly_567(delta_target.pose.position.y,goal_time,self.target.twist.linear.y,self.target.acceleration.linear.y,goal.traj_target.twist.linear.y,goal.traj_target.acceleration.linear.y)
      z_567 = poly_567(delta_target.pose.position.z,goal_time,self.target.twist.linear.z,self.target.acceleration.linear.z,goal.traj_target.twist.linear.z,goal.traj_target.acceleration.linear.z)

      rot_567 = poly_567(delta_angle,goal_time)

      for c in range(1,c_max+1):

        if self.traj_as.is_preempt_requested():

          c_max = floor(goal.stop_time/dt) + 1
          goal.stop_time = c_max*dt
          
          stop_twist = Twist()
          for k in range(0,c_max+1):
            stop_twist.linear.x = poly_vel(x_567,c*dt)*down_scaling_hann(k,c_max)
            stop_twist.linear.y = poly_vel(y_567,c*dt)*down_scaling_hann(k,c_max)
            stop_twist.linear.z = poly_vel(z_567,c*dt)*down_scaling_hann(k,c_max)
            self.target_from_twist(stop_twist,dt)
            self.rate.sleep()
    
          self.stop_target()
          self.traj_result.success = False
          self.traj_as.set_preempted()
          break
          
        self.lock.acquire()

        self.target.pose.position.x = c_target.pose.position.x + poly_pos(x_567,c*dt)
        self.target.pose.position.y = c_target.pose.position.y + poly_pos(y_567,c*dt)
        self.target.pose.position.z = c_target.pose.position.z + poly_pos(z_567,c*dt)
        self.target.twist.linear.x = poly_vel(x_567,c*dt)
        self.target.twist.linear.y = poly_vel(y_567,c*dt)
        self.target.twist.linear.z = poly_vel(z_567,c*dt)
        self.target.acceleration.linear.x = poly_accel(x_567,c*dt)
        self.target.acceleration.linear.y = poly_accel(y_567,c*dt)
        self.target.acceleration.linear.z = poly_accel(z_567,c*dt)
        self.target.jerk.linear.x = poly_jerk(x_567,c*dt)
        self.target.jerk.linear.y = poly_jerk(y_567,c*dt)
        self.target.jerk.linear.z = poly_jerk(z_567,c*dt)

        angle = poly_pos(rot_567,c*dt)
        self.target.pose.orientation = list_to_quat(
          ts.quaternion_multiply(ts.quaternion_from_matrix(ts.rotation_matrix(angle,delta_axis,delta_point)),quat_to_list(c_target.pose.orientation)))

        self.lock.release()

        self.traj_feedback.percentage = 100*c/c_max
        self.traj_as.publish_feedback(self.traj_feedback)

        self.rate.sleep()

      if self.traj_result.success:
        self.traj_as.set_succeeded(self.traj_result)

    elif goal.traj_type == goal.TRAPZMOD:

      if goal.trapz_max_vel <= 0:
        goal.trapz_max_vel = 0.5
      elif goal.trapz_max_vel > 1.5:
        goal.trapz_max_vel = 1.5
      
      if goal.trapz_max_angvel <= 0:
        goal.trapz_max_angvel = 0.5
      elif goal.trapz_max_angvel > 1:
        goal.trapz_max_angvel = 1 
      
      if goal.trapz_alpha <= 0:
        goal.trapz_alpha = 0.2
      elif goal.trapz_alpha > 0.5:
        goal.trapz_alpha = 0.5 

      max_vel = goal.trapz_max_vel
      max_angvel = goal.trapz_max_angvel
      while True:
        c_v = abs(1/(goal.trapz_alpha-1))
        goal_time = max(h*c_v/max_vel,h_rot*c_v/max_angvel)
        c_a = abs(2/(goal.trapz_alpha*(goal.trapz_alpha-1)))
        max_accel = c_a*h/pow(goal_time,2)
        max_angaccel = c_a*h_rot/pow(goal_time,2)
        if max_accel <= goal.traj_max_accel and max_angaccel <= goal.traj_max_angaccel:
          break
        max_vel = max_vel*0.99
        max_angvel = max_angvel*0.99

      c_max = floor(goal_time/dt) + 1
      goal_time = c_max*dt

      for c in range(1,c_max+1):

        if self.traj_as.is_preempt_requested():

          c_max = floor(goal.stop_time/dt) + 1
          goal.stop_time = c_max*dt
          
          stop_twist = Twist()
          for k in range(1,c_max+1):
            stop_twist.linear.x = trapz_mod_vel(delta_target.pose.position.x,goal_time,goal.trapz_alpha,c*dt)*down_scaling_hann(k,c_max)
            stop_twist.linear.y = trapz_mod_vel(delta_target.pose.position.y,goal_time,goal.trapz_alpha,c*dt)*down_scaling_hann(k,c_max)
            stop_twist.linear.z = trapz_mod_vel(delta_target.pose.position.z,goal_time,goal.trapz_alpha,c*dt)*down_scaling_hann(k,c_max)
            self.target_from_twist(stop_twist,dt)
            self.rate.sleep()
    
          self.stop_target()
          self.traj_result.success = False
          self.traj_as.set_preempted()
          break

        self.lock.acquire()

        self.target.pose.position.x = c_target.pose.position.x + trapz_mod_pos(delta_target.pose.position.x,goal_time,goal.trapz_alpha,c*dt)
        self.target.pose.position.y = c_target.pose.position.y + trapz_mod_pos(delta_target.pose.position.y,goal_time,goal.trapz_alpha,c*dt)
        self.target.pose.position.z = c_target.pose.position.z + trapz_mod_pos(delta_target.pose.position.z,goal_time,goal.trapz_alpha,c*dt)
        self.target.twist.linear.x = trapz_mod_vel(delta_target.pose.position.x,goal_time,goal.trapz_alpha,c*dt)
        self.target.twist.linear.y = trapz_mod_vel(delta_target.pose.position.y,goal_time,goal.trapz_alpha,c*dt)
        self.target.twist.linear.z = trapz_mod_vel(delta_target.pose.position.z,goal_time,goal.trapz_alpha,c*dt)
        self.target.acceleration.linear.x = trapz_mod_accel(delta_target.pose.position.x,goal_time,goal.trapz_alpha,c*dt)
        self.target.acceleration.linear.y = trapz_mod_accel(delta_target.pose.position.y,goal_time,goal.trapz_alpha,c*dt)
        self.target.acceleration.linear.z = trapz_mod_accel(delta_target.pose.position.z,goal_time,goal.trapz_alpha,c*dt)
        
        angle = trapz_mod_pos(delta_angle,goal_time,goal.trapz_alpha,c*dt)
        self.target.pose.orientation = list_to_quat(
          ts.quaternion_multiply(ts.quaternion_from_matrix(ts.rotation_matrix(angle,delta_axis,delta_point)),quat_to_list(c_target.pose.orientation)))
        
        self.lock.release()

        self.traj_feedback.percentage = 100*c/c_max
        self.traj_as.publish_feedback(self.traj_feedback)

        self.rate.sleep()

      if self.traj_result.success:
        self.traj_as.set_succeeded(self.traj_result)
  
  def update(self,event):
    msg = CartesianTrajectoryPointStamped()
    msg.header.stamp = rospy.Time.now()
    self.lock.acquire()
    msg.point = self.target
    self.traj_pnt_pub.publish(msg)
    self.lock.release()

  def target_from_twist(self, target_twist : Twist, dt : float):
    self.lock.acquire()
    self.target.pose.position.x = self.target.pose.position.x + target_twist.linear.x*dt
    self.target.pose.position.y = self.target.pose.position.y + target_twist.linear.y*dt
    self.target.pose.position.z = self.target.pose.position.z + target_twist.linear.z*dt
    self.target.acceleration.linear.x = (target_twist.linear.x - self.target.twist.linear.x)/dt
    self.target.acceleration.linear.y = (target_twist.linear.y - self.target.twist.linear.y)/dt
    self.target.acceleration.linear.z = (target_twist.linear.z - self.target.twist.linear.z)/dt
    self.target.twist.linear.x = target_twist.linear.x
    self.target.twist.linear.y = target_twist.linear.y
    self.target.twist.linear.z = target_twist.linear.z
    self.lock.release()

  def stop_target(self):
    self.lock.acquire()
    self.target.twist.linear.x = 0
    self.target.twist.linear.y = 0
    self.target.twist.linear.z = 0
    self.target.acceleration.linear.x = 0
    self.target.acceleration.linear.y = 0
    self.target.acceleration.linear.z = 0
    self.target.jerk.linear.x = 0
    self.target.jerk.linear.y = 0
    self.target.jerk.linear.z = 0
    self.lock.release()

  def copy_plugin_target(self, plugin_target : CartesianTrajectoryPoint):
    self.plugin_running = True
    while self.plugin_running:
      self.lock.acquire()
      self.target.pose = plugin_target.pose
      self.target.twist = plugin_target.twist
      self.target.acceleration = plugin_target.acceleration
      self.lock.release()
      self.rate.sleep()
  
  def stop_plugin_target(self,stop_time,dt):
    c_max = floor(stop_time/dt) + 1
    stop_time = c_max*dt

    self.plugin_running = False
    self.plugin_thread.join()

    stop_twist = Twist()
    for k in range(0,c_max+1):
      self.lock.acquire()
      stop_twist.linear.x = self.dmp_plugin.plugin_target.twist.linear.x*down_scaling_hann(k,c_max)
      stop_twist.linear.y = self.dmp_plugin.plugin_target.twist.linear.y*down_scaling_hann(k,c_max)
      stop_twist.linear.z = self.dmp_plugin.plugin_target.twist.linear.z*down_scaling_hann(k,c_max)
      self.target_from_twist(stop_twist,dt)
      self.lock.release()
      self.rate.sleep()

    self.stop_target()      

def main():

  rospy.init_node('cartesian_trajectory_generator_node')

  cart_traj_gen = CartesianTrajectoryGenerator()

  rospy.loginfo('Cartesian trajectory generator ready!')

  rospy.spin()

if __name__ == '__main__':
  main()