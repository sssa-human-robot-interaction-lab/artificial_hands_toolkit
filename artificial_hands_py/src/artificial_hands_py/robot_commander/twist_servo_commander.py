from time import sleep
from artificial_hands_py.robot_commander import *

class TwistServoCommander(ServoCommanderBase):

  j_traj_ctrl = 'pos_joint_traj_controller'
  j_servo_ctrl = 'joint_group_vel_controller'
  c_twist_servo = 'servo_twist_controller'

  def __init__(self, ns: str, ref: str, eef: str, ctrl_dict: dict = None, servo_dict: dict = None, move_group: str = 'manipulator') -> None:
    if ctrl_dict is None:
      ctrl_dict = {self.j_traj_ctrl : JointTrajectory, self.j_servo_ctrl : Float64MultiArray}
    if servo_dict is None:
      servo_dict = {self.c_twist_servo : TwistStamped}
    super().__init__(ns, ref, eef, ctrl_dict, servo_dict, move_group)

  def servo_delta(self, goal_time : float, type : str = 'harmonic', delta_pos : Point = Point(), delta_ang : Point = Point()):

    def harmonic_delta(goal_time : float, dt : float, delta_pos : Point = Point(), delta_ang : Point = Point()):

      def harmonic_vel(h : float, ts : float):
        t = np.arange(0,ts,dt)
        y = h/ts*(1 - np.cos(2*pi*t/ts))
        return y

      servo_vx = np.ndarray.tolist(harmonic_vel(delta_pos.x,goal_time))
      servo_vy = np.ndarray.tolist(harmonic_vel(delta_pos.y,goal_time))
      servo_vz = np.ndarray.tolist(harmonic_vel(delta_pos.z,goal_time))
      servo_wx = np.ndarray.tolist(harmonic_vel(delta_ang.x,goal_time))
      servo_wy = np.ndarray.tolist(harmonic_vel(delta_ang.y,goal_time))
      servo_wz = np.ndarray.tolist(harmonic_vel(delta_ang.z,goal_time))

      return servo_vx,servo_vy,servo_vz,servo_wx,servo_wy,servo_wz

    def biharmonic_delta_2(goal_time : float, dt : float, delta_pos : Point = Point(), delta_ang : Point = Point()):

      def biharmonic_vel_2(h : float, ts : float):
          t = np.arange(0,ts,dt)
          y = h*pi/ts*(np.sin(2*pi*t/ts) - 1/2*np.sin(4*pi*t/ts))
          return y

      servo_vx = np.ndarray.tolist(biharmonic_vel_2(delta_pos.x,goal_time))
      servo_vy = np.ndarray.tolist(biharmonic_vel_2(delta_pos.y,goal_time))
      servo_vz = np.ndarray.tolist(biharmonic_vel_2(delta_pos.z,goal_time))
      servo_wx = np.ndarray.tolist(biharmonic_vel_2(delta_ang.x,goal_time))
      servo_wy = np.ndarray.tolist(biharmonic_vel_2(delta_ang.y,goal_time))
      servo_wz = np.ndarray.tolist(biharmonic_vel_2(delta_ang.z,goal_time))

      return servo_vx,servo_vy,servo_vz,servo_wx,servo_wy,servo_wz
    
    def trapezoidal_delta_2(goal_time : float, dt : float, delta_pos : Point = Point(), delta_ang : Point = Point()):

      def trapezoidal_vel_2(h : float, ts : float):
          tv = ts/3 # cv 1.5 ca 4.5
          d = ts - 2*tv
          a = h/(tv*(ts-tv))
          
          t = np.arange(0,ts,dt)
          y = np.empty(np.size(t))
          y[0] = 0
          y[-1] = 0
          for c in range(1,np.size(t)-1):
            if t[c] <= tv:
              y[c] = y[c-1] + a * dt
            elif t[c] > tv and t[c] < d + tv:
              y[c] = y[c-1]
            elif t[c] >= d + tv:
              y[c] = y[c-1] - a * dt
          return y

      servo_vx = np.ndarray.tolist(trapezoidal_vel_2(delta_pos.x,goal_time))
      servo_vy = np.ndarray.tolist(trapezoidal_vel_2(delta_pos.y,goal_time))
      servo_vz = np.ndarray.tolist(trapezoidal_vel_2(delta_pos.z,goal_time))
      servo_wx = np.ndarray.tolist(trapezoidal_vel_2(delta_ang.x,goal_time))
      servo_wy = np.ndarray.tolist(trapezoidal_vel_2(delta_ang.y,goal_time))
      servo_wz = np.ndarray.tolist(trapezoidal_vel_2(delta_ang.z,goal_time))

      return servo_vx,servo_vy,servo_vz,servo_wx,servo_wy,servo_wz

    if type == 'harmonic':
      servo_vx,servo_vy,servo_vz,servo_wx,servo_wy,servo_wz = harmonic_delta(goal_time,self.dt,delta_pos,delta_ang)
    elif type == 'biharmonic2':
      servo_vx,servo_vy,servo_vz,servo_wx,servo_wy,servo_wz = biharmonic_delta_2(goal_time,self.dt,delta_pos,delta_ang)
    elif type == 'trapezoidal':
      servo_vx,servo_vy,servo_vz,servo_wx,servo_wy,servo_wz = trapezoidal_delta_2(goal_time,self.dt,delta_pos,delta_ang)
    
    self.switch_to_controller(self.j_servo_ctrl)

    servo_cmd = TwistStamped()
    for c in range(0,len(servo_vx)):
      if not self.paused:
        servo_cmd.twist.linear.x = servo_vx[c]
        servo_cmd.twist.linear.y = servo_vy[c]
        servo_cmd.twist.linear.z = servo_vz[c]
        servo_cmd.twist.angular.x = servo_wx[c]
        servo_cmd.twist.angular.y = servo_wy[c]
        servo_cmd.twist.angular.z = servo_wz[c]
        servo_cmd.header.stamp.secs = rospy.Time.now().secs
        servo_cmd.header.stamp.nsecs = rospy.Time.now().nsecs 

        self.servo_command(self.c_twist_servo,servo_cmd)

      self.rate.sleep()

    sleep(.5)
    
    self.switch_to_controller(self.j_traj_ctrl)