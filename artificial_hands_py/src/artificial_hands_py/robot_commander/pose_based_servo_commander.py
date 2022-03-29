from time import sleep
from artificial_hands_py.robot_commander import *

from cartesian_control_msgs.msg import CartesianTrajectory, CartesianTrajectoryPoint, FollowCartesianTrajectoryActionGoal
from geometry_msgs.msg import Transform, Point

class PoseBasedServoCommander(ServoCommanderBase):

  j_traj_ctrl = 'scaled_pos_joint_traj_controller'
  c_traj_ctrl = 'pose_based_cartesian_traj_controller'

  def __init__(self, ns: str, ref: str, eef: str, ctrl_dict: dict = None, servo_dict: dict = None, move_group: str = 'manipulator') -> None:
    if ctrl_dict is None:
      ctrl_dict = {self.j_traj_ctrl : JointTrajectory, self.c_traj_ctrl : FollowCartesianTrajectoryActionGoal}
    super().__init__(ns, ref, eef, ctrl_dict, servo_dict, move_group)

  def servo_delta(self, goal_time : float, goal_delta : Point = Point(), goal_delta_2 : Point = Point(), forward_command : bool = False) -> CartesianTrajectory:

    def harmonic_pos(h : float, ts : float):
      t = np.arange(0,ts,self.dt)
      y = h*(t/ts - 1/(2*pi)*np.sin(2*pi*t/ts))
      return y

    def harmonic_vel(h : float, ts : float):
      t = np.arange(0,ts,self.dt)
      y = h/ts*(1 - np.cos(2*pi*t/ts))
      return y

    def biharmonic_pos_2(h : float, ts : float):
      t = np.arange(0,ts,self.dt)
      y = h/2*((1-np.cos(2*pi*t/ts))-1/4*(1-np.cos(4*pi*t/ts)))
      return y
    
    def biharmonic_vel_2(h : float, ts : float):
      t = np.arange(0,ts,self.dt)
      y = h*pi/ts*(np.sin(2*pi*t/ts) - 1/2*np.sin(4*pi*t/ts))
      return y
    
    servo_x = np.ndarray.tolist(harmonic_pos(goal_delta.x,goal_time)+biharmonic_pos_2(goal_delta_2.x,goal_time))
    servo_y = np.ndarray.tolist(harmonic_pos(goal_delta.y,goal_time)+biharmonic_pos_2(goal_delta_2.y,goal_time))
    servo_z = np.ndarray.tolist(harmonic_pos(goal_delta.z,goal_time)+biharmonic_pos_2(goal_delta_2.z,goal_time))

    servo_vx = np.ndarray.tolist(harmonic_vel(goal_delta.x,goal_time)+biharmonic_vel_2(goal_delta_2.x,goal_time))
    servo_vy = np.ndarray.tolist(harmonic_vel(goal_delta.y,goal_time)+biharmonic_vel_2(goal_delta_2.y,goal_time))
    servo_vz = np.ndarray.tolist(harmonic_vel(goal_delta.z,goal_time)+biharmonic_vel_2(goal_delta_2.z,goal_time))

    c_pose = self.get_eef_frame()

    traj_cmd = CartesianTrajectory()

    for c in range(0,len(servo_x)):
      cmd = CartesianTrajectoryPoint()
      cmd.pose.position.x = servo_x[c] + c_pose.pose.position.x
      cmd.pose.position.y = servo_y[c] + c_pose.pose.position.y
      cmd.pose.position.z = servo_z[c] + c_pose.pose.position.z
      cmd.twist.linear.x = servo_vx[c]
      cmd.twist.linear.y = servo_vy[c]
      cmd.twist.linear.z = servo_vz[c]
      cmd.time_from_start = rospy.Duration.from_sec(c*self.dt)
      traj_cmd.points.append(cmd)
      
    if forward_command:
      self.switch_to_controller(self.c_traj_ctrl)
      for pnt_cmd in traj_cmd.points:
        if not self.paused:
          self.controller_command(pnt_cmd)
        self.rate.sleep()
      sleep(1)
      self.switch_to_controller(self.j_traj_ctrl)

    return traj_cmd   
  
  def servo_goal(self, goal_time : float, goal_pose : PoseStamped = PoseStamped(), auto_switch : bool = True):

    def harmonic_pos(h : float, ts : float):
      t = np.arange(0,ts,self.dt)
      y = h*(t/ts - 1/(2*pi)*np.sin(2*pi*t/ts))
      return y

    c_pose = self.get_eef_frame()
    delta = Point()
    delta.x = goal_pose.pose.position.x - c_pose.pose.position.x
    delta.y = goal_pose.pose.position.y - c_pose.pose.position.y
    delta.z = goal_pose.pose.position.z - c_pose.pose.position.z

    traj_cmd = self.servo_delta(goal_time,delta)

    base_to_eef = self.tf_buffer.lookup_transform(self.ee_frame,'base',rospy.Time(0),timeout=rospy.Duration(1))
    eef_to_base = self.tf_buffer.lookup_transform('base',self.ee_frame,rospy.Time(0),timeout=rospy.Duration(1))
    ee_delta = tf2_geometry_msgs.do_transform_pose(goal_pose,base_to_eef)

    ee_rot = ts.quaternion_matrix(quat_to_list(ee_delta.pose.orientation))
    ee_delta_angle,ee_delta_axis,ee_delta_point = ts.rotation_from_matrix(ee_rot)

    servo_angle = harmonic_pos(ee_delta_angle,goal_time)

    c = 0
    for angle in servo_angle:
      angle_pose = PoseStamped()
      angle_pose.header.frame_id = self.ee_frame
      angle_pose.pose.orientation = list_to_quat(ts.quaternion_from_matrix(ts.rotation_matrix(angle,ee_delta_axis,ee_delta_point)))
      angle_pose = tf2_geometry_msgs.do_transform_pose(angle_pose,eef_to_base)
      traj_cmd.points[c].pose.orientation = angle_pose.pose.orientation
      c += 1

    if auto_switch:
      self.switch_to_controller(self.c_traj_ctrl)

    cmd = FollowCartesianTrajectoryActionGoal()
    cmd.goal.trajectory.points = traj_cmd.points[1:-1]
    self.controller_command(cmd)
    sleep(goal_time)

    if auto_switch:
      sleep(1)
      self.switch_to_controller(self.j_traj_ctrl)   