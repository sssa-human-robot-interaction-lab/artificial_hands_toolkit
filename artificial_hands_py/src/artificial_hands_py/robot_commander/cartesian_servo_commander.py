from time import sleep
from numpy import c_
from artificial_hands_py.robot_commander import *

from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform

class CartesianServoCommander(ServoCommanderBase):

  j_traj_ctrl = 'pos_joint_traj_controller'
  c_traj_ctrl = 'cartesian_eik_position_controller'

  def __init__(self, ns: str, ref: str, eef: str, ctrl_dict: dict = None, servo_dict: dict = None, move_group: str = 'manipulator') -> None:
    if ctrl_dict is None:
      ctrl_dict = {self.j_traj_ctrl : JointTrajectory, self.c_traj_ctrl : MultiDOFJointTrajectoryPoint}
    super().__init__(ns, ref, eef, ctrl_dict, servo_dict, move_group)

  def servo_delta(self, goal_time : float, goal_pose : Pose = Pose(), goal_pose_2 : Pose = Pose()):

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
    
    self.switch_to_controller(self.c_traj_ctrl)

    c_pose = self.get_eef_frame()

    servo_x = np.ndarray.tolist(harmonic_pos(goal_pose.position.x,goal_time)+biharmonic_pos_2(goal_pose_2.position.x,goal_time))
    servo_y = np.ndarray.tolist(harmonic_pos(goal_pose.position.y,goal_time)+biharmonic_pos_2(goal_pose_2.position.x,goal_time))
    servo_z = np.ndarray.tolist(harmonic_pos(goal_pose.position.z,goal_time)+biharmonic_pos_2(goal_pose_2.position.x,goal_time))
    servo_vx = np.ndarray.tolist(harmonic_vel(goal_pose.position.x,goal_time)+biharmonic_vel_2(goal_pose_2.position.x,goal_time))
    servo_vy = np.ndarray.tolist(harmonic_vel(goal_pose.position.y,goal_time)+biharmonic_vel_2(goal_pose_2.position.y,goal_time))
    servo_vz = np.ndarray.tolist(harmonic_vel(goal_pose.position.z,goal_time)+biharmonic_vel_2(goal_pose_2.position.z,goal_time))

    for c in range(0,len(servo_x)):
      cmd = MultiDOFJointTrajectoryPoint()
      cmd_ = Transform()
      cmd_.translation.x = servo_x[c] + c_pose.pose.position.x
      cmd_.translation.y = servo_y[c] + c_pose.pose.position.y
      cmd_.translation.z = servo_z[c] + c_pose.pose.position.z
      cmd_.rotation = c_pose.pose.orientation
      cmd_dot = Twist()
      cmd_dot.linear.x = servo_vx[c]
      cmd_dot.linear.y = servo_vy[c]
      cmd_dot.linear.z = servo_vz[c]
      cmd.transforms.append(cmd_)
      cmd.velocities.append(cmd_dot)
      if not self.paused:
        self.controller_command(cmd)

      self.rate.sleep()
    
    sleep(.5)
    
    self.switch_to_controller(self.j_traj_ctrl)