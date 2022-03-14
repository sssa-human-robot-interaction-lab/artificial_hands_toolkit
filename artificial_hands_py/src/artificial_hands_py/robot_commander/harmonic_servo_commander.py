from time import sleep
from artificial_hands_py.robot_commander import *

class HarmonicServoCommander(ServoCommanderBase):
  """ Purpose of this class is giving access to cartesian planning in the eef frame.
  Using a moveit servo_server node it is possible to move the eef following cycloidal
  or biharmonic motion-laws

  Inherits
  --------
  artificial_hands_py.controller_manager_base.ControllerManagerBase
  
  moveit_commander.MoveGroupCommander

  Attributes
  ----------
  j_traj_ctlr : str
    name of ROS joint controller for trajectory planning and excution
  j_servo_ctrl : str
    name of ROS joint controller to feedforward output from moveit servo_server
  c_twist_servo : str
    name of ROS topic to publish cartesian twist command

  Methods
  -------
  servo_delta(goal_time : float, delta_pose : Pose = Pose(), delta_pose_2 : Pose = Pose()):
    move eef from the actual position/orientation by delta_pose, set max allowed displacement in delta_pose_2

  """

  j_traj_ctrl = 'pos_joint_traj_controller'
  j_servo_ctrl = 'joint_group_vel_controller'
  c_twist_servo = 'servo_twist_controller'

  def __init__(self, ns: str, ref: str, eef: str, ctrl_dict: dict = None, servo_dict: dict = None, move_group: str = 'manipulator') -> None:
    if ctrl_dict is None:
      ctrl_dict = {self.j_traj_ctrl : JointTrajectory, self.j_servo_ctrl : Float64MultiArray}
    if servo_dict is None:
      servo_dict = {self.c_twist_servo : TwistStamped}
    super().__init__(ns, ref, eef, ctrl_dict, servo_dict, move_group)

  def servo_delta(self, goal_time : float, delta_pose : Pose = Pose(), delta_pose_2 : Pose = Pose()):

    def harmonic_vel(h : float, ts : float):
      t = np.arange(0,ts,self.dt)
      y = h/ts*(1 - np.cos(2*pi*t/ts))
      return y

    def biharmonic_vel_2(h : float, ts : float):
        t = np.arange(0,ts,self.dt)
        y = h*pi/ts*(np.sin(2*pi*t/ts) - 1/2*np.sin(4*pi*t/ts))
        return y
    
    self.switch_to_controller(self.j_servo_ctrl)

    servo_vx = np.ndarray.tolist(harmonic_vel(delta_pose.position.x,goal_time) + biharmonic_vel_2(delta_pose_2.position.x,goal_time))
    servo_vy = np.ndarray.tolist(harmonic_vel(delta_pose.position.y,goal_time) + biharmonic_vel_2(delta_pose_2.position.y,goal_time))
    servo_vz = np.ndarray.tolist(harmonic_vel(delta_pose.position.z,goal_time) + biharmonic_vel_2(delta_pose_2.position.z,goal_time))
    servo_wx = np.ndarray.tolist(harmonic_vel(delta_pose.orientation.x,goal_time) + biharmonic_vel_2(delta_pose_2.orientation.x,goal_time))
    servo_wy = np.ndarray.tolist(harmonic_vel(delta_pose.orientation.y,goal_time) + biharmonic_vel_2(delta_pose_2.orientation.y,goal_time))
    servo_wz = np.ndarray.tolist(harmonic_vel(delta_pose.orientation.z,goal_time) + biharmonic_vel_2(delta_pose_2.orientation.z,goal_time))

    servo_cmd = TwistStamped()
    for c in range(0,len(servo_vx)):
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