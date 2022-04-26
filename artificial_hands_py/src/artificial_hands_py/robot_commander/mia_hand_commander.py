from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray

from artificial_hands_py.robot_commander.controller_manager_base import *

class MiaHandCommander(ControllerManagerBase):
  """ Simple commander for Mia hand: grasps using ROS control

  Inherits
  --------
  artificial_hands_py.controller_manager_base.ControllerManagerBase

  Attributes
  ----------
  traj_ctrl : str
    name of controller used for trajectory control
  vel_ctrl : dict
    name of controller used for velocity control
  joint_namse : list
    list of joint names
  pos : list
    list of current joint angles

  Methods
  -------
  joint_states_callback(msg : JointState)
    callback to update current joint angles
  
  close_cyl(autoswitch : bool, rest: bool, timeout : float)
    close mia hand in cylindrical grasp

  open(autoswitch : bool, vel : float)
    open mia hand with desired velocity
  
  rest(autoswitch : bool)
    stop mia hand motors to the current position
  """
  joint_names = ['j_index_fle','j_mrl_fle','j_thumb_fle']  

  def __init__(self,ns='',ctrl_dict : dict = {}):
    super().__init__(ns,ctrl_dict)
    sub = rospy.Subscriber("mia_hand_joint_states",JointState,self.joint_states_callback)
    self.pos = [.0,.0,.0]
  
  def joint_states_callback(self,msg : JointState):
    robot_joints = msg.name
    c = 0
    for j in self.joint_names:
      self.pos[c] = msg.position[robot_joints.index(j)]    
      c += 1                              

  def close_cylindrical_grasp(self,autoswitch=True,rest=True,timeout=1.5):
    if autoswitch:
      self.switch_to_controller(self.traj_ctrl)
    cmd = JointTrajectory()
    cmd.joint_names = self.joint_names
    cmd.points = [JointTrajectoryPoint()]
    cmd.points[0].positions = [1.3,1.3,0.3]
    cmd.points[0].velocities = [.0,.0,.0]
    cmd.points[0].time_from_start = rospy.Time.from_sec(1.5)
    self.controller_command(cmd)
    if rest:
      rospy.sleep(rospy.Duration().from_sec(timeout))
      self.rest(True)
  
  def open(self,autoswitch=True,vel=.5):
    if autoswitch:
      self.switch_to_controller(self.vel_ctrl)
    j_vel = Float64MultiArray()
    j_vel.data = [-vel,-vel,-vel/2]
    j_is_open = [False,False,False]
    j_limit_open = [.4,.34,.1]
    rate = rospy.Rate(50)
    start = rospy.Time.now()
    while True and (rospy.Time.now() - start).to_sec() < 10:
      for c in range(0,3):
        j_is_open[c] = self.pos[c] < j_limit_open[c]
        if j_is_open[c]:
          j_vel.data[c] = .0
      self.controller_command(j_vel) 
      rate.sleep()
      if all(j_is_open):
        break     
      
  def rest(self,autoswitch=True):
    if autoswitch:
      self.switch_to_controller(self.vel_ctrl)
    j_vel = Float64MultiArray()
    j_vel.data = [.0,.0,.0]
    self.controller_command(j_vel)