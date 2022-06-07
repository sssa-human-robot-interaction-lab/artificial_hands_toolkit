from threading import Thread, Lock

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64MultiArray, Float64

from artificial_hands_py.artificial_hands_py_base import singleton
from artificial_hands_py.robot_commander.controller_manager_base import *

@singleton
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

  lock = Lock()

  joint_names = ['j_index_fle','j_mrl_fle','j_thumb_fle']  

  mia_j_traj_ctrl = 'mia_hand_hw_vel_trajectory_controller'
  mia_j_vel_ctrl = 'mia_hand_joint_group_vel_controller'
  mia_j_index_pos_vel_ctrl = 'mia_hand_j_index_fle_pos_vel_controller'
  mia_j_mrl_pos_vel_ctrl = 'mia_hand_j_mrl_fle_pos_vel_controller'
  mia_j_thumb_pos_vel_ctrl = 'mia_hand_j_thumb_fle_pos_vel_controller'

  mia_ctrl_dict = {mia_j_traj_ctrl : JointTrajectory,
              mia_j_vel_ctrl : Float64MultiArray,
              mia_j_index_pos_vel_ctrl : Float64,
              mia_j_mrl_pos_vel_ctrl : Float64,
              mia_j_thumb_pos_vel_ctrl : Float64}

  def __init__(self,ns=''):
    super().__init__(ns,self.ctrl_dict)

    self.rate = rospy.Rate(20)

    self.j_pos = [.0,.0,.0]

    j_sub = rospy.Subscriber(ns+"/joint_states",JointState,self.joint_states_callback)
    j_pos_sub = rospy.Subscriber(ns+"/mia_hand_joint_group_pos_vel_controller/command",Float64MultiArray,self.j_pos_vel_callback)

    self.switch_to_pos_vel_controllers()

    self.j_index_pos = Float64()
    self.j_mrl_pos = Float64()
    self.j_thumb_pos = Float64()

    self.j_index_pos.data = 0.4

    j_index_pos_thread = Thread(target=self.j_index_pos_vel_update)
    j_mrl_pos_thread = Thread(target=self.j_mrl_pos_vel_update)
    j_thumb_pos_thread = Thread(target=self.j_thumb_pos_vel_update)

    j_index_pos_thread.start()
    j_mrl_pos_thread.start()
    j_thumb_pos_thread.start()
  
  def set_joint_positions(self, j_pos : list):
    self.lock.acquire()
    self.j_index_pos = j_pos[0]
    self.j_mrl_pos = j_pos[1]
    self.j_thumb_pos = j_pos[2]
    self.lock.release()
  
  def switch_to_pos_vel_controllers(self):
    self.start_controllers([self.mia_j_index_pos_vel_ctrl,
                            self.mia_j_mrl_pos_vel_ctrl,
                            self.mia_j_thumb_pos_vel_ctrl])

  def j_index_pos_vel_update(self):
    while not rospy.is_shutdown():
      self.controller_command(self.j_index_pos,ctrl=self.mia_j_index_pos_vel_ctrl)
      self.rate.sleep()
  
  def j_mrl_pos_vel_update(self):
    while not rospy.is_shutdown():
      self.controller_command(self.j_mrl_pos,ctrl=self.mia_j_mrl_pos_vel_ctrl)
      self.rate.sleep()

  def j_thumb_pos_vel_update(self):
    while not rospy.is_shutdown():
      self.controller_command(self.j_thumb_pos,ctrl=self.mia_j_thumb_pos_vel_ctrl)
      self.rate.sleep()                          
      
  def stop(self,autoswitch=True):
    self.switch_to_controller(self.mia_j_vel_ctrl)
    j_vel = Float64MultiArray()
    j_vel.data = [.0,.0,.0]
    self.controller_command(j_vel)
  
  def joint_states_callback(self,msg : JointState):
    robot_joints = msg.name
    c = 0
    for j in self.joint_names:
      self.j_pos[c] = msg.position[robot_joints.index(j)]    
      c += 1  

  def j_pos_vel_callback(self,msg : Float64MultiArray):
    self.lock.acquire()
    self.j_index_pos = msg.data[0]
    self.j_mrl_pos = msg.data[1]
    self.j_thumb_pos = msg.data[2]
    self.lock.release()

def main():

  rospy.init_node('mia_hand_commander_node')

  hand = MiaHandCommander(ns='mia_hand')

  rospy.loginfo('Mia hand commander ready!')

  rospy.spin()

if __name__ == '__main__':
  main()