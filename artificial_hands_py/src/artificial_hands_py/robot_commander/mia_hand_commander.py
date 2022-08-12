from math import floor
from threading import Thread, Lock
from time import sleep

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64MultiArray, Float64, Empty

from artificial_hands_py.artificial_hands_py_base import singleton
from artificial_hands_py.robot_commander.controller_manager_base import *
from artificial_hands_py.cartesian_trajectory_generator.cartesian_trajectory_base import harmonic_pos

@singleton
class MiaHandCommander(ControllerManagerBase):

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
    super().__init__(ns,self.mia_ctrl_dict)

    self.rate = rospy.Rate(20)

    self.j_pos = [.0,.0,.0]
    self.j_vel = [.0,.0,.0]
    self.j_acc = [.0,.0,.0]

    j_sub = rospy.Subscriber(ns+"/joint_states",JointState,self.joint_states_callback)
    j_pos_sub = rospy.Subscriber(ns+"/mia_hand_joint_group_pos_vel_controller/command",Float64MultiArray,self.j_pos_vel_callback)
    j_traj_sub = rospy.Subscriber(ns+"/mia_hand_joint_group_traj_controller/command",Float64MultiArray,self.j_traj_callback)
    j_stop_sub = rospy.Subscriber(ns+"/mia_hand_joint_group_stop_controller/command",Empty,self.j_stop_callback)
    j_open_sub = rospy.Subscriber(ns+"/mia_hand_joint_group_open_controller/command",Float64,self.j_open_callback)

    self.j_index_pos = Float64()
    self.j_mrl_pos = Float64()
    self.j_thumb_pos = Float64()

    rospy.wait_for_message(ns+"/joint_states",JointState)

    self.switch_to_pos_vel_controllers()

    j_index_pos_thread = Thread(target=self.j_index_pos_vel_update)
    j_mrl_pos_thread = Thread(target=self.j_mrl_pos_vel_update)
    j_thumb_pos_thread = Thread(target=self.j_thumb_pos_vel_update)

    j_index_pos_thread.start()
    j_mrl_pos_thread.start()
    j_thumb_pos_thread.start()
  
  def set_joint_positions(self, j_pos : list):
    self.lock.acquire()
    self.j_index_pos.data = j_pos[0]
    self.j_mrl_pos.data = j_pos[1]
    self.j_thumb_pos.data = j_pos[2]
    self.lock.release()
  
  def set_joint_target_positions(self, target : list, goal_time : float):

    c_target = self.j_pos.copy()

    delta_target = [y - x for (x,y) in zip(c_target,target)]

    dt = self.rate.sleep_dur.to_sec()

    c_max = floor(goal_time/dt) + 1
    goal_time = c_max*dt

    for c in range(1,c_max+1):

      self.lock.acquire()

      self.j_index_pos.data = c_target[0] + harmonic_pos(delta_target[0],c*dt,goal_time)
      self.j_mrl_pos.data = c_target[1] + harmonic_pos(delta_target[1],c*dt,goal_time)
      self.j_thumb_pos.data = c_target[2] + harmonic_pos(delta_target[2],c*dt,goal_time)

      self.lock.release()

      self.rate.sleep()
  
  def switch_to_pos_vel_controllers(self):
    self.j_index_pos.data = self.j_pos[0]
    self.j_mrl_pos.data = self.j_pos[1]
    self.j_thumb_pos.data = self.j_pos[2]
    self.start_controllers([self.mia_j_index_pos_vel_ctrl,
                            self.mia_j_mrl_pos_vel_ctrl,
                            self.mia_j_thumb_pos_vel_ctrl])

  def j_index_pos_vel_update(self):
    while not rospy.is_shutdown():
      self.lock.acquire()
      self.controller_command(self.j_index_pos,ctrl=self.mia_j_index_pos_vel_ctrl)
      self.lock.release()
      self.rate.sleep()
      
  def j_mrl_pos_vel_update(self):
    while not rospy.is_shutdown():
      self.lock.acquire()
      self.controller_command(self.j_mrl_pos,ctrl=self.mia_j_mrl_pos_vel_ctrl)
      self.lock.release()
      self.rate.sleep()

  def j_thumb_pos_vel_update(self):
    while not rospy.is_shutdown():
      self.lock.acquire()
      self.controller_command(self.j_thumb_pos,ctrl=self.mia_j_thumb_pos_vel_ctrl)
      self.lock.release()
      self.rate.sleep()   
                             
  def stop(self):
    self.lock.acquire()
    self.switch_to_controller(self.mia_j_vel_ctrl)
    j_vel = Float64MultiArray()
    j_vel.data = [.0,.0,.0]
    self.controller_command(j_vel)
    rospy.sleep(rospy.Duration.from_sec(0.5))
    self.switch_to_pos_vel_controllers()
    self.lock.release()
  
  def open(self, vel : float = 0.5, pos : list = [0.1,0.6,0.6]):

    self.lock.acquire()
    self.switch_to_controller(self.mia_j_vel_ctrl)
    j_vel = Float64MultiArray()
    j_vel.data = [-vel,-vel,-vel]
    self.controller_command(j_vel)

    dur_open = [abs(y - x)/vel for (x,y) in zip(self.j_pos,pos)]

    start_open = rospy.Time.now()
    while not all(v == 0 for v in j_vel.data):
      for j_id in range(3):
        if (rospy.Time.now() - start_open).to_sec() > dur_open[j_id]:
          j_vel.data[j_id] = 0
      self.controller_command(j_vel)
      self.rate.sleep()

    rospy.sleep(rospy.Duration.from_sec(0.5))
    self.switch_to_pos_vel_controllers()
    self.lock.release()
  
  def joint_states_callback(self,msg : JointState):
    robot_joints = msg.name
    c = 0
    for j in self.joint_names:
      self.j_pos[c] = msg.position[robot_joints.index(j)] 
      self.j_vel[c] = msg.velocity[robot_joints.index(j)]    
      c += 1  

  def j_pos_vel_callback(self,msg : Float64MultiArray):
    self.lock.acquire()
    self.j_index_pos.data = msg.data[0]
    self.j_mrl_pos.data = msg.data[1]
    self.j_thumb_pos.data = msg.data[2]
    self.lock.release()
  
  def j_traj_callback(self,msg : Float64MultiArray):
    self.set_joint_target_positions(target=msg.data[0:3],goal_time=msg.data[3])
  
  def j_stop_callback(self,msg : Empty):
    self.stop()

  def j_open_callback(self,msg : Float64):
    self.open(vel=msg.data)

def main():

  rospy.init_node('mia_hand_commander_node')

  hand = MiaHandCommander(ns='mia_hand')

  rospy.loginfo('Mia hand commander ready!')

  rospy.spin()

if __name__ == '__main__':
  main()