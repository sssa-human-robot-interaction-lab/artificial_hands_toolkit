from multiprocessing.managers import BaseManager
from time import sleep
import rospy

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, MultiDOFJointTrajectory

from artificial_hands_msgs.msg import *
from artificial_hands_py.cartesian_trajectory_generator.cartesian_publishers import CartesianMDOFPointPublisher, PoseStampedPublisher
from artificial_hands_py.robot_commander.arm_commander import ArmCommander 
from artificial_hands_py.robot_commander.mia_hand_commander import MiaHandCommander
from artificial_hands_py.robot_commander.wrist_dynamics_base import WristDynamics 

class RobotCommander:

  def __init__(self) -> None:

    #TO DO Make use of yaml to load controllers

    j_traj_pos_ctrl = 'scaled_pos_joint_traj_controller'
    cart_mot_pos_ctrl = 'cartesian_motion_position_controller'

    arm_ctrl_dict = {j_traj_pos_ctrl : JointTrajectory,
                cart_mot_pos_ctrl : PoseStamped}

    cart_mot_pos_pub = PoseStampedPublisher(cart_mot_pos_ctrl+'/command')

    self.arm_ = ArmCommander(ctrl_dict=arm_ctrl_dict) 

    mia_j_traj_ctrl = 'mia_hand_vel_trajectory_controller'
    mia_j_vel_ctrl = 'mia_hand_joint_group_vel_controller'
    
    mia_ctrl_dict = {mia_j_traj_ctrl : JointTrajectory,
                mia_j_vel_ctrl : Float64MultiArray}

    self.hand_ = MiaHandCommander(ns='mia_hand_sim',ctrl_dict=mia_ctrl_dict)

    self.wrist_dyn_ = WristDynamics()
  
  def arm(self) -> ArmCommander:
    return self.arm_

  def hand(self) -> MiaHandCommander:
    return self.hand_

  def wrist_dyn(self) -> WristDynamics:
    return self.wrist_dyn_

class RobotManager(BaseManager): 
  
  def get():
    sleep(1)
    RobotManager.register('get_robot')
    rob_mng = RobotManager(address=('', 50000), authkey=b'robot')
    rob_mng.connect()
    return rob_mng.get_robot()

def main():

  rospy.init_node('robot_commander_node')
 
  robot = RobotCommander()
  
  class RobotManager(BaseManager) : pass
  RobotManager.register('get_robot', callable=lambda:robot, exposed=['arm','hand','wrist_dyn'])
  rob_mng = RobotManager(address=('', 50000), authkey=b'robot')
  rob_mng.start()

  rospy.loginfo('Robot commander ready!')
  
  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    rate.sleep()  
  
  rob_mng.shutdown()

if __name__ == '__main__':
  main()