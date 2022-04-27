from abc import ABC

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, MultiDOFJointTrajectory

from artificial_hands_msgs.msg import *
from artificial_hands_py.cartesian_trajectory_generator.cartesian_publishers import CartesianMDOFPointPublisher, PoseStampedPublisher
from artificial_hands_py.robot_commander.arm_commander import ArmCommander
from artificial_hands_py.robot_commander.mia_hand_commander import MiaHandCommander
from artificial_hands_py.robot_commander.wrist_dynamics_base import WristDynamics 

class RobotCommander(ABC):

  def __init__(self) -> None:

    #TO DO Make use of yaml to load controllers

    j_traj_pos_ctrl = 'scaled_pos_joint_traj_controller'
    cart_mot_pos_ctrl = 'cartesian_motion_position_controller'

    arm_ctrl_dict = {j_traj_pos_ctrl : JointTrajectory,
                cart_mot_pos_ctrl : PoseStamped}

    cart_mot_pos_pub = PoseStampedPublisher(cart_mot_pos_ctrl+'/command')

    self.arm = ArmCommander(ctrl_dict=arm_ctrl_dict) 

    mia_j_traj_ctrl = 'mia_hand_vel_trajectory_controller'
    mia_j_vel_ctrl = 'mia_hand_joint_group_vel_controller'
    
    mia_ctrl_dict = {mia_j_traj_ctrl : JointTrajectory,
                mia_j_vel_ctrl : Float64MultiArray}

    # self.hand = MiaHandCommander(ns='mia_hand_sim',ctrl_dict=mia_ctrl_dict)

    self.wrist_dyn = WristDynamics()
