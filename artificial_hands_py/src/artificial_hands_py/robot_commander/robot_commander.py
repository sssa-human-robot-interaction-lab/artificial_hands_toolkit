from abc import ABC

from std_msgs.msg import Float64MultiArray, Float64
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, MultiDOFJointTrajectory

from artificial_hands_msgs.msg import *
from artificial_hands_py.cartesian_trajectory_generator.cartesian_publishers import CartesianMDOFPointPublisher, PoseStampedPublisher
from artificial_hands_py.robot_commander.arm_commander import ArmCommander
from artificial_hands_py.robot_commander.mia_hand_commander import MiaHandCommander
from artificial_hands_py.robot_commander.wrist_dynamics_base import WristDynamics 

class RobotCommander(ABC):

  def __init__(self) -> None:

    #TO DO Make use of yaml or args to load required controllers

    j_traj_pos_ctrl = 'scaled_pos_joint_traj_controller'
    cart_mot_pos_ctrl = 'cartesian_motion_position_controller'
    cart_mot_vel_ctrl = 'cartesian_motion_position_controller'
    cart_eik_pos_ctrl = 'cartesian_eik_position_controller'
    cart_eik_vel_ctrl = 'cartesian_eik_position_controller'

    arm_ctrl_dict = {j_traj_pos_ctrl : JointTrajectory,
                cart_mot_pos_ctrl : PoseStamped,
                cart_mot_vel_ctrl : PoseStamped,
                cart_eik_pos_ctrl : MultiDOFJointTrajectory,
                cart_eik_vel_ctrl : MultiDOFJointTrajectory}

    #TO DO Move this publishers within the ArmCommander class
    
    cart_mot_pos_pub = PoseStampedPublisher(cart_mot_pos_ctrl+'/command')
    cart_mot_vel_pub = PoseStampedPublisher(cart_mot_vel_ctrl+'/command')
    cart_eik_pos_pub = CartesianMDOFPointPublisher(cart_eik_pos_ctrl+'/command')
    cart_eik_vel_pub = CartesianMDOFPointPublisher(cart_eik_vel_ctrl+'/command')

    self.arm = ArmCommander(ctrl_dict=arm_ctrl_dict) 

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

    hand = MiaHandCommander(ns='mia_hand',ctrl_dict=mia_ctrl_dict)

    self.hand = MiaHandCommander(ns='mia_hand',ctrl_dict=mia_ctrl_dict)

    self.wrist_dyn = WristDynamics()
