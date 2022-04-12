from abc import ABC
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, MultiDOFJointTrajectory

from artificial_hands_msgs.msg import *
from artificial_hands_py.cartesian_trajectory_generator.cartesian_publishers import CartesianMDOFPointPublisher, PoseStampedPublisher
from artificial_hands_py.robot_commander.arm_commander import ArmCommander 

class RobotCommander(ABC):

  def __init__(self) -> None:

    #TO DO Make use of yaml to load controllers

    j_traj_pos_ctrl = 'scaled_pos_joint_traj_controller'
    cart_mot_pos_ctrl = 'cartesian_motion_position_controller'
    # cart_mot_vel_ctrl = 'cartesian_motion_velocity_controller'
    # c_eik_pos_ctrl = 'cartesian_eik_position_controller'
    # c_eik_vel_ctrl= 'cartesian_eik_velocity_controller'

    ctrl_dict = {j_traj_pos_ctrl : JointTrajectory,
                cart_mot_pos_ctrl : PoseStamped}
                # cart_mot_vel_ctrl : PoseStamped,
                # c_eik_pos_ctrl : MultiDOFJointTrajectory,
                # c_eik_vel_ctrl : MultiDOFJointTrajectory}

    cart_mot_pos_pub = PoseStampedPublisher(cart_mot_pos_ctrl+'/command')
    # cart_mot_vel_pub = PoseStampedPublisher(cart_mot_vel_ctrl+'/command')
    # cart_eik_pos_pub = CartesianMDOFPointPublisher(c_eik_pos_ctrl+'/command')
    # cart_eik_vel_pub = CartesianMDOFPointPublisher(c_eik_vel_ctrl+'/command')

    self.arm = ArmCommander(ctrl_dict=ctrl_dict)   

    #TO DO Add Mia Hand commander