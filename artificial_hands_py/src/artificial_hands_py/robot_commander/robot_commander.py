from abc import ABC

from std_msgs.msg import Float64MultiArray, Float64


from artificial_hands_msgs.msg import *

from artificial_hands_py.robot_commander.arm_commander import ArmCommander
from artificial_hands_py.robot_commander.mia_hand_commander import MiaHandCommander
from artificial_hands_py.robot_commander.wrist_dynamics_base import WristDynamics 

class RobotCommander(ABC):

  def __init__(self) -> None:

    self.arm = ArmCommander() 

    self.hand = MiaHandCommander(ns='mia_hand')

    self.wrist_dyn = WristDynamics()
