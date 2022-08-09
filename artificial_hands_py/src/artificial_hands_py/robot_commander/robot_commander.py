from abc import ABC

import rospy
from std_msgs.msg import Float64MultiArray, Float64

from artificial_hands_msgs.msg import *

from artificial_hands_py.robot_commander.arm_commander import ArmCommander
from artificial_hands_py.robot_commander.mia_hand_commander import MiaHandCommander 
class RobotCommander(ABC):

  def __init__(self) -> None:

    self.arm = ArmCommander() 

    self.hand = MiaHandCommander(ns='mia_hand')

class RobotCommanderBase(RobotCommander,ABC):
  
  def __init__(self) -> None:
    super().__init__()

def main():

  rospy.init_node('robot_commander_node')

  robot = RobotCommander()

  rospy.loginfo('Robot commander ready!')

  rospy.spin()

if __name__ == '__main__':
  main()
