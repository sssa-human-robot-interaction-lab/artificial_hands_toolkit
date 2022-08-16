import sys
from PyQt5.QtWidgets import *

import rospy

from artificial_hands_py.robot_commander.robot_commander import RobotCommander
from artificial_hands_py.robot_commander.arm_commander_gui import ArmCommanderGui
from artificial_hands_py.robot_commander.mia_hand_commander_gui import MiaHandCommanderGui

class RobotCommanderGui(QTabWidget):

  def __init__(self,robot : RobotCommander) -> None:
    super().__init__()

    self.robot = robot

    self.arm_tab_widget = ArmCommanderGui(self.robot.arm)
    self.hand_tab_widget = MiaHandCommanderGui(self.robot.hand)
    
    self.addTab(self.arm_tab_widget,'Arm')
    self.addTab(self.hand_tab_widget,'Hand')
    
def main():

  rospy.init_node('robot_commander_gui_node')

  app = QApplication(sys.argv)

  robot = RobotCommander()

  robot_cmd_gui = RobotCommanderGui(robot)

  rospy.loginfo('Robot commander GUI ready!')

  robot_cmd_gui.show()
  
  sys.exit(app.exec_())

if __name__ == '__main__':
  main()