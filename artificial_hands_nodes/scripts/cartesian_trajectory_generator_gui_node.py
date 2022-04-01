#!/usr/bin/env python3

import sys, rospy
from PyQt5.QtWidgets import QApplication
from artificial_hands_py.cartesian_trajectory_generator import CartesianTrajectoryGeneratorGUI

def main():

  rospy.init_node('cartesian_trajectory_generator_gui_node')

  app = QApplication(sys.argv)

  robot_commander_gui = CartesianTrajectoryGeneratorGUI()
  robot_commander_gui.show()

  sys.exit(app.exec_())

if __name__ == '__main__':
  main()