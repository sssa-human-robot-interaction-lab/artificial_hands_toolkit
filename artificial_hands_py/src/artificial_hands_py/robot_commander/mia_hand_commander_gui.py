import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt

import rospy

from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

from artificial_hands_py.cartesian_trajectory_generator.cartesian_trajectory_generator_gui import new_param_item
from artificial_hands_py.robot_commander.mia_hand_commander import *

class TeleopGroupBox(QGroupBox):

  def __init__(self,parent : QWidget = None, title : str = None):
    super().__init__()
    i_label,self.i_spin_box,self.i_sld = new_param_item('ind',-1.4,1.4,0.01,True)
    m_label,self.m_spin_box,self.m_sld = new_param_item('mrl',0,1.4,0.01,True)
    t_label,self.t_spin_box,self.t_sld = new_param_item('thu',0,1.4,0.01,True)

    spin_box_layout = QHBoxLayout()
    spin_box_layout.addWidget(i_label)
    spin_box_layout.addWidget(self.i_spin_box)
    spin_box_layout.addWidget(m_label)
    spin_box_layout.addWidget(self.m_spin_box)
    spin_box_layout.addWidget(t_label)
    spin_box_layout.addWidget(self.t_spin_box)

    sld_layout = QVBoxLayout()
    sld_layout.addWidget(self.i_sld)
    sld_layout.addWidget(self.m_sld)
    sld_layout.addWidget(self.t_sld)

    target_layout = QVBoxLayout()
    target_layout.addLayout(spin_box_layout)
    target_layout.addLayout(sld_layout)

    if title is not None:
      self.setTitle(title)
    self.setLayout(target_layout)

    self.setCheckable(True)
    self.setChecked(False)

class GraspGroupBox(QGroupBox):

  def __init__(self,parent : QWidget = None, title : str = None):
    super().__init__()
    j_label,self.j_spin_box = new_param_item('ind',-1.4,1.4,0.01,False)
    m_label,self.m_spin_box = new_param_item('thu',0,1.4,0.01,False)
    t_label,self.t_spin_box = new_param_item('mrl',0,1.4,0.01,False)

    spin_box_layout = QHBoxLayout()
    spin_box_layout.addWidget(j_label)
    spin_box_layout.addWidget(self.j_spin_box)
    spin_box_layout.addWidget(m_label)
    spin_box_layout.addWidget(self.m_spin_box)
    spin_box_layout.addWidget(t_label)
    spin_box_layout.addWidget(self.t_spin_box)

    target_layout = QVBoxLayout()
    target_layout.addLayout(spin_box_layout)

    if title is not None:
      self.setTitle(title)
    self.setLayout(target_layout)

    self.setCheckable(True)
    self.setChecked(False)

class MiaHandCommanderGui(QWidget):

  def __init__(self, hand : MiaHandCommander) -> None:
    super().__init__()

    self.hand = hand

    self.joint_group_box = TeleopGroupBox(title='Joint command')
    self.joint_group_box.clicked.connect(self.refresh_joint_positions)
    self.joint_group_box.i_spin_box.valueChanged.connect(self.update_joint_positions)
    self.joint_group_box.m_spin_box.valueChanged.connect(self.update_joint_positions)
    self.joint_group_box.t_spin_box.valueChanged.connect(self.update_joint_positions)

    main_layout = QVBoxLayout()
    main_layout.addWidget(self.joint_group_box)

    self.setLayout(main_layout)

  def refresh_joint_positions(self):
    self.joint_group_box.i_spin_box.setValue(self.hand.j_pos[0])
    self.joint_group_box.m_spin_box.setValue(self.hand.j_pos[1])
    self.joint_group_box.t_spin_box.setValue(self.hand.j_pos[2])

  def update_joint_positions(self):
    self.hand.set_joint_positions([self.joint_group_box.i_spin_box.value(),self.joint_group_box.m_spin_box.value(),self.joint_group_box.t_spin_box.value()])    

def main():

  rospy.init_node('mia_hand_commander_gui_node')

  app = QApplication(sys.argv)

  hand = MiaHandCommander(ns='mia_hand')

  hand_cmd_gui = MiaHandCommanderGui(hand)
  hand_cmd_gui.show()

  rospy.loginfo('Mia hand commander GUI ready!')
  
  sys.exit(app.exec_())

if __name__ == '__main__':
  main()