import sys
from functools import partial
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from sympy import rad

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
    ind_label,self.ind_spin_box = new_param_item('ind',-1.4,1.4,0.05,False)
    mrl_label,self.mrl_spin_box = new_param_item('mrl',0,1.4,0.05,False)
    thu_label,self.thu_spin_box = new_param_item('thu',0,1.4,0.05,False)
    time_label,self.time_spin_box = new_param_item('time',0,2.0,0.2,False)

    rad_label = QLabel("[rad]")
    rad_label.setAlignment(Qt.AlignCenter | Qt.AlignLeft)
    sec_label = QLabel("[s]")
    sec_label.setAlignment(Qt.AlignCenter | Qt.AlignLeft)

    spin_box_layout = QHBoxLayout()
    spin_box_layout.addWidget(ind_label)
    spin_box_layout.addWidget(self.ind_spin_box)
    spin_box_layout.addWidget(rad_label)
    spin_box_layout.addWidget(mrl_label)
    spin_box_layout.addWidget(self.mrl_spin_box)
    spin_box_layout.addWidget(rad_label)
    spin_box_layout.addWidget(thu_label)
    spin_box_layout.addWidget(self.thu_spin_box)
    spin_box_layout.addWidget(rad_label)
    spin_box_layout.addWidget(time_label)
    spin_box_layout.addWidget(self.time_spin_box)
    spin_box_layout.addWidget(sec_label)

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

    self.grasp_group_box = GraspGroupBox(title="Grasp")
    self.grasp_group_box.ind_spin_box.setValue(0.60)
    self.grasp_group_box.mrl_spin_box.setValue(1.00)
    self.grasp_group_box.thu_spin_box.setValue(1.00)
    self.grasp_group_box.time_spin_box.setValue(1.0)

    self.close_push_button = QPushButton("Close")
    self.stop_push_button = QPushButton("Stop")
    self.open_push_button = QPushButton("Open")
    
    self.open_vel_label = QLabel("[rad/s]")
    self.open_vel_spin_box = QDoubleSpinBox()
    self.open_vel_spin_box.setValue(1.5)
    self.open_vel_spin_box.setRange(0.25,3.0)
    self.open_vel_spin_box.setSingleStep(0.25)

    self.close_push_button.clicked.connect(self.set_grasp_joint_positions)
    self.stop_push_button.clicked.connect(self.stop_joint_positions)
    self.open_push_button.clicked.connect(self.zero_joint_positions)
    self.grasp_group_box.clicked.connect(partial(self.joint_group_box.setChecked, False))
    self.joint_group_box.clicked.connect(partial(self.grasp_group_box.setChecked, False))

    self.buttons_layout = QHBoxLayout()
    self.buttons_layout.addWidget(self.close_push_button)
    self.buttons_layout.addWidget(self.stop_push_button)
    self.buttons_layout.addWidget(self.open_push_button)
    self.buttons_layout.addWidget(self.open_vel_spin_box)
    self.buttons_layout.addWidget(self.open_vel_label)

    main_layout = QVBoxLayout()
    main_layout.addWidget(self.joint_group_box)
    main_layout.addWidget(self.grasp_group_box)
    main_layout.addLayout(self.buttons_layout)

    self.setLayout(main_layout)
  
  def stop_joint_positions(self):
    self.joint_group_box.setChecked(False)
    self.hand.stop()
  
  def zero_joint_positions(self):
    self.joint_group_box.setChecked(False)
    self.hand.open(self.open_vel_spin_box.value())
    self.refresh_joint_positions()

  def refresh_joint_positions(self):
    if self.joint_group_box.isChecked():  
      self.joint_group_box.i_spin_box.setValue(self.hand.j_pos[0])
      self.joint_group_box.m_spin_box.setValue(self.hand.j_pos[1])
      self.joint_group_box.t_spin_box.setValue(self.hand.j_pos[2])
  
  def set_grasp_joint_positions(self):
    if self.grasp_group_box.isChecked(): 
      if self.grasp_group_box.time_spin_box.value() > 0.5:
        self.hand.set_joint_target_positions([self.grasp_group_box.ind_spin_box.value(),self.grasp_group_box.mrl_spin_box.value(),self.grasp_group_box.thu_spin_box.value()],self.grasp_group_box.time_spin_box.value())
      else:
        self.hand.set_joint_positions([self.grasp_group_box.ind_spin_box.value(),self.grasp_group_box.mrl_spin_box.value(),self.grasp_group_box.thu_spin_box.value()])
       
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