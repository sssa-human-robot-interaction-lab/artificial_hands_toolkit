import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt

import rospy

from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

from artificial_hands_py.cartesian_trajectory_generator.cartesian_trajectory_generator_gui import new_param_item
from artificial_hands_py.robot_commander.mia_hand_commander import MiaHandCommander

class GraspGroupBox(QGroupBox):

  def __init__(self,parent : QWidget = None, title : str = None):
    super().__init__()
    j_label,self.j_spin_box = new_param_item('x',-1.4,1.4,0.01,False)
    m_label,self.m_spin_box = new_param_item('y',0,1.4,0.01,False)
    t_label,self.t_spin_box = new_param_item('z',0,1.4,0.01,False)

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

    self.cyl_grasp_group_box = GraspGroupBox(self,'CYL')
    self.cyl_grasp_group_box.j_spin_box.setValue(1.20)
    self.cyl_grasp_group_box.m_spin_box.setValue(1.10)
    self.cyl_grasp_group_box.t_spin_box.setValue(0.60)
    self.cyl_grasp_group_box.setChecked(True)

    self.pin_grasp_group_box = GraspGroupBox(self,'PIN')
    self.pin_grasp_group_box.j_spin_box.setValue(1.22)
    self.pin_grasp_group_box.m_spin_box.setValue(0.40)
    self.pin_grasp_group_box.t_spin_box.setValue(0.60)

    self.tri_grasp_group_box = GraspGroupBox(self,'TRI')
    self.tri_grasp_group_box.j_spin_box.setValue(1.20)
    self.tri_grasp_group_box.m_spin_box.setValue(1.10)
    self.tri_grasp_group_box.t_spin_box.setValue(0.80)

    self.close_push_button = QPushButton('Close')
    self.open_push_button = QPushButton('Open')

    buttons_layout = QHBoxLayout()
    buttons_layout.addWidget(self.close_push_button)
    buttons_layout.addWidget(self.open_push_button)

    close_time_label,self.close_time_spin_box = new_param_item('Close time [s]',0.2,2.0,0.2,False)
    self.close_rest_check_box = QCheckBox('Rest')

    close_p_layout = QHBoxLayout()
    close_p_layout.addWidget(close_time_label)
    close_p_layout.addWidget(self.close_time_spin_box)
    close_p_layout.addWidget(self.close_rest_check_box)

    open_vel_label = QLabel('Opening velocity [%]')
    open_vel_label.setAlignment(Qt.AlignCenter | Qt.AlignRight)

    self.open_vel_slider = QSlider(Qt.Horizontal)
    self.open_vel_slider.setRange(10, 100)
    self.open_vel_slider.setSingleStep(10)
    self.open_vel_slider.setValue(50)

    open_vel_layout = QHBoxLayout()
    open_vel_layout.addWidget(open_vel_label)
    open_vel_layout.addWidget(self.open_vel_slider)

    main_layout = QVBoxLayout()
    main_layout.addWidget(self.cyl_grasp_group_box)
    main_layout.addWidget(self.pin_grasp_group_box)
    main_layout.addWidget(self.tri_grasp_group_box)
    main_layout.addLayout(close_p_layout)
    main_layout.addLayout(open_vel_layout)
    main_layout.addLayout(buttons_layout)

    self.cyl_grasp_group_box.clicked.connect(self.on_cyl_grasp_checked)
    self.pin_grasp_group_box.clicked.connect(self.on_pin_grasp_checked)
    self.tri_grasp_group_box.clicked.connect(self.on_tri_grasp_checked)
    self.close_push_button.clicked.connect(self.on_close_button)
    self.open_push_button.clicked.connect(self.on_open_button)

    self.setLayout(main_layout)    
  
  def on_cyl_grasp_checked(self):
    self.pin_grasp_group_box.setChecked(False)
    self.tri_grasp_group_box.setChecked(False)
  
  def on_pin_grasp_checked(self):
    self.cyl_grasp_group_box.setChecked(False)
    self.tri_grasp_group_box.setChecked(False)
  
  def on_tri_grasp_checked(self):
    self.cyl_grasp_group_box.setChecked(False)
    self.pin_grasp_group_box.setChecked(False)
  
  def on_close_button(self):
    if self.cyl_grasp_group_box.isChecked():
      j = self.cyl_grasp_group_box.j_spin_box.value()
      m = self.cyl_grasp_group_box.m_spin_box.value()
      t = self.cyl_grasp_group_box.t_spin_box.value()
    elif self.pin_grasp_group_box.isChecked():
      j = self.pin_grasp_group_box.j_spin_box.value()
      m = self.pin_grasp_group_box.m_spin_box.value()
      t = self.pin_grasp_group_box.t_spin_box.value()
    elif self.tri_grasp_group_box.isChecked():
      j = self.tri_grasp_group_box.j_spin_box.value()
      m = self.tri_grasp_group_box.m_spin_box.value()
      t = self.tri_grasp_group_box.t_spin_box.value()
    self.hand.close([j,m,t],rest=self.close_rest_check_box.isChecked(),close_time=self.close_time_spin_box.value())

  def on_open_button(self):
    self.hand.open(vel=self.open_vel_slider.value()*3/100)

def main():

  rospy.init_node('arm_commander_gui_node')

  app = QApplication(sys.argv)

  mia_j_traj_ctrl = 'mia_hand_hw_vel_trajectory_controller'
  mia_j_vel_ctrl = 'mia_hand_joint_group_vel_controller'

  mia_ctrl_dict = {mia_j_traj_ctrl : JointTrajectory,
              mia_j_vel_ctrl : Float64MultiArray}

  hand = MiaHandCommander(ns='mia_hand',ctrl_dict=mia_ctrl_dict)
  
  hand_cmd_gui = MiaHandCommanderGui(hand)

  hand_cmd_gui.show()
  
  sys.exit(app.exec_())

if __name__ == '__main__':
  main()