import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt

import rospy, actionlib
import tf.transformations as ts

from artificial_hands_msgs.msg import *
from artificial_hands_py.artificial_hands_py_base import list_to_quat

class TargetGroupBox(QGroupBox):

  def __init__(self,parent : QWidget = None, title : str = None, max : float = 1):
    super().__init__()

    x_label = QLabel('x')
    y_label = QLabel('y')
    z_label = QLabel('z')

    x_label.setAlignment(Qt.AlignCenter | Qt.AlignRight)
    y_label.setAlignment(Qt.AlignCenter | Qt.AlignRight)
    z_label.setAlignment(Qt.AlignCenter | Qt.AlignRight)

    self.x_spin_box = QDoubleSpinBox()
    self.y_spin_box = QDoubleSpinBox()
    self.z_spin_box = QDoubleSpinBox()

    self.x_spin_box.setRange(-max,max)
    self.y_spin_box.setRange(-max,max)
    self.z_spin_box.setRange(-max,max)

    self.x_spin_box.setSingleStep(.2)
    self.y_spin_box.setSingleStep(.2)
    self.z_spin_box.setSingleStep(.2)

    target_layout = QHBoxLayout()
    target_layout.addWidget(x_label)
    target_layout.addWidget(self.x_spin_box)
    target_layout.addWidget(y_label)
    target_layout.addWidget(self.y_spin_box)
    target_layout.addWidget(z_label)
    target_layout.addWidget(self.z_spin_box)
    if title is not None:
      self.setTitle(title)
    self.setLayout(target_layout)

class CartesianTrajectoryGeneratorGUI(QWidget):

  def __init__(self,title : str = 'ATK Cartesian Trajectory Generator GUI') -> None:
    super().__init__()

    self.traj_cl = actionlib.SimpleActionClient('cartesian_trajectory_generator',TrajectoryGenerationAction)

    self.setWindowTitle(title)

    self.target_position_group_box = TargetGroupBox(self,'Position [m]')
    self.target_orientation_group_box = TargetGroupBox(self,'Orientation [rad]')
    
    self.send_push_button = QPushButton('Send')
    self.stop_push_button = QPushButton('Stop')
    
    buttons_row_layout = QHBoxLayout()
    buttons_row_layout.addWidget(self.send_push_button)
    buttons_row_layout.addWidget(self.stop_push_button)

    main_layout = QVBoxLayout()
    main_layout.addWidget(self.target_position_group_box)  
    main_layout.addWidget(self.target_orientation_group_box)  
    main_layout.addLayout(buttons_row_layout)  

    self.setLayout(main_layout)  

  def connect(self):
    self.send_push_button.clicked.connect(self.on_send_button)
    self.stop_push_button.clicked.connect(self.on_cancel_button) 

  def on_send_button(self):
    goal = TrajectoryGenerationGoal()
    goal.traj_target.pose.position.x = self.target_position_group_box.x_spin_box.value()
    goal.traj_target.pose.position.y = self.target_position_group_box.y_spin_box.value()
    goal.traj_target.pose.position.z = self.target_position_group_box.z_spin_box.value()

    goal.traj_target.pose.orientation = list_to_quat(ts.quaternion_from_euler(
      self.target_orientation_group_box.x_spin_box.value(),
      self.target_orientation_group_box.y_spin_box.value(),
      self.target_orientation_group_box.z_spin_box.value()))

    self.traj_cl.send_goal(goal)
  
  def on_cancel_button(self):
    self.traj_cl.cancel_all_goals()
    
def main():

  rospy.init_node('cartesian_trajectory_generator_gui_node')

  app = QApplication(sys.argv)

  cart_trj_gen_gui = CartesianTrajectoryGeneratorGUI()
  cart_trj_gen_gui.connect()
  cart_trj_gen_gui.show()

  sys.exit(app.exec_())

if __name__ == '__main__':
  main()