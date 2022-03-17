import sys

from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt

class TargetGroupBox(QGroupBox):

  def __init__(self,title : str = None, max : float = 1):
    super().__init__()

    x_label = QLabel('x')
    x_label.setAlignment(Qt.AlignCenter | Qt.AlignRight)
    y_label = QLabel('y')
    y_label.setAlignment(Qt.AlignCenter | Qt.AlignRight)
    z_label = QLabel('z')
    z_label.setAlignment(Qt.AlignCenter | Qt.AlignRight)
    max_x_spin_box = QDoubleSpinBox()
    max_y_spin_box = QDoubleSpinBox()
    max_z_spin_box = QDoubleSpinBox()
    max_x_spin_box.setRange(-max,max)
    max_y_spin_box.setRange(-max,max)
    max_z_spin_box.setRange(-max,max)
    max_x_spin_box.setSingleStep(.2)
    max_y_spin_box.setSingleStep(.2)
    max_z_spin_box.setSingleStep(.2)
    twist_target_layout = QHBoxLayout()
    twist_target_layout.addWidget(x_label)
    twist_target_layout.addWidget(max_x_spin_box)
    twist_target_layout.addWidget(y_label)
    twist_target_layout.addWidget(max_y_spin_box)
    twist_target_layout.addWidget(z_label)
    twist_target_layout.addWidget(max_z_spin_box)
    if title is not None:
      self.setTitle(title)
    self.setLayout(twist_target_layout)

class RobotoCommanderGui(QWidget):

  def __init__(self) -> None:
    super().__init__()

    self.setWindowTitle('ATK Twist Commander GUI')

    goal_time_label = QLabel('Goal time [s]')
    goal_time_label.setAlignment(Qt.AlignCenter | Qt.AlignRight)
    goal_time_spin_box = QDoubleSpinBox()
    goal_time_spin_box.setSingleStep(.1)
    delta_percent_label = QLabel('\u03B4 [%]')
    delta_percent_label.setDisabled(True)
    delta_percent_label.setAlignment(Qt.AlignCenter | Qt.AlignRight)
    delta_percent_spin_box = QDoubleSpinBox()
    delta_percent_spin_box.setSingleStep(.1)
    delta_percent_spin_box.setDisabled(True)
    traj_type_combo_box = QComboBox()
    traj_type_combo_box.addItem('Harmonic')
    traj_type_combo_box.addItem('Biharmonic2')
    traj_type_combo_box.addItem('Trapezoidal')
    first_row_layout = QHBoxLayout()
    first_row_layout.addWidget(goal_time_label)
    first_row_layout.addWidget(goal_time_spin_box)
    first_row_layout.addWidget(traj_type_combo_box)
    first_row_layout.addWidget(delta_percent_label)
    first_row_layout.addWidget(delta_percent_spin_box)

    twist_target_lin_accel_group_box = TargetGroupBox('Max lin accel [m/s2]',2)
    second_row_layout = QVBoxLayout()
    second_row_layout.addWidget(twist_target_lin_accel_group_box)

    twist_target_lin_vel_group_box = TargetGroupBox('Max ang accel [rad/s2]',3)
    third_row_layout = QVBoxLayout()
    third_row_layout.addWidget(twist_target_lin_vel_group_box)

    main_layout = QVBoxLayout()
    main_layout.addLayout(first_row_layout)
    main_layout.addLayout(second_row_layout)
    main_layout.addLayout(third_row_layout)
    self.setLayout(main_layout)

def main():
  app = QApplication(sys.argv)

  robot_commander_gui = RobotoCommanderGui()
  robot_commander_gui.show()

  sys.exit(app.exec_())

if __name__ == '__main__':
  main()
