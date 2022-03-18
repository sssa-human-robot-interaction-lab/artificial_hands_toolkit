import sys
from syslog import LOG_WARNING
from threading import Thread

from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt

from handover_proprioceptive_base import *

class MaxDispGroupBox(QGroupBox):

  def __init__(self, title : str = None):
    super().__init__()

    x_label = QLabel('x:')
    y_label = QLabel('y:')
    z_label = QLabel('z:')

    x_label.setAlignment(Qt.AlignCenter | Qt.AlignRight)
    y_label.setAlignment(Qt.AlignCenter | Qt.AlignRight)
    z_label.setAlignment(Qt.AlignCenter | Qt.AlignRight)

    self.delta_x = QLabel('-')
    self.delta_y = QLabel('-')
    self.delta_z = QLabel('-')

    max_disp_layout = QHBoxLayout()
    max_disp_layout.addWidget(x_label)
    max_disp_layout.addWidget(self.delta_x)
    max_disp_layout.addWidget(y_label)
    max_disp_layout.addWidget(self.delta_y)
    max_disp_layout.addWidget(z_label)
    max_disp_layout.addWidget(self.delta_z)
    if title is not None:
      self.setTitle(title)
    self.setLayout(max_disp_layout)

class TargetGroupBox(QGroupBox):

  def __init__(self,parent : QWidget = None, title : str = None, max : float = 1):
    super().__init__()

    x_label = QLabel('x')
    y_label = QLabel('y')
    z_label = QLabel('z')

    x_label.setAlignment(Qt.AlignCenter | Qt.AlignRight)
    y_label.setAlignment(Qt.AlignCenter | Qt.AlignRight)
    z_label.setAlignment(Qt.AlignCenter | Qt.AlignRight)

    self.max_x_spin_box = QDoubleSpinBox()
    self.max_y_spin_box = QDoubleSpinBox()
    self.max_z_spin_box = QDoubleSpinBox()

    self.max_x_spin_box.setRange(-max,max)
    self.max_y_spin_box.setRange(-max,max)
    self.max_z_spin_box.setRange(-max,max)

    self.max_x_spin_box.setSingleStep(.2)
    self.max_y_spin_box.setSingleStep(.2)
    self.max_z_spin_box.setSingleStep(.2)

    self.max_x_spin_box.valueChanged.connect(parent.traj_design_changed)
    self.max_y_spin_box.valueChanged.connect(parent.traj_design_changed)
    self.max_z_spin_box.valueChanged.connect(parent.traj_design_changed)

    twist_target_layout = QHBoxLayout()
    twist_target_layout.addWidget(x_label)
    twist_target_layout.addWidget(self.max_x_spin_box)
    twist_target_layout.addWidget(y_label)
    twist_target_layout.addWidget(self.max_y_spin_box)
    twist_target_layout.addWidget(z_label)
    twist_target_layout.addWidget(self.max_z_spin_box)
    if title is not None:
      self.setTitle(title)
    self.setLayout(twist_target_layout)

class RobotCommanderGui(QWidget):

  def __init__(self) -> None:
    super().__init__()

    rate = rospy.Rate(500)    
    self.robot = RobotCommander()
    self.robot.twist_servo.set_rate(rate)

    self.setWindowTitle('ATK Twist Commander GUI')

    goal_time_label = QLabel('Goal time [s]')
    goal_time_label.setAlignment(Qt.AlignCenter | Qt.AlignRight)
    
    self.goal_time_spin_box = QDoubleSpinBox()
    self.goal_time_spin_box.setSingleStep(.1)
    self.goal_time_spin_box.setValue(3)
    self.goal_time_spin_box.valueChanged.connect(self.traj_design_changed)

    self.delta_percent_label = QLabel('\u03B4 [%]')
    self.delta_percent_label.setDisabled(True)
    self.delta_percent_label.setAlignment(Qt.AlignCenter | Qt.AlignRight)

    self.delta_percent_spin_box = QDoubleSpinBox()
    self.delta_percent_spin_box.setSingleStep(.1)
    self.delta_percent_spin_box.setDisabled(True)
    self.delta_percent_spin_box.setValue(.5)
    self.delta_percent_spin_box.setRange(.1,.5)
    self.delta_percent_spin_box.valueChanged.connect(self.traj_design_changed)

    self.traj_type_combo_box = QComboBox()
    self.traj_type_combo_box.addItem('Harmonic')
    self.traj_type_combo_box.addItem('Biharmonic2')
    self.traj_type_combo_box.addItem('Trapezoidal')
    self.traj_type_combo_box.currentTextChanged.connect(self.traj_design_changed)

    first_row_layout = QHBoxLayout()
    first_row_layout.addWidget(goal_time_label)
    first_row_layout.addWidget(self.goal_time_spin_box)
    first_row_layout.addWidget(self.traj_type_combo_box)
    first_row_layout.addWidget(self.delta_percent_label)
    first_row_layout.addWidget(self.delta_percent_spin_box)

    self.twist_target_lin_accel_group_box = TargetGroupBox(self,'Max lin accel [m/s2]',2)
    self.twist_target_ang_accel_group_box = TargetGroupBox(self,'Max ang accel [rad/s2]',3)
    self.max_lin_disp_group_box = MaxDispGroupBox('Max lin displacement [mm] ')
    self.max_ang_disp_group_box = MaxDispGroupBox('Max ang displacement [deg] ')

    self.send_push_button = QPushButton('Send')
    self.stop_push_button = QPushButton('Stop')
    self.home_push_button = QPushButton('Home')
    self.calib_push_button = QPushButton('Calib')
    self.recog_push_button = QPushButton('Recog')
    self.wrench_push_button = QPushButton('Wrench')

    self.send_push_button.clicked.connect(self.on_send_button)
    self.stop_push_button.clicked.connect(self.on_pause_button)
    self.home_push_button.clicked.connect(self.on_home_button)
    self.calib_push_button.clicked.connect(self.on_calib_button)
    self.recog_push_button.clicked.connect(self.on_recog_button)
    self.wrench_push_button.clicked.connect(self.on_wrench_button)

    buttons_row_layout = QHBoxLayout()
    buttons_row_layout.addWidget(self.send_push_button)
    buttons_row_layout.addWidget(self.stop_push_button)
    buttons_row_layout.addWidget(self.home_push_button)
    buttons_row_layout.addWidget(self.calib_push_button)
    buttons_row_layout.addWidget(self.recog_push_button)
    buttons_row_layout.addWidget(self.wrench_push_button)
   
    main_layout = QVBoxLayout()
    main_layout.addLayout(first_row_layout)
    main_layout.addWidget(self.twist_target_lin_accel_group_box)
    main_layout.addWidget(self.twist_target_ang_accel_group_box)  
    main_layout.addWidget(self.max_lin_disp_group_box)
    main_layout.addWidget(self.max_ang_disp_group_box)   
    main_layout.addLayout(buttons_row_layout)

    self.setLayout(main_layout)

    self.traj_design_changed()
  
  def on_send_button(self):
    self.robot.twist_servo.set_preempted()
    self.traj_design_changed()
  
  def on_pause_button(self):
    self.robot.twist_servo.paused = True

  def on_home_button(self):
    self.send_push_button.setDisabled(True)
    self.stop_push_button.setDisabled(True)
    self.home_push_button.setDisabled(True)

    joint_ini = np.array([76.68,-127.80,114.44,-165.77,-77.07,180.45])
    joint_ini = joint_ini/180*pi  

    self.robot.twist_servo.go(joint_ini,wait=True)

    self.send_push_button.setEnabled(True)
    self.stop_push_button.setEnabled(True)
    self.home_push_button.setEnabled(True)
  
  def on_calib_button(self):
    if not self.robot.check_calib():
      self.robot.do_calib()
      self.robot.arm.go(joint_home, wait=True)
  
  def on_recog_button(self):
    self.robot.start_object_recognition()

  def on_wrench_button(self):
    self.robot.estimate_wrench()

  def servo_robot(self,ts,type,delta_pos,delta_ang):
    self.send_push_button.setDisabled(True)
    self.robot.twist_servo.servo_delta(ts,type,delta_pos,delta_ang)
    self.send_push_button.setEnabled(True)
    self.robot.twist_servo.preempted = False

    if self.robot.wrist.recog_preempted:
      self.robot.wrist.recog_preempted = False
      self.robot.build_object_model()

  def traj_design_changed(self):

    ts = self.goal_time_spin_box.value()

    type = self.traj_type_combo_box.currentText().lower()

    hlx = 0
    hly = 0
    hlz = 0
    hax = 0
    hay = 0
    haz = 0

    if type == 'trapezoidal':
      self.delta_percent_label.setEnabled(True)
      self.delta_percent_spin_box.setEnabled(True)

      delta = self.delta_percent_spin_box.value()
      tv = (ts -delta)/2

      hlx = self.twist_target_lin_accel_group_box.max_x_spin_box.value()*(tv*(ts-tv))
      hly = self.twist_target_lin_accel_group_box.max_y_spin_box.value()*(tv*(ts-tv))
      hlz = self.twist_target_lin_accel_group_box.max_z_spin_box.value()*(tv*(ts-tv))

      hax = self.twist_target_ang_accel_group_box.max_x_spin_box.value()*(tv*(ts-tv))
      hay = self.twist_target_ang_accel_group_box.max_y_spin_box.value()*(tv*(ts-tv))
      haz = self.twist_target_ang_accel_group_box.max_z_spin_box.value()*(tv*(ts-tv))

    else:
      self.delta_percent_label.setDisabled(True)
      self.delta_percent_spin_box.setDisabled(True)
      
      if type == 'harmonic':
        
        hlx = self.twist_target_lin_accel_group_box.max_x_spin_box.value()/6.28*pow(ts,2)
        hly = self.twist_target_lin_accel_group_box.max_y_spin_box.value()/6.28*pow(ts,2)
        hlz = self.twist_target_lin_accel_group_box.max_z_spin_box.value()/6.28*pow(ts,2)

        hax = self.twist_target_ang_accel_group_box.max_x_spin_box.value()/6.28*pow(ts,2)
        hay = self.twist_target_ang_accel_group_box.max_y_spin_box.value()/6.28*pow(ts,2)
        haz = self.twist_target_ang_accel_group_box.max_z_spin_box.value()/6.28*pow(ts,2)
      
      elif type == 'biharmonic2':

        hlx = self.twist_target_lin_accel_group_box.max_x_spin_box.value()/9.9*pow(ts/2,2)
        hly = self.twist_target_lin_accel_group_box.max_y_spin_box.value()/9.9*pow(ts/2,2)
        hlz = self.twist_target_lin_accel_group_box.max_z_spin_box.value()/9.9*pow(ts/2,2)

        hax = self.twist_target_ang_accel_group_box.max_x_spin_box.value()/9.9*pow(ts/2,2)
        hay = self.twist_target_ang_accel_group_box.max_y_spin_box.value()/9.9*pow(ts/2,2)
        haz = self.twist_target_ang_accel_group_box.max_z_spin_box.value()/9.9*pow(ts/2,2)
    
    self.max_lin_disp_group_box.delta_x.setText("{d: .2f}".format(d = hlx*1000))
    self.max_lin_disp_group_box.delta_y.setText("{d: .2f}".format(d = hly*1000))
    self.max_lin_disp_group_box.delta_z.setText("{d: .2f}".format(d = hlz*1000))

    self.max_ang_disp_group_box.delta_x.setText("{d: .2f}".format(d = hax*180/pi))
    self.max_ang_disp_group_box.delta_y.setText("{d: .2f}".format(d = hay*180/pi))
    self.max_ang_disp_group_box.delta_z.setText("{d: .2f}".format(d = haz*180/pi))

    if self.robot.twist_servo.preempted:

      delta_pos = Point()
      delta_pos.x = hlx
      delta_pos.y = hly
      delta_pos.z = hlz

      delta_ang = Point()
      delta_ang.x = hax
      delta_ang.y = hay
      delta_ang.z = haz

      t = Thread(target=self.servo_robot,args=(ts,type,delta_pos,delta_ang,))
      t.start()

def main():

  rospy.init_node('robot_commander_gui_node')

  app = QApplication(sys.argv)

  robot_commander_gui = RobotCommanderGui()
  robot_commander_gui.show()

  sys.exit(app.exec_())

if __name__ == '__main__':
  main()
