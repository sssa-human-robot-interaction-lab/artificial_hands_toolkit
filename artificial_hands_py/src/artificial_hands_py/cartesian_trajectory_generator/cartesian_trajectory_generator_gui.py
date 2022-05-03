from cmath import pi
import sys
from threading import Thread
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt

import rospy, actionlib
import tf.transformations as ts
from geometry_msgs.msg import Pose
from artificial_hands_msgs.msg import *
from artificial_hands_py.artificial_hands_py_base import list_to_quat, quat_to_list

def new_param_item(lab : str, min : float, max : float, step : float, slider : bool = False):

  label = QLabel(lab)
  label.setAlignment(Qt.AlignCenter | Qt.AlignRight)
  spin_box = QDoubleSpinBox()
  spin_box.setDecimals(3)
  spin_box.setRange(min,max)
  spin_box.setSingleStep(step)
  if slider:
    sld = QSlider(Qt.Horizontal)
    sld.setRange(-100, 100)
    sld.setSingleStep(100/10)
    sld.valueChanged.connect(lambda: spin_box.setValue(sld.value()/100*max))
    spin_box.valueChanged.connect(lambda: sld.setValue(spin_box.value()*100/max))
    return label,spin_box,sld
  else:
    return label,spin_box
 
class TargetGroupBox(QGroupBox):

  def __init__(self,parent : QWidget = None, title : str = None, max : float = 1, step : float = .1):
    super().__init__()

    x_label,self.x_spin_box,x_sld = new_param_item('x',-max,max,step,True)
    y_label,self.y_spin_box,y_sld = new_param_item('y',-max,max,step,True)
    z_label,self.z_spin_box,z_sld = new_param_item('z',-max,max,step,True)

    spin_box_layout = QHBoxLayout()
    spin_box_layout.addWidget(x_label)
    spin_box_layout.addWidget(self.x_spin_box)
    spin_box_layout.addWidget(y_label)
    spin_box_layout.addWidget(self.y_spin_box)
    spin_box_layout.addWidget(z_label)
    spin_box_layout.addWidget(self.z_spin_box)

    sld_layout = QVBoxLayout()
    sld_layout.addWidget(x_sld)
    sld_layout.addWidget(y_sld)
    sld_layout.addWidget(z_sld)

    target_layout = QVBoxLayout()
    target_layout.addLayout(spin_box_layout)
    target_layout.addLayout(sld_layout)

    if title is not None:
      self.setTitle(title)
    self.setLayout(target_layout)
    
class TargetTable(QTableWidget):

  def __init__(self):
    super().__init__()
    self.setColumnCount(6)
    self.setMinimumWidth(400)
    self.setMinimumHeight(600)
    for c in range(0,6):
      self.setColumnWidth(c,int(400/6))

  def add_pose_target(self, target : Pose, row : int = None):
    if row is None:
      row = self.rowCount()
    self.insertRow(row) 
    self.setItem(row, 0, QTableWidgetItem(str(target.position.x)))
    self.setItem(row, 1, QTableWidgetItem(str(target.position.y)))
    self.setItem(row, 2, QTableWidgetItem(str(target.position.z)))
    rot = ts.euler_from_quaternion(quat_to_list(target.orientation))
    self.setItem(row, 3, QTableWidgetItem(str(rot[0])))
    self.setItem(row, 4, QTableWidgetItem(str(rot[1])))
    self.setItem(row, 5, QTableWidgetItem(str(rot[2])))

class CartesianTrajectoryGeneratorGUI(QWidget):

  def __init__(self,title : str = 'ATK Cartesian Trajectory Generator GUI') -> None:
    super().__init__()

    self.traj_cl = actionlib.SimpleActionClient('cartesian_trajectory_generator',TrajectoryGenerationAction)
    self.gen_cl = actionlib.SimpleActionClient('/cartesian_trajectory_plugin_manager',TrajectoryGenerationAction)

    self.teleop_running = False
    self.cancel_send_thread = False

    self.setWindowTitle(title)

    self.cart_traj_generator_combo_box = QComboBox()
    self.cart_traj_generator_combo_box.addItem('dmp_extended_trajectory_generator')
    self.cart_traj_generator_combo_box.addItem('harmonic_trajectory_generator')
    self.cart_traj_generator_combo_box.addItem('polynomial_345_trajectory_generator')
    self.cart_traj_generator_combo_box.addItem('polynomial_567_trajectory_generator')
    self.cart_traj_generator_combo_box.addItem('modified_trapezoidal_trajectory_genreator')
    self.cart_traj_generator_combo_box.setCurrentText('harmonic_trajectory_generator')

    self.teleop_check_box = QCheckBox('Telop')
    self.teleop_check_box.setChecked(False)
    self.teleop_check_box.setDisabled(True)

    dmp_ratio_label, self.dmp_ratio_spin_box  = new_param_item('Dmp ratio [-]:',0.1,1,0.05)
    self.dmp_ratio_spin_box.setValue(0.2)

    stop_time_label, self.stop_time_spin_box  = new_param_item('Stop time [s]:',0,1,0.05)
    self.stop_time_spin_box.setValue(0.3)

    self.params_layout = QHBoxLayout()
    self.params_layout.addWidget(self.teleop_check_box)
    self.params_layout.addWidget(dmp_ratio_label)
    self.params_layout.addWidget(self.dmp_ratio_spin_box)
    self.params_layout.addWidget(stop_time_label)
    self.params_layout.addWidget(self.stop_time_spin_box)

    self.target_position_group_box = TargetGroupBox(self,'Position [m]',max=1,step=0.001)
    self.target_orientation_group_box = TargetGroupBox(self,'Orientation [rad]',max=pi,step=0.001)
    
    self.add_push_button = QPushButton('Add')
    self.clear_push_button = QPushButton('Clear')
    self.send_push_button = QPushButton('Send')
    self.stop_push_button = QPushButton('Stop')
    
    buttons_row_layout = QHBoxLayout()
    buttons_row_layout.addWidget(self.add_push_button)
    buttons_row_layout.addWidget(self.clear_push_button)
    buttons_row_layout.addWidget(self.send_push_button)
    buttons_row_layout.addWidget(self.stop_push_button)

    self.target_table = TargetTable()

    main_layout = QVBoxLayout()
    main_layout.addWidget(self.cart_traj_generator_combo_box)
    main_layout.addLayout(self.params_layout)
    main_layout.addWidget(self.target_position_group_box)  
    main_layout.addWidget(self.target_orientation_group_box) 
    main_layout.addLayout(buttons_row_layout)  
    main_layout.addWidget(self.target_table) 

    self.add_push_button.clicked.connect(self.on_add_button)
    self.clear_push_button.clicked.connect(self.on_clear_button)

    self.setLayout(main_layout)  

  def connect(self):
    self.cart_traj_generator_combo_box.currentIndexChanged.connect(self.on_gen_changed)
    self.teleop_check_box.clicked.connect(self.on_teleop_check_box)
    self.send_push_button.clicked.connect(self.on_send_button)
    self.stop_push_button.clicked.connect(self.on_stop_button) 
    
  def on_gen_changed(self):
    goal = TrajectoryGenerationGoal()
    self.teleop_check_box.setChecked(False)
    self.teleop_check_box.setDisabled(True)
    if self.cart_traj_generator_combo_box.currentText() == 'dmp_extended_trajectory_generator':
      goal.traj_type = goal.DMP
      goal.dmp_ratio = self.dmp_ratio_spin_box.value()
      self.teleop_check_box.setEnabled(True)
    elif self.cart_traj_generator_combo_box.currentText() == 'harmonic_trajectory_generator':
      goal.traj_type = goal.HARMONIC
    elif self.cart_traj_generator_combo_box.currentText() == 'polynomial_345_trajectory_generator':
      goal.traj_type = goal.POLY345
    elif self.cart_traj_generator_combo_box.currentText() == 'polynomial_567_trajectory_generator':
      goal.traj_type = goal.POLY567
    elif self.cart_traj_generator_combo_box.currentText() == 'modified_trapezoidal_trajectory_genreator':
      goal.traj_type = goal.TRAPZMOD
    self.gen_cl.send_goal_and_wait(goal)

  def on_teleop_check_box(self):
    if self.teleop_check_box.isChecked():
      self.teleop_thread = Thread(target=self.update_teleop_target)
      self.teleop_running = True
      self.teleop_thread.start()
    else:
      self.teleop_running = False
      self.teleop_thread.join()

  def on_add_button(self):
    indices = self.target_table.selectionModel().selectedRows() 
    if len(indices) == 1:
      self.target_table.add_pose_target(self.get_current_target(),indices[0].row())
    else:
      self.target_table.add_pose_target(self.get_current_target())

  def on_clear_button(self):  
    if self.target_table.rowCount() == 0:
      return
    indices = self.target_table.selectionModel().selectedRows() 
    if len(indices) != 0:
      for index in sorted(indices):
          self.target_table.removeRow(index.row()) 
    else:
      msg = QMessageBox(self)
      msg.setText('Clear the target table?')
      msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
      msg.buttonClicked.connect(self.clear_table)
      msg.show()
  
  def on_send_button(self):
    goal = TrajectoryGenerationGoal()
    if self.cart_traj_generator_combo_box.currentText() == 'dmp_extended_trajectory_generator':
      goal.traj_type = goal.DMP
    elif self.cart_traj_generator_combo_box.currentText() == 'harmonic_trajectory_generator':
      goal.traj_type = goal.HARMONIC
    elif self.cart_traj_generator_combo_box.currentText() == 'polynomial_345_trajectory_generator':
      goal.traj_type = goal.POLY345
    elif self.cart_traj_generator_combo_box.currentText() == 'polynomial_567_trajectory_generator':
      goal.traj_type = goal.POLY567
    elif self.cart_traj_generator_combo_box.currentText() == 'modified_trapezoidal_trajectory_genreator':
      goal.traj_type = goal.TRAPZMOD

    if self.target_table.rowCount() == 0:
      goal.stop_time = self.stop_time_spin_box.value()
      goal.traj_target.pose = self.get_current_target()
      self.traj_cl.send_goal(goal)
    else:
      self.cancel_send_thread = False
      if self.send_waypoints_thread.is_alive():
        self.send_waypoints_thread.join()
      self.send_waypoints_thread = Thread(target=self.send_waypoints,args=(goal.traj_type,))
      self.send_waypoints_thread.start()
    
  def on_stop_button(self):
    self.cancel_send_thread = True
    goal = TrajectoryGenerationGoal()
    goal.traj_type = goal.STOP
    goal.stop_time = self.stop_time_spin_box.value()
    self.traj_cl.send_goal_and_wait(goal)

  def send_waypoints(self,traj_type):
    goal = TrajectoryGenerationGoal()
    goal.stop_time = self.stop_time_spin_box.value()
    goal.traj_type = traj_type
    for r in range(0,self.target_table.rowCount()):
      if self.cancel_send_thread:
        self.traj_cl.cancel_all_goals()
        return
      goal.traj_target.pose.position.x = float(self.target_table.item(r,0).text())
      goal.traj_target.pose.position.y = float(self.target_table.item(r,1).text())
      goal.traj_target.pose.position.z = float(self.target_table.item(r,2).text())
      goal.traj_target.pose.orientation = list_to_quat(
        ts.quaternion_from_euler(
          float(self.target_table.item(r,3).text()),
          float(self.target_table.item(r,4).text()),
          float(self.target_table.item(r,5).text())))
      self.traj_cl.send_goal_and_wait(goal)
  
  def update_teleop_target(self):
    goal = TrajectoryGenerationGoal()
    goal.traj_type = goal.DMP
    rate = rospy.Rate(30)
    while self.teleop_running:
      goal.traj_target.pose = self.get_current_target()
      self.traj_cl.send_goal(goal)
      rate.sleep()
  
  def clear_table(self,msgbtn):
    if msgbtn.text() == '&OK':
      self.target_table.setRowCount(0)
  
  def get_current_target(self):
    target = Pose()
    target.position.x = self.target_position_group_box.x_spin_box.value()
    target.position.y = self.target_position_group_box.y_spin_box.value()
    target.position.z = self.target_position_group_box.z_spin_box.value()
    target.orientation = list_to_quat(ts.quaternion_multiply(
      ts.quaternion_about_axis(self.target_orientation_group_box.z_spin_box.value(),[0,0,1]),
      ts.quaternion_multiply(
      ts.quaternion_about_axis(self.target_orientation_group_box.y_spin_box.value(),[0,1,0]),
      ts.quaternion_about_axis(self.target_orientation_group_box.x_spin_box.value(),[1,0,0]))))
    return target

def main():

  rospy.init_node('cartesian_trajectory_generator_gui_node')

  app = QApplication(sys.argv)

  cart_trj_gen_gui = CartesianTrajectoryGeneratorGUI()
  cart_trj_gen_gui.connect()
  cart_trj_gen_gui.show()

  sys.exit(app.exec_())

if __name__ == '__main__':
  main()