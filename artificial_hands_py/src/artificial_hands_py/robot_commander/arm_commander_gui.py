import sys
from threading import Thread
from PyQt5.QtWidgets import *

import rospy
import tf.transformations as ts
from geometry_msgs.msg import Pose, PoseStamped
from trajectory_msgs.msg import JointTrajectory, MultiDOFJointTrajectory

from artificial_hands_msgs.msg import *
from artificial_hands_py.artificial_hands_py_base import list_to_quat, quat_to_list
from artificial_hands_py.cartesian_trajectory_generator.cartesian_trajectory_generator_gui import CartesianTrajectoryGeneratorGUI
from artificial_hands_py.cartesian_trajectory_generator.cartesian_publishers import CartesianTrajectoryPointPublisher, PoseStampedPublisher
from artificial_hands_py.robot_commander.arm_commander import ArmCommander

class ArmCommanderGui(QWidget):

  def __init__(self,arm : ArmCommander) -> None:
    super().__init__()

    self.arm = arm

    self.send_waypoints_thread = Thread()

    self.cart_motion_ctrl_combo_box = QComboBox()
    self.cart_motion_ctrl_combo_box.addItems(list(arm.ctrl_dict.keys())[1:])
    
    self.cart_traj_gen_widget = CartesianTrajectoryGeneratorGUI()

    self.cart_traj_progress_bar = QProgressBar()
    self.cart_traj_feedback_thread = Thread(target=self.update_traj_percentage)
    self.cart_traj_feedback_thread.start()

    main_layout = QVBoxLayout()
    main_layout.addWidget(self.cart_motion_ctrl_combo_box)
    main_layout.addWidget(self.cart_traj_gen_widget)
    main_layout.addWidget(self.cart_traj_progress_bar)

    self.setLayout(main_layout)
    
    c_pose = self.arm.get_current_frame().pose
    c_rot = ts.euler_from_quaternion(quat_to_list(c_pose.orientation))
    
    self.cart_traj_gen_widget.target_position_group_box.x_spin_box.setValue(c_pose.position.x)
    self.cart_traj_gen_widget.target_position_group_box.y_spin_box.setValue(c_pose.position.y)
    self.cart_traj_gen_widget.target_position_group_box.z_spin_box.setValue(c_pose.position.z)

    self.cart_traj_gen_widget.target_orientation_group_box.x_spin_box.setValue(c_rot[0])
    self.cart_traj_gen_widget.target_orientation_group_box.y_spin_box.setValue(c_rot[1])
    self.cart_traj_gen_widget.target_orientation_group_box.z_spin_box.setValue(c_rot[2])

    self.arm.set_harmonic_traj_generator()
    self.arm.switch_to_cartesian_controller(list(arm.ctrl_dict.keys())[1])

    self.cart_motion_ctrl_combo_box.currentIndexChanged.connect(self.on_ctrl_changed)
    self.cart_traj_gen_widget.cart_traj_generator_combo_box.currentIndexChanged.connect(self.on_gen_changed) 
    self.cart_traj_gen_widget.teleop_check_box.clicked.connect(self.on_teleop_check_box) 
    self.cart_traj_gen_widget.track_ratio_spin_box.valueChanged.connect(self.on_track_ratio_changed) 
    self.cart_traj_gen_widget.stop_time_spin_box.valueChanged.connect(self.on_stop_time_changed)
    self.cart_traj_gen_widget.send_push_button.clicked.connect(self.on_send_button)
    self.cart_traj_gen_widget.stop_push_button.clicked.connect(self.on_stop_button) 
  
  def on_ctrl_changed(self):
    self.arm.switch_to_cartesian_controller(self.cart_motion_ctrl_combo_box.currentText())
  
  def on_gen_changed(self):
    self.cart_traj_gen_widget.teleop_check_box.setChecked(False)
    self.cart_traj_gen_widget.teleop_check_box.setDisabled(True)
    if self.cart_traj_gen_widget.cart_traj_generator_combo_box.currentText() == 'mj_tracker_trajectory_generator':
      self.arm.set_mj_traj_generator()
      self.cart_traj_gen_widget.teleop_check_box.setEnabled(True)
    elif self.cart_traj_gen_widget.cart_traj_generator_combo_box.currentText() == 'harmonic_trajectory_generator':
      self.arm.set_harmonic_traj_generator()
    elif self.cart_traj_gen_widget.cart_traj_generator_combo_box.currentText() == 'polynomial_345_trajectory_generator':
      self.arm.set_poly_345_traj_generator()
    elif self.cart_traj_gen_widget.cart_traj_generator_combo_box.currentText() == 'polynomial_567_trajectory_generator':
      self.arm.set_poly_567_traj_generator()
    elif self.cart_traj_gen_widget.cart_traj_generator_combo_box.currentText() == 'modified_trapezoidal_trajectory_genreator':
      self.arm.set_mod_trapz_traj_generator()
  
  def on_teleop_check_box(self):
    if self.cart_traj_gen_widget.teleop_check_box.isChecked():
      self.teleop_thread = Thread(target=self.update_teleop_target)
      self.cart_traj_gen_widget.teleop_running = True
      self.teleop_thread.start()
    else:
      self.cart_traj_gen_widget.teleop_running = False
      self.teleop_thread.join()
  
  def on_track_ratio_changed(self):
    self.arm.set_track_ratio(self.cart_traj_gen_widget.track_ratio_spin_box.value())

  def on_stop_time_changed(self):
    self.arm.set_stop_time(self.cart_traj_gen_widget.stop_time_spin_box.value())

  def on_send_button(self):
    if self.cart_traj_gen_widget.target_table.rowCount() == 0:
      self.arm.set_pose_target(self.cart_traj_gen_widget.get_current_target(),False)
      self.arm.update_trajectory_monitor()
    else:
      self.cancel_send_thread = False
      if self.send_waypoints_thread.is_alive():
        self.send_waypoints_thread.join()
      self.send_waypoints_thread = Thread(target=self.send_waypoints)
      self.send_waypoints_thread.start()

  def on_stop_button(self):
    self.cancel_send_thread = True
    self.arm.stop()
    self.on_gen_changed()
  
  def send_waypoints(self):
    target = Pose()
    for r in range(0,self.cart_traj_gen_widget.target_table.rowCount()):
      if self.cancel_send_thread:
        return
      target.position.x = float(self.cart_traj_gen_widget.target_table.item(r,0).text())
      target.position.y = float(self.cart_traj_gen_widget.target_table.item(r,1).text())
      target.position.z = float(self.cart_traj_gen_widget.target_table.item(r,2).text())
      target.orientation = list_to_quat(
        ts.quaternion_from_euler(
          float(self.cart_traj_gen_widget.target_table.item(r,3).text()),
          float(self.cart_traj_gen_widget.target_table.item(r,4).text()),
          float(self.cart_traj_gen_widget.target_table.item(r,5).text())))
      self.arm.set_pose_target(target)
      rospy.sleep(rospy.Duration().from_sec(float(self.cart_traj_gen_widget.target_table.item(r,6).text())))
  
  def update_teleop_target(self):
    rate = rospy.Rate(30)
    while self.cart_traj_gen_widget.teleop_running:
      self.arm.set_pose_target(self.cart_traj_gen_widget.get_current_target(),False)
      rate.sleep()
  
  def update_traj_percentage(self):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      self.cart_traj_progress_bar.setValue(self.arm.percentage)
      rate.sleep()
    
def main():

  rospy.init_node('arm_commander_gui_node')

  app = QApplication(sys.argv)

  arm = ArmCommander()

  arm_cmd_gui = ArmCommanderGui(arm)
  
  arm_cmd_gui.show()
  
  sys.exit(app.exec_())

if __name__ == '__main__':
  main()