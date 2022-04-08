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
from artificial_hands_py.cartesian_trajectory_generator.cartesian_publishers import CartesianMDOFPointPublisher, PoseStampedPublisher
from artificial_hands_py.robot_commander.arm_commander import ArmCommander

class ArmCommanderGui(QWidget):

  def __init__(self,arm : ArmCommander) -> None:
    super().__init__()

    self.arm = arm

    self.ctrl_combo_box = QComboBox()
    self.ctrl_combo_box.addItems(list(arm.ctrl_dict.keys())[1:])
    self.ctrl_combo_box.currentIndexChanged.connect(self.on_ctrl_changed)
    
    self.cart_traj_gen_widget = CartesianTrajectoryGeneratorGUI()

    main_layout = QVBoxLayout()
    main_layout.addWidget(self.ctrl_combo_box)
    main_layout.addWidget(self.cart_traj_gen_widget)

    self.setLayout(main_layout)
    
    c_pose = self.arm.get_current_frame().pose
    c_rot = ts.euler_from_quaternion(quat_to_list(c_pose.orientation))
    
    self.cart_traj_gen_widget.target_position_group_box.x_spin_box.setValue(c_pose.position.x)
    self.cart_traj_gen_widget.target_position_group_box.y_spin_box.setValue(c_pose.position.y)
    self.cart_traj_gen_widget.target_position_group_box.z_spin_box.setValue(c_pose.position.z)

    self.cart_traj_gen_widget.target_orientation_group_box.x_spin_box.setValue(c_rot[0])
    self.cart_traj_gen_widget.target_orientation_group_box.y_spin_box.setValue(c_rot[1])
    self.cart_traj_gen_widget.target_orientation_group_box.z_spin_box.setValue(c_rot[2])

    print(c_pose.position)
    print(c_rot)

    self.arm.switch_to_cartesian_controller(list(arm.ctrl_dict.keys())[1])

    self.cart_traj_gen_widget.send_push_button.clicked.connect(self.on_send_button)
    self.cart_traj_gen_widget.stop_push_button.clicked.connect(self.on_cancel_button) 

  def send_goals(self):
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

  def on_send_button(self):
    if self.cart_traj_gen_widget.target_table.rowCount() == 0:
      self.arm.set_pose_target(self.cart_traj_gen_widget.get_current_target())
    else:
      self.cancel_send_thread = False
      send_thread = Thread(target=self.send_goals)
      send_thread.start()

  def on_cancel_button(self):
    self.cancel_send_thread = True
  
  def on_ctrl_changed(self):
    self.arm.switch_to_cartesian_controller(self.ctrl_combo_box.currentText())
    
def main():

  rospy.init_node('arm_commander_gui_node')

  app = QApplication(sys.argv)

  j_traj_pos_ctrl = 'pos_joint_traj_controller'
  c_eik_pos_ctrl = 'cartesian_eik_position_controller'
  c_eik_vel_ctrl= 'cartesian_eik_velocity_controller'
  cart_mot_pos_ctrl = 'cartesian_motion_position_controller'
  cart_mot_vel_ctrl = 'cartesian_motion_velocity_controller'

  ctrl_dict = {j_traj_pos_ctrl : JointTrajectory,
              c_eik_pos_ctrl : MultiDOFJointTrajectory,
              c_eik_vel_ctrl : MultiDOFJointTrajectory,
              cart_mot_pos_ctrl : PoseStamped,
              cart_mot_vel_ctrl : PoseStamped}

  cart_eik_pos_pub = CartesianMDOFPointPublisher(c_eik_pos_ctrl+'/command')
  cart_eik_vel_pub = CartesianMDOFPointPublisher(c_eik_vel_ctrl+'/command')
  cart_mot_pos_pub = PoseStampedPublisher(cart_mot_pos_ctrl+'/command')
  cart_mot_vel_pub = PoseStampedPublisher(cart_mot_vel_ctrl+'/command')

  arm = ArmCommander(ctrl_dict=ctrl_dict)

  arm_cmd_gui = ArmCommanderGui(arm)
  
  arm_cmd_gui.show()
  
  sys.exit(app.exec_())

if __name__ == '__main__':
  main()