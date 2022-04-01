import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt

import rospy
import tf.transformations as ts
from geometry_msgs.msg import Pose

from artificial_hands_msgs.msg import *
from artificial_hands_py.artificial_hands_py_base import list_to_quat, quat_to_list
from artificial_hands_py.cartesian_trajectory_generator.cartesian_trajectory_generator_gui import CartesianTrajectoryGeneratorGUI
from artificial_hands_py.cartesian_trajectory_generator.cartesian_mdof_point_publisher import CartesianMDOFPointPublisher
from artificial_hands_py.robot_commander.robot_commander import RobotCommander

class RobotCommanderGUI(QWidget):

  def __init__(self) -> None:
    super().__init__()

    self.robot = RobotCommander()
    
    self.cart_traj_gen_widget = CartesianTrajectoryGeneratorGUI()

    main_layout = QVBoxLayout()
    main_layout.addWidget(self.cart_traj_gen_widget)

    self.setLayout(main_layout)
    
    c_pose = self.robot.get_current_frame().pose
    c_rot = ts.euler_from_quaternion(quat_to_list(c_pose.orientation))
    
    self.cart_traj_gen_widget.target_position_group_box.x_spin_box.setValue(c_pose.position.x)
    self.cart_traj_gen_widget.target_position_group_box.y_spin_box.setValue(c_pose.position.y)
    self.cart_traj_gen_widget.target_position_group_box.z_spin_box.setValue(c_pose.position.z)

    self.cart_traj_gen_widget.target_orientation_group_box.x_spin_box.setValue(c_rot[0])
    self.cart_traj_gen_widget.target_orientation_group_box.y_spin_box.setValue(c_rot[1])
    self.cart_traj_gen_widget.target_orientation_group_box.z_spin_box.setValue(c_rot[2])

    print(c_pose.position)
    print(c_rot)

    self.robot.set_pose_target(c_pose)
    self.robot.switch_to_cartesian_trajectory()

    self.cart_traj_gen_widget.send_push_button.clicked.connect(self.on_send_button)
    self.cart_traj_gen_widget.stop_push_button.clicked.connect(self.on_cancel_button) 

  def on_send_button(self):
    target = Pose()
    target.position.x = self.cart_traj_gen_widget.target_position_group_box.x_spin_box.value()
    target.position.y = self.cart_traj_gen_widget.target_position_group_box.y_spin_box.value()
    target.position.z = self.cart_traj_gen_widget.target_position_group_box.z_spin_box.value()
    target.orientation = list_to_quat(ts.quaternion_from_euler(
      self.cart_traj_gen_widget.target_orientation_group_box.x_spin_box.value(),
      self.cart_traj_gen_widget.target_orientation_group_box.y_spin_box.value(),
      self.cart_traj_gen_widget.target_orientation_group_box.z_spin_box.value()))
    self.robot.set_pose_target(target)
  
  def on_cancel_button(self):
    self.robot.cancel_pose_target()
    
def main():

  rospy.init_node('robot_commander_gui_node')

  app = QApplication(sys.argv)

  robot_cmd_gui = RobotCommanderGUI()
  cart_mdof_pnt_pub = CartesianMDOFPointPublisher('/cartesian_eik_position_controller/command')
  robot_cmd_gui.show()
  
  sys.exit(app.exec_())

if __name__ == '__main__':
  main()