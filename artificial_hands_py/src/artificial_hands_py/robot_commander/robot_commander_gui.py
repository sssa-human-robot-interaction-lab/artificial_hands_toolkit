import sys
from threading import Thread
from PyQt5.QtWidgets import *

import rospy
import tf.transformations as ts
from geometry_msgs.msg import Pose

from artificial_hands_msgs.msg import *
from artificial_hands_py.artificial_hands_py_base import list_to_quat, quat_to_list
from artificial_hands_py.cartesian_trajectory_generator.cartesian_trajectory_generator_gui import CartesianTrajectoryGeneratorGUI
from artificial_hands_py.cartesian_trajectory_generator.cartesian_publishers import CartesianMDOFPointPublisher, PoseStampedPublisher
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
      self.robot.set_pose_target(target)

  def on_send_button(self):
    if self.cart_traj_gen_widget.target_table.rowCount() == 0:
      self.robot.set_pose_target(self.cart_traj_gen_widget.get_current_target())
    else:
      self.cancel_send_thread = False
      send_thread = Thread(target=self.send_goals)
      send_thread.start()

  def on_cancel_button(self):
    self.cancel_send_thread = True
    
def main():

  rospy.init_node('robot_commander_gui_node')

  app = QApplication(sys.argv)

  robot_cmd_gui = RobotCommanderGUI()
  # cart_mdof_pnt_pub = CartesianMDOFPointPublisher('/cartesian_eik_position_controller/command')
  pose_st_pub = PoseStampedPublisher('/cartesian_motion_controller/command')
  robot_cmd_gui.show()
  
  sys.exit(app.exec_())

if __name__ == '__main__':
  main()