from math import floor
from time import sleep
from threading import Thread
import csv

import rospy

from std_msgs.msg import Float64MultiArray, Float64
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint

from artificial_hands_py.cartesian_trajectory_generator.cartesian_trajectory_base import harmonic_pos, harmonic_vel, harmonic_accel
from artificial_hands_py.robot_commander.mia_hand_commander import MiaHandCommander

class MiaHandTrajectoryTest:

  def __init__(self,mia_pos_traj) -> None:

    mia_j_traj_ctrl = 'mia_hand_hw_vel_trajectory_controller'
    mia_j_vel_ctrl = 'mia_hand_joint_group_vel_controller'
    self.mia_j_index_pos_vel_ctrl = 'mia_hand_j_index_fle_pos_vel_controller'

    mia_ctrl_dict = {mia_j_traj_ctrl : JointTrajectory,
                mia_j_vel_ctrl : Float64MultiArray,
                self.mia_j_index_pos_vel_ctrl : Float64}

    self.mia = MiaHandCommander(ns='mia_hand',ctrl_dict=mia_ctrl_dict)

    self.mia_pos_traj = mia_pos_traj

    rospy.loginfo('Waiting for controllers...')
    sleep(2)

    self.mia.open()
    sleep(1)

    # self.mia.switch_to_controller(mia_j_traj_ctrl)
    self.mia.switch_to_controller(self.mia_j_index_pos_vel_ctrl)

    self.j_index_pos = 0.4

    j_pub_th = Thread(target=self.j_pub_thread)
    j_pub_th.start()

  def test_loop(self):

    loop_rate = 20
    dt = 1/loop_rate
    rate = rospy.Rate(loop_rate)

    goal_time = 0.8
    c_max = floor(goal_time/dt) + 1
    goal_time = c_max*dt

    c_index_pos = self.mia_pos_traj[0][0]
    for c in range(1,c_max+1):
      j_i = harmonic_pos(self.j_index_pos-c_index_pos,c*dt,goal_time)
      self.j_index_pos = c_index_pos + j_i
      self.mia.controller_command(self.j_index_pos)
      rate.sleep()

    sleep(.5)

    c = 6
    while True:
      if c > len(self.mia_pos_traj):
        break
      self.j_index_pos = self.mia_pos_traj[c][0]
      self.mia.controller_command(self.j_index_pos)
      c += 6
      rate.sleep()

    sleep(2)

    c_index_pos = self.mia.pos[0]
    for c in range(1,c_max+1):
      jp_i = harmonic_pos(0.4-c_index_pos,c*dt,goal_time)
      self.j_index_pos = c_index_pos + jp_i
      self.mia.controller_command(self.j_index_pos)
      rate.sleep()
  
  def test_loop_trajectory(self):

    loop_rate = 20
    dt = 1/loop_rate
    rate = rospy.Rate(loop_rate)

    j_traj = JointTrajectory()
    j_traj.joint_names = self.mia.joint_names

    self.j_index_pos = self.mia_pos_traj[0][0]
    self.mia.close([self.j_index_pos,0.1,0.1],autoswitch=False,rest=False,close_time=0.5)

    sleep(1)

    c = 6
    while True:
      if c > len(self.mia_pos_traj):
        break
      j_traj_pnt = JointTrajectoryPoint()
      j_traj_pnt.time_from_start = rospy.Duration.from_sec(c*1/120)
      j_traj_pnt.positions = [self.mia_pos_traj[c][0],0.0,0.0]
      j_traj.points.append(j_traj_pnt)
      c += 6
      rate.sleep()

    def update_j_index_pos(j_traj : JointTrajectory):
      for pnt in j_traj.points:
        self.j_index_pos = pnt.positions[0]
        rate.sleep()

    t = Thread(target = update_j_index_pos,args = [j_traj,])
    t.start()

    self.mia.controller_command(j_traj)

    sleep(2)

    t.join()

    self.j_index_pos = 0.4
    self.mia.close([self.j_index_pos,0.1,0.1],autoswitch=False,rest=False)

  def j_pub_thread(self):

    j_msg = Float64MultiArray()
    j_msg.data.append(0.0)
    j_pub = rospy.Publisher('mia_hand_trajectory_test',Float64MultiArray,queue_size=1000)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
      j_msg.data[0] = self.j_index_pos
      j_pub.publish(j_msg)
      rate.sleep()


def main():

  rospy.init_node('mia_hand_trajectory_test_node')


  with open('/home/penzo/Desktop/2022_r_WK37_insole_spherical_front_98.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    mia_pos_traj = []
    for row in csv_reader:
      if line_count == 0:
        line_count += 1
      else:
        mia_pos_traj.append([float(row[2]),float(row[3]),float(row[1])])

  mia_traj_test = MiaHandTrajectoryTest(mia_pos_traj)

  while not rospy.is_shutdown():
    input('Press Enter to continue >')
    mia_traj_test.test_loop()
    # mia_traj_test.test_loop_trajectory()

  rospy.spin()

if __name__ == '__main__':
  main()