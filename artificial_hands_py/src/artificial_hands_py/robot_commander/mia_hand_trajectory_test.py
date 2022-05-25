from math import floor
from time import sleep
from threading import Thread
import csv

import rospy

from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint

from artificial_hands_py.cartesian_trajectory_generator.cartesian_trajectory_base import harmonic_pos
from artificial_hands_py.robot_commander.mia_hand_commander import MiaHandCommander

class MiaHandTrajectoryTest:

  def __init__(self,mia_pos_traj) -> None:
    
    mia_j_traj_ctrl = 'mia_hand_hw_vel_trajectory_controller'
    mia_j_vel_ctrl = 'mia_hand_joint_group_vel_controller'

    mia_ctrl_dict = {mia_j_traj_ctrl : JointTrajectory,
                mia_j_vel_ctrl : Float64MultiArray}

    mia = MiaHandCommander(ns='mia_hand',ctrl_dict=mia_ctrl_dict)

    rospy.loginfo('Waiting for ritardo...')
    sleep(2)

    # self.mia_pos = [0.4,0.0,0.0]
    self.mia_pos = mia_pos_traj[0]

    j_pub_th = Thread(target=self.j_pub_thread)
    j_pub_th.start()
    
    mia.open()
    mia.close(self.mia_pos)

    mia.switch_to_controller(mia_j_traj_ctrl)

    j_traj_pnt = JointTrajectoryPoint()
    j_traj_pnt.time_from_start = rospy.Duration.from_sec(0.1)
    j_traj = JointTrajectory()
    j_traj.joint_names = mia.joint_names
    j_traj.points.append(j_traj_pnt)

    loop_rate = 20
    dt = 1/loop_rate
    rate = rospy.Rate(loop_rate)

    goal_time = 0.8
    c_max = floor(goal_time/dt) + 1
    goal_time = c_max*dt

    rospy.loginfo('Ready!')

    c_pos = mia.pos.copy()
    for c in range(1,c_max+1):
      self.mia_pos = c_pos.copy()
      j_i = harmonic_pos(0.6,c*dt,goal_time)
      j_m = harmonic_pos(0.45,c*dt,goal_time)
      j_t = harmonic_pos(0.5,c*dt,goal_time)
      self.mia_pos[0] += j_i
      self.mia_pos[1] += j_m
      self.mia_pos[2] += j_t
      j_traj_pnt.positions = self.mia_pos
      j_traj.header.stamp = rospy.Time.now()
      mia.controller_command(j_traj)
      rate.sleep()

    # for pos in mia_pos_traj:
    #   self.mia_pos = pos
    #   j_traj_pnt.positions = self.mia_pos
    #   j_traj.header.stamp = rospy.Time.now()
    #   mia.controller_command(j_traj)
    #   rate.sleep()
  
  def j_pub_thread(self):

    j_msg = Float64MultiArray()
    j_pub = rospy.Publisher('mia_hand_trajectory_test',Float64MultiArray,queue_size=1000)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
      j_msg.data = self.mia_pos
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

  rospy.spin()

if __name__ == '__main__':
  main()