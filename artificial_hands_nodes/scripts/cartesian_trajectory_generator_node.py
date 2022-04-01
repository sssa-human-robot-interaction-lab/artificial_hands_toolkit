#!/usr/bin/env python3

import rospy

from artificial_hands_py.cartesian_trajectory_generator import CartesianTrajectoryGenerator

def main():

  rospy.init_node('trajectory_generator_node')

  cart_traj_gen = CartesianTrajectoryGenerator()

  rospy.spin()

if __name__ == '__main__':
  main()