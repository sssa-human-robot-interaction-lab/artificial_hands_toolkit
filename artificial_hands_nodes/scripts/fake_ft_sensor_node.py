#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import WrenchStamped

def main():
  rospy.init_node('fake_ft_sensor_node')

  ft = WrenchStamped()
  ft.header.frame_id = 'ft_sensor_frame'
  ft.wrench.force.x = 0
  ft.wrench.force.y = 0
  ft.wrench.force.z = 0
  ft.wrench.torque.x = 0
  ft.wrench.torque.y = 0
  ft.wrench.torque.z = 0

  pub = rospy.Publisher('ft_sensor',WrenchStamped,queue_size=1000)

  rate = rospy.Rate(200)

  rospy.loginfo('Starting fake ft_sensor...')

  while not rospy.is_shutdown():
    pub.publish(ft)
    rate.sleep()
  
  rospy.loginfo('bye!')

if __name__ == '__main__':
  main()
