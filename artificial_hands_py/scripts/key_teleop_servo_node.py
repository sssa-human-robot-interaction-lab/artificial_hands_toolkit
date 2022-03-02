#! /usr/bin/env python3

import rospy
import sys, select, termios, tty
from cmath import pi

from artificial_hands_py.robot_commander import HarmonicServoCommander
from geometry_msgs.msg import Twist,TwistStamped

class KeyTeleopServoCommander(HarmonicServoCommander):

  def __init__(self,ns='') -> None:
    super().__init__(ns)
    self.lin = 0
    self.ang = 0
    key_sub = rospy.Subscriber("/key_vel",Twist,self.key_callback)

  def key_callback(self,msg : Twist):
    self.lin = msg.linear.x
    self.ang = msg.angular.z

  def key_servo(self, servo_axis : list, servo_ff : float = 1):
    cmd = TwistStamped()
    cmd.twist.linear.x = self.lin*servo_ff*servo_axis[0]
    cmd.twist.linear.y = self.lin*servo_ff*servo_axis[1]
    cmd.twist.linear.z = self.lin*servo_ff*servo_axis[2]
    cmd.twist.angular.x = self.ang*servo_ff*servo_axis[0]
    cmd.twist.angular.y = self.ang*servo_ff*servo_axis[1]
    cmd.twist.angular.z = self.ang*servo_ff*servo_axis[2]
    cmd.header.stamp.secs = rospy.Time.now().secs
    cmd.header.stamp.nsecs = rospy.Time.now().nsecs

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():

  input('Press Enter to start key teleop>\n')

  rospy.init_node("key_teleop_servo_node")
  
  servo_cmd = KeyTeleopServoCommander()

  print("\nKey settings:\n + : increase speed feedforward\n - : decrease speed feedforward\n x : control x axis\n y : control y axis\n z : control z axis\n h : move to home position\n")

  servo_ff = 1
  servo_axis = [1,0,0]

  servo_cmd.switch_to_controller(servo_cmd.j_servo_ctrl)

  rate = rospy.Rate(50)
  while not rospy.is_shutdown():

    servo_cmd.key_servo(servo_axis,servo_ff)

    set_key = getKey(rate.remaining().to_sec())
    if set_key == '+':
      servo_ff = servo_ff * 1.5
      rospy.loginfo("Increased speed feed_forward to %.2f",servo_ff)
    elif set_key == '-':
      servo_ff = servo_ff * 0.5
      rospy.loginfo("Decreased speed feed_forward to %.2f",servo_ff)
    elif set_key in ['x','y','z']:
      servo_axis = [0,0,0]
      servo_axis[['x','y','z'].index(set_key)] = 1
      rospy.loginfo("Switched to control axis %s",set_key)
    elif set_key == 'h':
      rospy.loginfo("Moving to home position")
      servo_cmd.switch_to_controller(servo_cmd.j_traj_ctrl)
      servo_cmd.go([.0,-pi/2,pi/2,.0,pi/2,.0],wait=True)
      servo_cmd.switch_to_controller(servo_cmd.j_servo_ctrl)
    else:
      if (set_key == '\x03'):
        break

    rate.sleep()
  
if __name__ == '__main__':
  settings = termios.tcgetattr(sys.stdin)
  main()