#! /usr/bin/env python3

from artificial_hands_py.robot_commander import *
from artificial_hands_py import get_key

class KeyTeleopServoCommander(ServoCommanderBase):

  j_traj_ctrl = 'pos_joint_traj_controller'
  j_servo_ctrl = 'joint_group_vel_controller'
  c_frame_servo = 'servo_twist_controller'

  def __init__(self, rate: rospy.Rate, ns: str = '') -> None:
    super().__init__(rate,ns,{self.j_traj_ctrl : JointTrajectory, self.j_servo_ctrl : Float64MultiArray},{self.c_frame_servo : TwistStamped})
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
    cmd.header.stamp.nsecs = rospy.Time.now().nsecs + self.rate.sleep_dur.to_nsec()
    self.servo_command(self.c_frame_servo,cmd)

def main():

  input('Press Enter to start key teleop>\n')

  rospy.init_node("key_teleop_servo_node")
  
  rate = rospy.Rate(50)
  servo_cmd = KeyTeleopServoCommander(rate)

  print("\nKey settings:\n + : increase speed feedforward\n - : decrease speed feedforward\n x : control x axis\n y : control y axis\n z : control z axis\n h : move to home position\n")

  servo_ff = 1
  servo_axis = [1,0,0]

  servo_cmd.switch_to_controller(servo_cmd.j_servo_ctrl)

  rate.sleep()
  while not rospy.is_shutdown():

    servo_cmd.key_servo(servo_axis,servo_ff)

    set_key = get_key(rate.remaining().to_sec())
    if set_key == '+':
      if servo_ff < 20:
        servo_ff = servo_ff * 2
        rospy.loginfo("Increased speed feed_forward to %.2f",servo_ff)
    elif set_key == '-':
      if servo_ff > .05:
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
  main()