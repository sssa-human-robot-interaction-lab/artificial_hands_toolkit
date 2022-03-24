from threading import Thread
from time import sleep
from artificial_hands_py.robot_commander import *
from artificial_hands_py import get_key

from dmp_extended.msg import DesiredTrajectory, MJerkTrackTarget
from geometry_msgs.msg import QuaternionStamped, Transform
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint

class DMPExtendedCommander(HarmonicServoCommander):

  j_traj_ctrl = 'pos_joint_traj_controller'
  j_servo_ctrl = 'joint_group_vel_controller'
  c_twist_servo = 'servo_twist_controller'
  c_traj_ctrl = 'cartesian_eik_position_controller'

  def __init__(self, rate: rospy.Rate, ns: str = '', ref: str = '', eef: str = ''):
    super().__init__(rate, ns, ref, eef, {self.j_traj_ctrl : JointTrajectory, self.c_traj_ctrl : MultiDOFJointTrajectoryPoint, self.j_servo_ctrl : Float64MultiArray},{self.c_twist_servo : TwistStamped})
    self.dmp_traj = DesiredTrajectory()
    self.dmp_target = MJerkTrackTarget()
    self.dmp_orientation = QuaternionStamped()
    self.dmp_follow = False    
    
    dmp_traj_sub = rospy.Subscriber("/target_traj",DesiredTrajectory,self.target_traj_callback)
    dmp_servo_sub = rospy.Subscriber("/dmp_extended_servo_node/command",MJerkTrackTarget,self.servo_target_callback)
    self.dmp_pub = rospy.Publisher("/dmp/target_position_state",MJerkTrackTarget,queue_size=1000)
    self.dmp_rot = rospy.Publisher("/dmp/target_orientation",QuaternionStamped,queue_size=1000)

  def target_traj_callback(self, msg : DesiredTrajectory):
    self.dmp_traj = msg
  
  def servo_target_callback(self, msg : MJerkTrackTarget):
    self.dmp_target = msg

  def dmp_servo(self):
    cmd = TwistStamped()
    cmd.header = self.dmp_traj.header
    cmd.header.frame_id = self.reference_frame
    cmd.twist = self.dmp_traj.twist
    self.servo_command(self.c_twist_servo,cmd)
  
  def dmp_command(self):
    cmd = MultiDOFJointTrajectoryPoint()
    cmd_ = Transform()
    cmd_.translation = self.dmp_traj.pose.position
    cmd_.rotation = self.dmp_traj.pose.orientation
    cmd_dot = self.dmp_traj.twist
    cmd.transforms.append(cmd_)
    cmd.velocities.append(cmd_dot)
    self.controller_command(cmd)
  
  def dmp_set_rot(self):
    self.dmp_rot.publish(self.dmp_orientation)

  def dmp_set_target(self):
    while True:
      self.dmp_pub.publish(self.dmp_target)
      self.rate.sleep()
    
  def dmp_servo_follow(self):
    self.cmd_thread_stop = False
    while not self.cmd_thread_stop:
      if self.dmp_follow:
        self.dmp_servo()
      self.rate.sleep()
  
  def dmp_command_follow(self):
    self.cmd_thread_stop = False
    while not self.cmd_thread_stop:
      if self.dmp_follow:
        self.dmp_command()
      self.rate.sleep()

def main():

  input('< Press Enter to start dmp servoing >\n')

  rospy.init_node("dmp_extended_servo_node")

  set_key = ''
  
  servo_rate = rospy.Rate(500)
  servo_cmd = DMPExtendedCommander(servo_rate,ref='world',eef='ft_sensor_frame')

  print("\nKey settings:\n h : move to home position\n r : read current ee pose\n s : switch to servo twist control\n t : switch to cartesian trajectory control\n f : start to follow dmp generated target\n c : update dmp target to current ee pose\n 1-9: set a predefined dmp target" )

  dmp_thread = Thread(target = servo_cmd.dmp_set_target)
  dmp_thread.start()

  cmd_thread = Thread(target = servo_cmd.dmp_servo_follow)
  cmd_thread.start()
  cmd_ctrl = servo_cmd.j_servo_ctrl

  target = {'1' : [0.5,0.1,1.0], '2' : [0.5,0.1,0.4], '3' : [0.7,0.1,0.4], '4' : [0.1,0.1,0.4]}

  rate = rospy.Rate(1)
  while not rospy.is_shutdown():
      
    if set_key == 'h':
      rospy.loginfo("Moving to home position (stop to follow dmp)")
      servo_cmd.dmp_follow = False
      servo_cmd.switch_to_controller(servo_cmd.j_traj_ctrl)
      servo_cmd.go([.0,-pi/2,pi/2,.0,pi/2,.0],wait=True)
      servo_cmd.switch_to_controller(cmd_ctrl)
    elif set_key == 's':
      rospy.loginfo("Switching to servo control")
      servo_cmd.cmd_thread_stop = True
      sleep(.1)
      cmd_thread.join()
      del cmd_thread
      cmd_ctrl = servo_cmd.j_servo_ctrl
      servo_cmd.switch_to_controller(cmd_ctrl)
      cmd_thread = Thread(target = servo_cmd.dmp_servo_follow)
      cmd_thread.start()
    elif set_key == 't':
      rospy.loginfo("Switching to traj control")
      servo_cmd.cmd_thread_stop = True
      sleep(.1)
      cmd_thread.join()
      del cmd_thread
      cmd_ctrl = servo_cmd.c_traj_ctrl
      servo_cmd.switch_to_controller(cmd_ctrl)
      cmd_thread = Thread(target = servo_cmd.dmp_command_follow)
      cmd_thread.start()
    elif set_key == 'f':
      servo_cmd.dmp_follow = not servo_cmd.dmp_follow
      rospy.loginfo("Follow dmp generated trajectory: %s",servo_cmd.dmp_follow)
    elif set_key == 'c':
      rospy.loginfo("Updating dmp target to current eef position")
      eef_pose = servo_cmd.get_eef_frame()
      servo_cmd.dmp_target.header = eef_pose.header
      servo_cmd.dmp_target.pos = eef_pose.pose.position
      servo_cmd.dmp_orientation.quaternion = eef_pose.pose.orientation
      servo_cmd.dmp_set_rot()
    elif set_key == 'r':
      rospy.loginfo("Current eef pose")
      print(servo_cmd.get_eef_frame())
    elif set_key in ['1','2','3','4','5']:
      try:
        rospy.loginfo("Setting dmp target %s", target[set_key])
      except:
        rospy.loginfo("Target %i not defined", int(set_key))
      servo_cmd.dmp_target.pos.x = target[set_key][0]
      servo_cmd.dmp_target.pos.y = target[set_key][1]
      servo_cmd.dmp_target.pos.z = target[set_key][2]

    if rate.remaining().to_sec() > 0:
      set_key = get_key(rate.remaining().to_sec())  
    else:
      set_key = ''
    
    if (set_key == '\x03'):
        rospy.signal_shutdown('bye!')
        break   
    else:
      rate.sleep()
  
if __name__ == '__main__':
  main()
  
