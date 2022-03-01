import rospy
import numpy as np
from time import sleep

from geometry_msgs.msg import WrenchStamped, Wrench
from artificial_hands_msgs.msg import DetectionStamped
from artificial_hands_py.mia_hand_commander import MiaHandCommander
from artificial_hands_py.pyatk import BaseFilter, WristFTDetection

def wrench_to_list(wrench : Wrench):
    wrench_list = [.0,.0,.0,.0,.0,.0]
    wrench_list[0] = wrench.force.x
    wrench_list[1] = wrench.force.y
    wrench_list[2] = wrench.force.z
    wrench_list[3] = wrench.torque.x
    wrench_list[4] = wrench.torque.y
    wrench_list[5] = wrench.torque.z
    return wrench_list

class HandoverReleaseNode:
  filter = BaseFilter(20)
  detection = WristFTDetection(50,20,.5,.4,2)
  hand = MiaHandCommander('')
  
  def __init__(self):
    self.d_fi = []
    self.f_mag = .0
    ft = rospy.wait_for_message("/ft_sensor",WrenchStamped)
    self.detection.init(wrench_to_list(ft.wrench))
    self.filter.init(wrench_to_list(ft.wrench))
    ft_sub = rospy.Subscriber("/ft_sensor",WrenchStamped,self.ft_data_callback)
    
  def ft_data_callback(self,msg : WrenchStamped):
    self.detection.update(wrench_to_list(msg.wrench))
    self.filter.update(wrench_to_list(msg.wrench))
  
  def update_d_fi(self):
    fx,fy,fz = self.filter.get()[0:3]
    f_mag = np.linalg.norm(np.array(self.filter.get()[0:3]))
    if len(self.d_fi) == 50:
      self.d_fi = self.d_fi[1:]
    self.d_fi.append((f_mag - self.f_mag)*100)
    self.f_mag = f_mag
    
  def get_d_fi_max(self):
    return max(abs(np.array(self.d_fi)))

def main():

  rospy.init_node("mia_hand_release_node")

  pub = rospy.Publisher('detection',DetectionStamped,queue_size=1000)
  msg = DetectionStamped()

  hrn = HandoverReleaseNode()

  print("Initializing IIR filter...")
  sleep(2)
  hrn.detection.get()
  hrn.detection.set_zero(True)
  print("Ready.")

  hrn.hand.switch_to_controller(hrn.hand.vel_ctrl)
  hrn.hand.open()

  rate = rospy.Rate(100)

  while True:

    hrn.detection.get()
    hrn.update_d_fi()

    hrn.detection.pre_trigger_static()
    if hrn.detection.get_pretrig():
      sleep(.05)
      hrn.detection.trigger_static()

    hrn.detection.backup_trigger_static()
    if hrn.detection.get_trigger() or hrn.detection.get_backtrig():
      # print(hrn.detection.get_d_fi_max())
      print(hrn.get_d_fi_max())
      hrn.hand.open(3)
      hrn.detection.reset_trigger()
      
      if input("Press enter to grasp ('e' to exit) > ") == 'e':
        break
      else:
        hrn.hand.switch_to_controller(hrn.hand.traj_ctrl)
        hrn.hand.close_cyl()
        
        print("Restoring IIR filter...")
        hrn.detection.set_zero(False)
        hrn.d_fi = []
        sleep(2)
        hrn.detection.get()
        hrn.detection.set_zero(True)
        sleep(1)
        hrn.hand.switch_to_controller(hrn.hand.vel_ctrl)
        print("Ready.")
        
    msg.detection.backtrig = hrn.detection.get_backtrig()
    msg.detection.pretrig = hrn.detection.get_pretrig()
    msg.detection.trigger = hrn.detection.get_trigger()
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)

    rate.sleep()

if __name__ == '__main__':
  main()