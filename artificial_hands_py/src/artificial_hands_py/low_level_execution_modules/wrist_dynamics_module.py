from abc import ABC

import rospy

from artificial_hands_msgs.msg import DetectionStamped
from artificial_hands_msgs.srv import WristDynamicsCommand

class WristDynamicsModule(ABC):

  def __init__(self) -> None:
    super().__init__()
    
  def wrist_dynamics_command(self,command) -> bool:
    res = rospy.ServiceProxy('wrist_dynamics_command/'+command,WristDynamicsCommand)
    return res().success
  
  def wrist_dynamics_mode(self,mode) -> bool:
    res = rospy.ServiceProxy('wrist_dynamics_mode/'+mode,WristDynamicsCommand)
    return res().success
  
  def wrist_dynamics_macro(self,macro) -> bool:
    res = rospy.ServiceProxy('wrist_dynamics_macro/'+macro,WristDynamicsCommand)
    return res().success
  
  def subscribe(self) -> bool:
    return self.wrist_dynamics_command('subscribe')
  
  def start_loop(self) -> bool:
    return self.wrist_dynamics_command('start_loop')
  
  def build_model(self) -> bool:
    return self.wrist_dynamics_command('build_model')
  
  def stop_loop(self) -> bool:
    return self.wrist_dynamics_command('stop_loop')
  
  def read_loop_time(self) -> bool:
    return self.wrist_dynamics_command('read_loop_time')
  
  def do_zero(self) -> bool:
    return self.wrist_dynamics_command('set_zero')
  
  def estimate_calibration(self) -> bool:
    return self.wrist_dynamics_command('estimate_calibration')
  
  def apply_calibration(self) -> bool:
    return self.wrist_dynamics_command('set_calibration')

  def check_calibration(self) -> bool:
    return self.wrist_dynamics_command('check_calibration')
  
  def set_publish(self) -> bool:
    return self.wrist_dynamics_mode('publish')
  
  def set_trigger_static(self) -> bool:
    return self.wrist_dynamics_mode('trigger_static')
  
  def set_estimate_wrench(self) -> bool:
    return self.wrist_dynamics_mode('estimate_wrench')
  
  def set_save_dynamics(self) -> bool:
    return self.wrist_dynamics_mode('save_dynamics')
  
  def set_save_interaction(self) -> bool:
    return self.wrist_dynamics_mode('save_interaction')
  
  def set_trigger_dynamics(self) -> bool:
    return self.wrist_dynamics_mode('trigger_dynamics')
  
  def set_save_calibration(self) -> bool:
    return self.wrist_dynamics_mode('save_calibration')
  
  def start_node(self, calib = False) -> bool:
    self.wrist_dynamics_macro('start_node')
    if calib:
      return self.apply_calibration()
    return True