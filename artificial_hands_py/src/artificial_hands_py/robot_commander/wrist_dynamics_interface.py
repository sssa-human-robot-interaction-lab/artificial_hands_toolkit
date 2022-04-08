import rospy

from artificial_hands_msgs.srv import WristDynamicsCommand

class WristDynamicsInterface:

  def __init__(self,ns:str = '') -> None:
    self.ns = ns
    
  def wrist_dynamics_command(self,command) -> bool:
    res = rospy.ServiceProxy(self.ns+'/wrist_dynamics_command/'+command,WristDynamicsCommand)
    return res().success
  
  def wrist_dynamics_mode(self,mode) -> bool:
    res = rospy.ServiceProxy(self.ns+'/wrist_dynamics_mode/'+mode,WristDynamicsCommand)
    return res().success
  
  def wrist_dynamics_macro(self,macro) -> bool:
    res = rospy.ServiceProxy(self.ns+'/wrist_dynamics_macro/'+macro,WristDynamicsCommand)
    return res().success