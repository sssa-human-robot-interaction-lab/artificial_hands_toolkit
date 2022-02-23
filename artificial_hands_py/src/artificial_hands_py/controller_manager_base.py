import rospy
from rqt_controller_manager.controller_manager import *

class ControllerManagerBase:
  """ This base class provides methods to check and load required controllers,
  switch to and command the active one.

  Attributes
  ----------
  sw_ser : rospy.ServiceProxy
    proxy to controller_manager/switch_controller
  ctrl_dict : dict
    dictionaries of command publisher for a given controller
  ctrl : str
    name of the active controller

  Methods
  -------
  switch_to_controller(controller_name)
    make a controller active
  
  controller_command(cmd)
    forward command to the active controller
  """

  def __init__(self,ns : str, ctrl_dict : dict) -> None:  
    ls_ser = rospy.ServiceProxy(ns+'/controller_manager/list_controllers',ListControllers)
    ld_ser = rospy.ServiceProxy(ns+'/controller_manager/load_controller',LoadController)
    self.sw_ser = rospy.ServiceProxy(ns+'/controller_manager/switch_controller',SwitchController) 
    self.ctrl_dict = {}
    self.ctrl = ''
    
    ls_ctrl = ls_ser()
    for y in ctrl_dict.keys():
      self.ctrl_dict[y] = rospy.Publisher(ns+'/'+y+'/command',ctrl_dict[y],queue_size=1000)
      c = False
      for x in ls_ctrl.controller:
        if x.name == y:
          c = True
          break
      if not c:
        ld_ser(y)

  def switch_to_controller(self,start_ctrl : str) -> None:
    self.ctrl = start_ctrl
    stop_ctrl = self.ctrl_dict.keys() - start_ctrl
    return self.sw_ser([start_ctrl],stop_ctrl,1,False,5).ok
  
  def controller_command(self,cmd) -> None:
    self.ctrl_dict[self.ctrl].publish(cmd)