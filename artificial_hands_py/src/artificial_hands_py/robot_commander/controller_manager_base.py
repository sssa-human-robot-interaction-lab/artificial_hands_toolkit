import rospy

from rqt_controller_manager.controller_manager import *

class ControllerManagerBase:
  """ This base class provides methods to check and load required controllers,
  switch to or pause the desired controller.

  Attributes
  ----------
  sw_ser : rospy.ServiceProxy
    proxy to rosservice controller_manager/switch_controller
  ctrl_dict : dict
    dictionaries of command publishers for given controllers
  ctrl : str
    name of the active controller

  Methods
  -------
  switch_to_controller
    make a controller active

  pause_controller
    pause the active controller active

  unpause_controller
    unpause the last active controller

  pause_all_controllers
    pause any active controller
  
  controller_command
    send command to the active controller
  """

  def __init__(self,ns : str, ctrl_dict : dict) -> None:  
    ls_ser = rospy.ServiceProxy(ns+'/controller_manager/list_controllers',ListControllers)
    ld_ser = rospy.ServiceProxy(ns+'/controller_manager/load_controller',LoadController)
    self.sw_ser = rospy.ServiceProxy(ns+'/controller_manager/switch_controller',SwitchController) 
    self.ctrl_dict = {}
    self.ctrl = ''

    # for y in ctrl_dict.keys():
    #   self.ctrl_dict[rospy.remap_name(y).replace('/','',1)] = ctrl_dict[y]

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
  
  def pause_controller(self) -> None:
    return self.sw_ser([],[self.ctrl],1,False,5).ok
  
  def unpause_controller(self) -> None:
    return self.sw_ser([self.ctrl],[],1,False,5).ok

  def pause_all_controllers(self) -> None:
    return self.sw_ser([],list(self.ctrl_dict.keys()),1,False,5).ok
  
  def controller_command(self,cmd) -> None:
    self.ctrl_dict[self.ctrl].publish(cmd)
