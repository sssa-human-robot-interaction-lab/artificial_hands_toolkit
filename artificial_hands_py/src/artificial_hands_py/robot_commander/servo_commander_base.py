import rospy
import tf2_ros, tf2_geometry_msgs
import moveit_commander, moveit_commander.conversions as cv
from rqt_controller_manager.controller_manager import *
from geometry_msgs.msg import PoseStamped, Quaternion

def list_to_quat(q : list) -> Quaternion: 
  quat = Quaternion()
  quat.x = q[0]
  quat.y = q[1]
  quat.z = q[2]
  quat.w = q[3]
  return quat

def quat_to_list(quat : Quaternion) -> list: 
  q = []
  q.append(quat.x)
  q.append(quat.y)
  q.append(quat.z)
  q.append(quat.w)
  return q
class ControllerManagerBase:
  """ This base class provides methods to check and load required controllers,
  switch to and command the active one.

  Attributes
  ----------
  sw_ser : rospy.ServiceProxy
    proxy to controller_manager/switch_controller
  ctrl_dict : dict
    dictionaries of command publisher for a given controller
  servo_dict : dict
    dictionaries of command publisher for a given servo topic
  ctrl : str
    name of the active controller

  Methods
  -------
  switch_to_controller(controller_name)
    make a controller active
  
  controller_command(cmd)
    forward command to the active controller

  servo_command(servo_name. cmd)
    forward command to a given servo topic
  """

  def __init__(self,ns : str, ctrl_dict : dict, servo_dict : dict = None) -> None:  
    ls_ser = rospy.ServiceProxy(ns+'/controller_manager/list_controllers',ListControllers)
    ld_ser = rospy.ServiceProxy(ns+'/controller_manager/load_controller',LoadController)
    self.sw_ser = rospy.ServiceProxy(ns+'/controller_manager/switch_controller',SwitchController) 
    self.ctrl_dict = {}
    self.servo_dict ={}
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

    if servo_dict is not None:
      for y in servo_dict.keys():
        self.servo_dict[y] = rospy.Publisher(ns+'/'+y+'/command',servo_dict[y],queue_size=1000)

  def switch_to_controller(self,start_ctrl : str) -> None:
    self.ctrl = start_ctrl
    stop_ctrl = self.ctrl_dict.keys() - start_ctrl
    return self.sw_ser([start_ctrl],stop_ctrl,1,False,5).ok
  
  def controller_command(self,cmd) -> None:
    self.ctrl_dict[self.ctrl].publish(cmd)
  
  def servo_command(self,servo,cmd) -> None:
    self.servo_dict[servo].publish(cmd)

class ServoCommanderBase(ControllerManagerBase,moveit_commander.MoveGroupCommander):

  def __init__(self, ns: str, ref: str, eef: str, ctrl_dict: dict, servo_dict: dict = None, move_group: str = 'manipulator') -> None:
    super().__init__(ns, ctrl_dict, servo_dict)
    super(ControllerManagerBase,self).__init__(move_group) 
    self.reference_frame = ref
    self.ee_frame = eef
    self.paused = False
    self.preempted = False
    self.set_pose_reference_frame(ref)
    self.set_end_effector_link(eef)
  
  def set_paused(self):
    self.paused = True
  
  def set_preempted(self):
    self.preempted = True
    self.paused = False
  
  def set_rate(self, rate : rospy.Rate) -> None:
    self.rate = rate
    self.dt = self.rate.sleep_dur.to_sec()
    self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
    tf_listener = tf2_ros.TransformListener(self.tf_buffer)
  
  def get_eef_frame(self,ref : str = None, frame : str = None) -> PoseStamped:
    if frame is None:
      frame = self.ee_frame
    if ref is None:
      ref = self.reference_frame
    pose_ref : PoseStamped = self.get_current_pose(frame)
    ref_to_base = self.tf_buffer.lookup_transform(ref,pose_ref.header.frame_id,rospy.Time.now(),timeout=rospy.Duration(1))
    return tf2_geometry_msgs.do_transform_pose(pose_ref,ref_to_base)