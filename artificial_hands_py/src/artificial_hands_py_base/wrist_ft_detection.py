from artificial_hands_py import pyatk

class WristFTContactDetection(pyatk.pyWristFTContactDetection):

  def __init__(self,buf : int = 50, th : int = 20, rto : float = 0.5, dev : float =0.4, factor : float = 2.0) -> None:
    super().__init__(buf,th,rto,dev,factor)

  def init(self,ft : list):
    self.c_init(ft)
  
  def update(self,ft : list):
    self.c_update(ft)

  def get(self):
    return self.c_get()
  
  def set_zero(self,do_zero : bool):
    self.c_set_zero(do_zero)

  def pre_trigger_static(self) -> bool:
    return self.c_pre_trigger_static()
  
  def trigger_static(self):
    self.c_trigger_static()
  
  def backup_trigger_static(self):
    self.c_backup_trigger_static()
  
  def get_d_fi_max(self) -> float:
    return self.c_get_d_fi_max()
  
  def get_pretrig(self) -> bool:
    return self.c_pretrig

  def get_trigger(self) -> bool:
    return self.c_trigger
  
  def get_backtrig(self) -> bool:
    return self.c_backtrig
  
  def reset_trigger(self):
    self.c_trigger = False
  