from artificial_hands_py import pyatk

class BaseFilter(pyatk.pyBaseFilter):

  def __init__(self,len : int = 20) -> None:
      super().__init__(len)
  
  def init(self,f : list):
    self.c_init(f)
  
  def update(self,f : list):
    self.c_update(f)
  
  def get(self) -> list:
    return self.c_get()