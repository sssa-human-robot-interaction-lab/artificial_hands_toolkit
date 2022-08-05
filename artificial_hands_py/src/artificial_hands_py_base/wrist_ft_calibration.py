from artificial_hands_py import pyatk

class WristFTCalibration(pyatk.pyWristFTCalibration):

  def __init__(self) -> None:
    pass
  
  def update(self,ft : list, g : list):
    self.c_update(ft)

  def get(self):
    return self.c_get()