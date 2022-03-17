from artificial_hands_py_base import BaseFilter, WristFTDetection

from geometry_msgs.msg import Quaternion

import sys, select, termios, tty

key_settings = termios.tcgetattr(sys.stdin)

def get_key(key_timeout):
  tty.setraw(sys.stdin.fileno())
  rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
  if rlist:
      key = sys.stdin.read(1)
  else:
      key = ''
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, key_settings)
  return key

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
