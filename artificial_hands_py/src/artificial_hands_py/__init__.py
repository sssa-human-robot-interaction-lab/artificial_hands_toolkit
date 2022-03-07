from artificial_hands_py_base import BaseFilter, WristFTDetection

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