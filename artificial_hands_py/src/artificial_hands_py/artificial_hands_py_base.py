import rospy

from geometry_msgs.msg import Quaternion

from artificial_hands_msgs.srv import WristDynamicsCommand

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
