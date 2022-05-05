import numpy as np

from geometry_msgs.msg import Quaternion,Pose

def singleton(cls, *args, **kw):
     instances = {}
     def _singleton(*args, **kw):
        if cls not in instances:
             instances[cls] = cls(*args, **kw)
        return instances[cls]
     return _singleton
     
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

def norm_quat(quat : Quaternion):
  q = quat_to_list(quat)
  q_norm = np.linalg.norm(q)
  return list_to_quat([q_/q_norm for q_ in q])

def pose_copy(pose : Pose) -> Pose:
  p = Pose()
  p.position.x = pose.position.x
  p.position.y = pose.position.y
  p.position.z = pose.position.z
  p.orientation.x = pose.orientation.x
  p.orientation.y = pose.orientation.y
  p.orientation.z = pose.orientation.z
  p.orientation.w = pose.orientation.w
  return p
