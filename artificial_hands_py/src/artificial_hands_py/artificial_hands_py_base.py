from multipledispatch import dispatch

import numpy as np

import tf.transformations as ts
import moveit_commander.conversions as cv

from cartesian_control_msgs.msg import CartesianTrajectoryPoint
from geometry_msgs.msg import Quaternion, Pose, Twist, Transform

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

def pose_to_list(pose : Pose):
  return cv.pose_to_list(pose)

def list_to_pose(pose : list):
  return cv.list_to_pose(pose)

def list_to_twist(twist : list):
  t = Twist()
  t.linear.x = twist[0]
  t.linear.y = twist[1]
  t.linear.z = twist[2]
  t.angular.x = twist[3]
  t.angular.y = twist[4]
  t.angular.z = twist[5]
  return t

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

def twist_copy(twist : Twist) -> Twist:
  t = Twist()
  t.linear.x = twist.linear.x
  t.linear.y = twist.linear.y
  t.linear.z = twist.linear.z
  t.angular.x = twist.angular.x
  t.angular.y = twist.angular.y
  t.angular.z = twist.angular.z
  return t

def twist_rotate(twist : Twist, M : np.ndarray) -> Twist:
  t = Twist()
  t_linear = np.dot(M,np.array([twist.linear.x,twist.linear.y,twist.linear.z]).transpose())
  t_angular = np.dot(M,np.array([twist.angular.x,twist.angular.y,twist.angular.z]).transpose())
  t.linear.x = t_linear[0]
  t.linear.y = t_linear[1]
  t.linear.z = t_linear[2]
  t.angular.x = t_angular[0]
  t.angular.y = t_angular[1]
  t.angular.z = t_angular[2]
  return t

@dispatch(CartesianTrajectoryPoint, CartesianTrajectoryPoint)
def cart_traj_point_copy(trg : CartesianTrajectoryPoint, pnt : CartesianTrajectoryPoint) -> Pose:
  trg.pose = pose_copy(pnt.pose)
  trg.twist = twist_copy(pnt.twist)
  trg.acceleration = twist_copy(pnt.acceleration)

@dispatch(CartesianTrajectoryPoint)
def cart_traj_point_copy(pnt : CartesianTrajectoryPoint) -> Pose:
  p = CartesianTrajectoryPoint()
  p.pose = pose_copy(pnt.pose)
  p.twist = twist_copy(pnt.twist)
  p.acceleration = twist_copy(pnt.acceleration)
  return p

def pose_to_matrix(pose: Pose) -> np.matrix:
  T = ts.quaternion_matrix(quat_to_list(pose.orientation))
  T[0,3] = pose.position.x
  T[1,3] = pose.position.y
  T[2,3] = pose.position.z
  return T

def matrix_to_pose(T: np.matrix) -> Pose:
  p = Pose()
  p.position.x = T[0,3]
  p.position.y = T[1,3]
  p.position.z = T[2,3]
  p.orientation = list_to_quat(ts.quaternion_from_matrix(T))
  return p

def transform_to_matrix(tf : Transform) -> np.matrix:
  return pose_to_matrix(list_to_pose(cv.transform_to_list(tf)))
