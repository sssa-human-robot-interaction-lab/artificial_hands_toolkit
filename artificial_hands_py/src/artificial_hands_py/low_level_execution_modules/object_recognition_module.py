import numpy as np
from cmath import pi
from threading import Lock

import rospy, actionlib
import tf, tf.transformations as ts
from cartesian_control_msgs.msg import CartesianTrajectoryPoint

from artificial_hands_msgs.msg import *
from artificial_hands_py.robot_commander.robot_commander import RobotCommander
from artificial_hands_py.artificial_hands_py_base import matrix_to_pose, pose_to_matrix

class ObjectRecognitionModule(RobotCommander):
  sleep_dur = rospy.Duration(0.5)
  recog_feedback = ObjectRecognitionFeedback()
  recog_result = ObjectRecognitionResult()

  lock = Lock()

  def __init__(self) -> None:
    super().__init__()
    
    self.recog_as = actionlib.SimpleActionServer('/object_recognition',ObjectRecognitionAction,execute_cb=self.recognition_cb,auto_start=False)

    self.recog_as.start()

  def recognition_cb(self, goal : ObjectRecognitionGoal):
    self.recog_result.success = True
    self.recog_feedback.percentage = 100

    # initialize wrist_dynamics_module making use of previous calibration (assumed valid)
    self.wrist_dyn.start_node(calib=True)
    self.wrist_dyn.set_publish()
    
    # go to home position
    self.arm.set_max_accel(goal.max_accel)
    self.arm.set_max_angaccel(goal.max_angaccel)
    self.arm.set_harmonic_traj_generator()
    self.arm.switch_to_cartesian_controller('cartesian_eik_position_controller')
    self.arm.set_pose_target(goal.home)

    # start to fill wrist_dynamics matrix
    self.wrist_dyn.set_save_dynamics()
    rospy.sleep(self.sleep_dur)

    # perform estimate trajectory
    self.estimate_trajectory()

    # get wrist_dynamics_module to idle and build the object inertial model
    self.wrist_dyn.stop_loop()
    self.wrist_dyn.build_model()

    # stop controllers
    self.arm.wait_for_trajectory_monitor()
    self.arm.pause_all_controllers()
    self.recog_as.set_succeeded(self.recog_result)
  
  def estimate_trajectory(self):

    def eepose(Q, t, n, h, dt, Ts):

      yd = np.zeros((6,1))
      ydd = np.zeros((6,1))

      # harmonic composition
      for k in range(6):
        for c in range(1,n[k]+1):
          yd[k] += (h[k]/c)/Ts*(1 - np.cos(2*pi*c*t/Ts))
          ydd[k] += 2*pi*(h[k]/c)/(Ts*Ts)*np.sin(2*pi*c*t/Ts)

      if t > Ts:
        yd *= -1
        ydd *= -1

      rot = Q[0:3,0:3]

      y = yd*dt + 0.5*ydd*dt*dt

      P = np.zeros((4,4))
      P[0:3,0:3] = np.dot(rot,ts.euler_matrix(y[3],y[4],y[5])[0:3,0:3])
      P[0:3,3] = Q[0:3,3] + np.transpose(np.dot(rot[0:3,0:3],y[0:3]))
      P[3,3] = 1

      yd += ydd*dt

      yd[0:3] = np.dot(rot[0:3,0:3],yd[0:3])
      yd[3:] = np.dot(rot[0:3,0:3],yd[3:])

      ydd[0:3] = np.dot(rot[0:3,0:3],ydd[0:3])
      ydd[3:] = np.dot(rot[0:3,0:3],ydd[3:])

      return P, yd, ydd
    
    n = [1,	3,	0,	4,	2,	1,]
    h = [	0.526319896161023,	-0.169345026550763,	-0.461850572474118,
      0.707099952959623,	-0.982888313968955,	-0.922852046071500]
      
    y0 =	-2.21893729363067	
    z0 = 4.79108051617237	
    
    hg = 0.371475378268316	
    tg = 2*0.512041741562113

    dt = 0.01
    Ts = tg
    h = [hi*hg for hi in h]

    rot = pose_to_matrix(self.arm.get_current_frame().pose)
    rot = np.dot(ts.rotation_matrix(-y0,np.dot(rot[0:3,0:3],np.array([1,0,0]).transpose())),rot)
    P0 = np.dot(ts.rotation_matrix(-z0,np.dot(rot[0:3,0:3],np.array([0,0,1]).transpose())),rot)
    P0[0,3] = self.arm.get_current_frame().pose.position.x
    P0[1,3] = self.arm.get_current_frame().pose.position.y
    P0[2,3] = self.arm.get_current_frame().pose.position.z

    self.arm.set_pose_target(matrix_to_pose(P0))
    
    rate = rospy.Rate(1/dt)
        
    Q = pose_to_matrix(self.arm.get_current_frame().pose)

    for t in np.arange(0,2*Ts+dt,dt):

      P, yd, ydd = eepose(Q, t, n, h, dt, Ts)

      target = CartesianTrajectoryPoint()

      target.pose.position.x = P[0,3]
      target.pose.position.y = P[1,3]
      target.pose.position.z = P[2,3]

      q = ts.quaternion_from_matrix(P)
      target.pose.orientation.x = q[0]
      target.pose.orientation.y = q[1]
      target.pose.orientation.z = q[2]
      target.pose.orientation.w = q[3]

      target.twist.linear.x = yd[0]
      target.twist.linear.y = yd[1]
      target.twist.linear.z = yd[2]
      target.twist.angular.x = yd[3]
      target.twist.angular.y = yd[4]
      target.twist.angular.z = yd[5]

      target.acceleration.linear.x = ydd[0]
      target.acceleration.linear.y = ydd[1]
      target.acceleration.linear.z = ydd[2]
      target.acceleration.angular.x = ydd[3]
      target.acceleration.angular.y = ydd[4]
      target.acceleration.angular.z = ydd[5]

      self.arm.forward_target(target)
      
      rate.sleep()

      Q = P

def main():

  rospy.init_node('object_recognition_module_node')

  obj_rec_mod = ObjectRecognitionModule()

  rospy.loginfo('Object recognition module ready!')

  rospy.spin()

if __name__ == '__main__':
  main()