from handover_proprioceptive_base import *

import csv
from time import strftime

class WristSubscriber():

  def __init__(self) -> None:
    sub = rospy.Subscriber("wrist_dynamics_data",WristDynamicsStamped,self.wrist_data_callback)
    self.c = 0
    self.res = [0,0,0,0,0,0]

  def save_calib(self):
    msg = rospy.wait_for_message("wrist_dynamics_data",WristDynamicsStamped,rospy.Duration(1))
    csv_name = "/home/penzo/Desktop/calib_param_{0}.csv".format(strftime("%Y%m%d_%H%M%S"))
    csv_file = open(csv_name,'w')
    csv_writer = csv.writer(csv_file)
    cal = [float(c) for c in list(msg.wrist_dynamics.cal.split(" "))[0:-1]]
    csv_writer.writerow(cal)
    csv_file.close()
  
  def wrist_data_callback(self, msg : WristDynamicsStamped):
    self.c += 1
    self.res[0] += msg.wrist_dynamics.gravity.x
    self.res[1] += msg.wrist_dynamics.gravity.y
    self.res[2] += msg.wrist_dynamics.gravity.z
    self.res[3] += msg.wrist_dynamics.wrench_fir.force.x
    self.res[4] += msg.wrist_dynamics.wrench_fir.force.y
    self.res[5] += msg.wrist_dynamics.wrench_fir.force.z
  
  def average_results(self) -> list:
    return [r/self.c for r in self.res]
  
  def reset(self):
    self.c = 0
    self.res = [0,0,0,0,0,0]

def main():

  rospy.init_node('vtk_ft_calib_node')

  robot = RobotCommander()

  wrist_sub = WristSubscriber()
  
  arm_rate = rospy.Rate(125)
  robot.arm.set_rate(arm_rate)

  robot.arm.switch_to_controller(robot.arm.j_traj_ctrl)
  robot.arm.go(joint_home,wait=True)
  
  # if not robot.check_calib():
  #   robot.do_calib()
  robot.wrist.wrist_command("wrist_dynamics_command/subscribe")
  robot.wrist.wrist_command("wrist_dynamics_command/start_loop")     
  wrist_sub.save_calib()
  robot.wrist.wrist_command("wrist_dynamics_command/stop_loop")

  ref_pose = PoseStamped()
  ref_pose.header.frame_id = 'base'
  ref_pose.pose.position.x = .1
  ref_pose.pose.position.y = -.5
  ref_pose.pose.position.z = .4
  ref_pose.pose.orientation = list_to_quat(ts.quaternion_about_axis(0,[0,0,1]))

  (plan, fraction) = robot.arm.compute_cartesian_path([ref_pose.pose],0.01,0.0)
  if fraction == 1:
    robot.arm.execute(plan)

  sleep(2)

  robot.wrist.wrist_command("wrist_dynamics_command/subscribe")
  robot.wrist.wrist_command("wrist_dynamics_command/start_loop")     
  robot.wrist.wrist_command("wrist_dynamics_command/set_calibration") 

  csv_name = "/home/penzo/Desktop/calib_data_{0}.csv".format(strftime("%Y%m%d_%H%M%S"))
  csv_file = open(csv_name,'w')
  csv_writer = csv.writer(csv_file)

  robot.arm.switch_to_controller(robot.arm.c_traj_ctrl)

  c_pose = robot.arm.get_eef_frame()

  wrist_sub.reset()
  sleep(.5)
  res = wrist_sub.average_results()
  csv_writer.writerow(res)

  n_div = 9
  for c in range(0,n_div):

    q = ts.quaternion_multiply(ts.quaternion_about_axis((c+1)*pi/n_div,[1,0,0]),quat_to_list(ref_pose.pose.orientation))
    q_norm = [float(i)/max(q) for i in q]
    c_pose.pose.orientation = list_to_quat(q_norm)

    robot.arm.servo_goal(1,c_pose,False) 

    wrist_sub.reset()
    sleep(.5)
    res = wrist_sub.average_results()
    csv_writer.writerow(res)

    k_pose = robot.arm.get_eef_frame()

    for k in range(0,n_div):
      if c == n_div - 1:
        break
      rot = ts.quaternion_matrix(quat_to_list(c_pose.pose.orientation))
      if c % 2:
        q = ts.quaternion_multiply(ts.quaternion_about_axis((k+1)*2*pi/n_div,rot[:,2]),quat_to_list(c_pose.pose.orientation))
      else:
        q = ts.quaternion_multiply(ts.quaternion_about_axis((k+1)*-2*pi/n_div,rot[:,2]),quat_to_list(c_pose.pose.orientation))
      q_norm = [float(i)/max(q) for i in q]
      
      k_pose.pose.orientation = list_to_quat(q_norm)

      robot.arm.servo_goal(1,k_pose,False)

      wrist_sub.reset()
      sleep(.5)
      res = wrist_sub.average_results()
      csv_writer.writerow(res)

  csv_file.close() 

  rospy.loginfo('bye!')     
  rospy.spin() 

if __name__ == '__main__':
  main()