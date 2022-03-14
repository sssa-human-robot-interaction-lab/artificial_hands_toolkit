from math import atan2
from os import mkdir
from pickletools import read_uint1
from platform import release
from time import strftime

from handover_proprioceptive_base import *
  
def demo_execution(robot : RobotCommander, start_poses : MarkerArray, bagdir : str):
  robot.arm.switch_to_controller(robot.arm.j_traj_ctrl)

  ref_to_base = robot.arm.tf_buffer.lookup_transform('base','handover_start_frame',rospy.Time(0),timeout=rospy.Duration(1))

  end_pose = PoseStamped()
  end_pose.pose = robot.arm.get_eef_frame().pose

  # marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=1000)

  if not robot.wait_for_enter():
    rospy.logwarn("Abroting execution!")
    return

  c = 0
  for marker in start_poses.markers:
    # marker_pub.publish(marker)

    c += 1
    bag_name = bagdir+"/marker{0}.bag".format(c)
    
    if rospy.is_shutdown():
      break

    marker_pose = PoseStamped()
    marker_pose.pose = marker.pose
    ref_orientation = marker_pose.pose.orientation
    marker_pose = tf2_geometry_msgs.do_transform_pose(marker_pose,ref_to_base)
    marker_pose.pose.orientation = ref_orientation

    if not robot.wait_for_touch_enter():
      rospy.logwarn("Terminated execution!")
      break

    sleep(2)
    robot.arm.paused = False
    robot.hand.close_cyl()
    (plan, fraction) = robot.arm.compute_cartesian_path([marker_pose.pose],0.01,0.0)
    if fraction == 1:
      robot.arm.execute(plan)
      robot.do_object_recognition()
      (plan, fraction) = robot.arm.compute_cartesian_path([end_pose.pose],0.01,0.0)
      if fraction == 1:
        robot.wrist.open_bag(bag_name)
        robot.wrist.wristCommand("wrist_dynamics_command/subscribe")
        robot.wrist.wristCommand("wrist_dynamics_command/start_loop")     
        robot.wrist.wristCommand("wrist_dynamics_command/set_calibration")  
        robot.wrist.wristCommand("wrist_dynamics_mode/save_interaction")      
        delta = Pose()
        goal_time = 5
        delta.position.x = end_pose.pose.position.x - marker_pose.pose.position.x
        delta.position.y = end_pose.pose.position.y - marker_pose.pose.position.y
        delta.position.z = end_pose.pose.position.z - marker_pose.pose.position.z
        delta_thread = Thread(target=robot.arm.servo_delta,args=(goal_time,delta,Pose()))
        delta_thread.start()
        rate = rospy.Rate(10)
        while robot.eef_distance(end_pose.pose.position) > 0.2:
          rate.sleep()
        robot.hand.switch_to_controller(robot.hand.vel_ctrl)
        rel_thread = Thread(target=robot.hand.open,args=(False,3,))
        robot.wrist.wristCommand("wrist_dynamics_mode/trigger_dynamics")
        robot.wait_for_release_trigger(10)
        if robot.wrist.detection.trigger:
          rel_thread.start()
          robot.arm.paused = True
        robot.wrist.close_bag()
        robot.wrist.wristCommand("wrist_dynamics_command/stop_loop")     
      else:
        rospy.logwarn("Failed to compute return path for marker %i", marker.id)
    else:
      rospy.logwarn("Skipping marker %i", marker.id)

def main():

  rospy.init_node('handover_proprio_test_node')

  robot = RobotCommander()

  rate = rospy.Rate(125)
  twist_rate = rospy.Rate(200)
  robot.arm.set_rate(rate)
  robot.twist_servo.set_rate(rate)

  robot.arm.switch_to_controller(robot.arm.j_traj_ctrl)
  robot.arm.go(joint_ini, wait=True)
  robot.hand.open(vel=3)

  if not robot.check_calib():
    robot.do_calib()
    robot.arm.go(joint_ini, wait=True)

  handover_end_pose = robot.arm.get_eef_frame().pose
  handover_start_frame_angle = atan2(handover_end_pose.position.y,handover_end_pose.position.x)
  handover_start_frame = ts.rotation_matrix(handover_start_frame_angle,[0,0,1],[0,0,0])

  br_thread = Thread(target=send_frame,args=([0,0,0],ts.quaternion_from_matrix(handover_start_frame),'handover_start_frame','base'))
  br_thread.start()

  n_mark = 5
  markers = generate_markers(handover_end_pose,n_mark)
  rospy.loginfo('Marker generation done')
  marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=1000)
  marker_pub.publish(markers)

  bagdir = "/home/penzo/Desktop/proprioceptive_test_{0}".format(strftime("%Y%m%d_%H%M%S"))
  mkdir(bagdir)
  marker_bag = rosbag.Bag(bagdir+'/markers.bag','w')
  marker_bag.write('markers',markers)
  marker_bag.close()
  demo_execution(robot,markers,bagdir)

  # set_key = ''
  # rate = rospy.Rate(1)
  # while not rospy.is_shutdown():

  #   if set_key == 'h':
  #     rospy.loginfo("Moving to home position (stop to follow dmp)")
  #     robot.arm.switch_to_controller(robot.arm.j_traj_ctrl)
  #     robot.arm.go(joint_ini, wait=True)
  #     robot.arm.switch_to_controller(robot.arm.j_traj_ctrl)
  #   elif set_key == 'r':
  #     rospy.loginfo("Current eef pose")
  #     print(robot.arm.get_eef_frame())
  #   elif set_key == 's':
  #     rospy.loginfo("Sending markers to rviz")
  #     marker_pub.publish(markers)
  #   elif set_key == 'n':
  #     rospy.loginfo("Generating new markers")
  #     markers = generate_markers(handover_end_pose,n_mark)
  #     marker_pub.publish(markers)
  #   elif set_key == 'd':
  #     rospy.loginfo("Starting demo")
  #     demo_thread = Thread(target=demo_execution, args=(robot,markers))
  #     demo_thread.start()

  #   if rate.remaining().to_sec() > 0:
  #     set_key = get_key(rate.remaining().to_sec())
  #   else:
  #     set_key = ''

  #   if (set_key == '\x03'):
  #     rospy.signal_shutdown('bye!')
  #   else:
  #     rate.sleep()

if __name__ == '__main__':
  main()