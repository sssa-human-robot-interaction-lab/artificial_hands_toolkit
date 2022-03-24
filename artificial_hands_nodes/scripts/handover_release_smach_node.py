#! /usr/bin/env python3

import rospy

from artificial_hands_py.handover_release_smach import HandoverReleaseSmach

def main():

  rospy.init_node("handover_release_smach",)

  handover_release_smach = HandoverReleaseSmach()
  
  handover_release_smach.execute()
  
if __name__ == '__main__':
  main()