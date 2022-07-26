#!/usr/bin/env python3

import sys, os
import rospy

def main():
  rospy.init_node("tty_low_latency_node")

  os.system("setserial /dev/" + sys.argv[1] + " low_latency")

  rospy.loginfo("/dev/" + sys.argv[1] + " is now in low latency mode!")

if __name__ == "__main__":
  main()