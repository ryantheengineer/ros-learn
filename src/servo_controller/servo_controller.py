#!/usr/bin/env python
"""servo_controller.py: Interfaces with Arduino using ROS to control servos"""

# For now, make sure roscore is already running directly in the terminal.

import rospy
from std_msgs.msg import _UInt16

def servo-write():
    pub = rospy.Publisher('cmd_msg',_UInt16)
