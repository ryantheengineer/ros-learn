#!/usr/bin/env python
## EVERY Python ROS Node will have this^^ at the top.
## It makes sure the script is executed as a python script.
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

# Importing the String message type (string container) for publishing
import rospy
from std_msgs.msg import String

def talker():
    # Declare that this node is publishing to chatter topic, using message
    # type 'String'. 'queue_size' limits the number of queued messages if a
    # subscriber isn't receiving them fast enough
    pub = rospy.Publisher('chatter', String, queue_size=10)

    # This line names the node as 'talker.' It is required for communication
    # with the ROS Master. The name cannot include any slashes.
    # anonymous = True makes sure the node has a unique name by adding random
    # numbers to the end of the node name.
    rospy.init_node('talker', anonymous=True)
    
    # Create a Rate object called rate. Goes through the loop at 10Hz, assuming
    # processing time doesn't exceed that.
    rate = rospy.Rate(10) # 10hz

    # Check if rospy is running (it isn't if there is a Ctrl-C)
    while not rospy.is_shutdown():
        # Simple string with time
        hello_str = "hello world %s" % rospy.get_time()
        # This line does 3 things:
        # 1. Print messages to screen
        # 2. Write messages to the Node's log file
        # 3. Write messages to rosout (use rqt_console to show message output)
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

# This section checks whether the script is being run directly or whether it's
# being imported only. If it's being imported, the code is not executed, just
# made available as functions for the other script
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
