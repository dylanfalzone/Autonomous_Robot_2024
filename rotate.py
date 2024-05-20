#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import os
import time
from std_msgs.msg import String, Int16

def rotate(Xmid):
    # List of commands for patrolling
    print(Xmid.data)
    if Xmid.data < 271:
        rospy.loginfo("Sending command: left")
        pub.publish('j')
        if Xmid < 50:
            rospy.loginfo("Sending command: left")
            pub.publish('j')
    elif Xmid.data > 369:
        rospy.loginfo("Sending command: right")
        pub.publish('l')
    
    
if __name__ == '__main__':
    rospy.init_node('rotate', anonymous=True)
    pub = rospy.Publisher('teleop_commands', String, queue_size=1)
    rospy.Subscriber('/rotate_data', Int16, rotate)
    rospy.spin()
