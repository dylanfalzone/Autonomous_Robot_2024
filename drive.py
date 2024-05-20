#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import os
import time
from std_msgs.msg import String

def drive(distance):

    print(distance)

    if float(distance.data) > 1300:
        pub.publish('i')
        rospy.loginfo("Sending command: forward")
    elif float(distance.data) < 750:
        rospy.loginfo("Sending command: back")
        pub.publish('k')
    else:
        rospy.loginfo("Sending command: forward")
        pub.publish('i')


if __name__ == '__main__':
    rospy.init_node('drive', anonymous=True)
    pub = rospy.Publisher('teleop_commands', String, queue_size=1)
    rospy.Subscriber('/drive_data', String, drive)
    rospy.spin()
