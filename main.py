#!/usr/bin/python

from __future__ import print_function

import sys
import rospy
from std_srvs.srv import Trigger, TriggerRequest
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import os
import time
from std_msgs.msg import String, Int16

def main_function(done):
    os.system('rosrun darknet_ros toggle.py')
    #rospy.loginfo("runing toggle")
    print(done)
if __name__ == '__main__':
    rospy.init_node('main', anonymous=True)
    rospy.Subscriber('done_commands', String, main_function)   
    main_function(0)
    rospy.spin()
