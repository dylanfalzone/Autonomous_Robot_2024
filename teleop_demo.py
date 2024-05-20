#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int16

# Initialize speed and turn rates
speed = 2
turn = 0.3
done = 'move'

# Callback function for the subscriber
def callback(data):
    global speed, turn, done
    twist = Twist()
    if data.data == 'i':
        twist.linear.x = speed
    elif data.data == 'k':
        twist.linear.x = -speed
    elif data.data == 'j':
        twist.angular.z = turn
    elif data.data == 'l':
        twist.angular.z = -turn
    else:
        # If the command is not recognized, stop the robot
        twist.linear.x = 0
        twist.angular.z = 0
    # Publish the twist message to the cmd_vel topic
    pub.publish(twist)
    pub_done.publish(done)

if __name__=="__main__":
    rospy.init_node('teleop_twist_keyboard_modified')
    # Publisher to the cmd_vel topic
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    # Subscriber to the teleop_commands topic
    rospy.Subscriber('teleop_commands', String, callback)
    pub_done = rospy.Publisher('done_commands', String, queue_size=1)   
    rospy.spin()


