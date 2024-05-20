#!/usr/bin/env python
import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

def control_gripper(width):
    rospy.init_node('fetch_gripper_control')

    client = actionlib.SimpleActionClient('/gripper_controller/gripper_action', GripperCommandAction)
    rospy.loginfo("Waiting for gripper action server...")
    client.wait_for_server()

    goal = GripperCommandGoal()
    goal.command.position = width
    goal.command.max_effort = 100.0

    rospy.loginfo("Sending goal to gripper: set width to {} meters".format(width))
    client.send_goal(goal)
    client.wait_for_result()

    result = client.get_result()
    rospy.loginfo("Action finished with state: {}".format(client.get_state()))
    rospy.loginfo("Action result: {}".format(result))
    return result

if __name__ == '__main__':
    try:
        result_open = control_gripper(0.1)
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted before completion")

