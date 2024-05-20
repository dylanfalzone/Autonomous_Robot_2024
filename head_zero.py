#!/usr/bin/env python
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def look_forward():
    rospy.init_node('fetch_head_control_action_client')

    # Connect to the action server
    client = actionlib.SimpleActionClient('head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo("Waiting for head controller action server...")
    client.wait_for_server()

    # Define the goal (trajectory)
    goal = FollowJointTrajectoryGoal()
    trajectory = JointTrajectory()
    trajectory.joint_names = ['head_pan_joint', 'head_tilt_joint']

    # Create a point in the trajectory
    point = JointTrajectoryPoint()
    point.positions = [0, 0]  # Pan and tilt positions (radians)
    point.time_from_start = rospy.Duration(1)  # Reach the goal in 1 second
    trajectory.points.append(point)

    goal.trajectory = trajectory

    # Send the goal
    rospy.loginfo("Sending head position goal...")
    client.send_goal(goal)

    # Wait for the server to finish performing the action
    client.wait_for_result()

    # Print result
    return client.get_result()

if __name__ == '__main__':
    try:
        result = look_forward()
        if result:
            rospy.loginfo("Head movement executed successfully")
        else:
            rospy.loginfo("Head movement failed")
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted before completion")

