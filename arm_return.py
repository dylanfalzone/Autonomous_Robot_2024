#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_robot_arm', anonymous=True)

    # Interface to the robot as a whole.
    robot = moveit_commander.RobotCommander()

    # Interface to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()

    # Interface to the group of joints making up the robot's arm.
    group_name = "arm"  # The name of the group you want to control (adjust as necessary)
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    rviz_x = 0.18
    rviz_y = 0.0
    rviz_z = 0.49
    
    print(rviz_x)
    print(rviz_y)
    print(rviz_z)

    # Position and orientation target
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 1.0
    pose_target.position.x = rviz_x  # Modify these values based on the target coordinates
    pose_target.position.y = rviz_y
    pose_target.position.z = rviz_z

    # Set the goal state to the pose you have just created.
    move_group.set_pose_target(pose_target)

    # Plan to the new pose
    plan = move_group.plan()

    # Execute the plan
    move_group.go(wait=True)

    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    
    # Clear targets after planning with poses.
    move_group.clear_pose_targets()

if __name__ == '__main__':
    main()

