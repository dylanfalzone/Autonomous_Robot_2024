#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String, Int16
import os

global robot, move_group, scene, target, done
done = 'arm'
target = 'bottle'
pub_target = rospy.Publisher('/target_data', String, queue_size = 10)
pub_done = rospy.Publisher('/done_commands', String, queue_size = 1)

def arm_movement_callback(data):
    global temp1, temp2, temp3, robot, move_group, scene, target, done
    
    try:
        coords = data.data.split()
        temp1 = float(coords[0])
        temp2 = float(coords[1])
        temp3 = float(coords[2])
    except (IndexError, ValueError) as e:
        rospy.logerr("Error parsing incoming data: {}".format(e))
        return

    cam_x = temp1 + 10
    cam_y = temp2
    cam_z = temp3 - 80

    rviz_x = cam_z / 1000
    rviz_y = cam_x / 1000
    rviz_z = (cam_y + 1120) / 1000



    if target == 'bottle':
        # Create a no-go zone below the bottle (representing the table)
        table_height = 1.0  # 10 cm high
        table_size = 3.0    # 50 cm square    
        table_pose = PoseStamped()
        table_pose.header.frame_id = robot.get_planning_frame()
        table_pose.pose.position.x = rviz_x + 1.4
        table_pose.pose.position.y = rviz_y
        table_pose.pose.position.z = rviz_z - ((table_height / 2) + 0.15) 
        remove_no_go_zone()
        scene.add_box("no_go_zone", table_pose, (table_size, table_size, table_height))
        rospy.loginfo("Added no-go zone")
        #move right behind bottle
        pose_target = Pose()
        pose_target.orientation.w = 1.0
        pose_target.position.x = rviz_x - 0.08
        pose_target.position.y = rviz_y
        pose_target.position.z = rviz_z
        move_group.set_pose_target(pose_target)
        plan = move_group.plan()
        move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        
        #move forward to bottle
        os.system('rosrun nav gripper_open.py')
        pose_target = Pose()
        pose_target.orientation.w = 1.0
        pose_target.position.x = rviz_x + 0.09
        pose_target.position.y = rviz_y
        pose_target.position.z = rviz_z
        move_group.set_pose_target(pose_target)
        plan = move_group.plan()
        move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
    
        #close gripper
        os.system('rosrun nav gripper_close.py')
        target = 'bottle in hand'

    elif target == 'person':
        # Create a no-go zone around the person
        table_height = 100
        table_size = 3.0      
        table_pose = PoseStamped()
        table_pose.header.frame_id = robot.get_planning_frame()
        table_pose.pose.position.x = rviz_x + 2
        table_pose.pose.position.y = rviz_y
        table_pose.pose.position.z = rviz_z - ((table_height / 2) + 0.15) 
        remove_no_go_zone()
        scene.add_box("no_go_zone", table_pose, (table_size, table_size, table_height))
        rospy.loginfo("Added no-go zone")

        #move bottle in front of person
        pose_target = Pose()
        pose_target.orientation.w = 1.0
        pose_target.position.x = rviz_x - 0.2
        pose_target.position.y = rviz_y
        pose_target.position.z = rviz_z
        move_group.set_pose_target(pose_target)
        plan = move_group.plan()
        move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        #open gripper
        os.system('rosrun nav gripper_open.py')
        target = 'bottle'
        pub_target.publish(target)
    
    #tuck arm
    pose_target = Pose()
    pose_target.orientation.w = 1.0
    pose_target.position.x = 0.18
    pose_target.position.y = 0.0
    pose_target.position.z = 0.49
    move_group.set_pose_target(pose_target)
    plan = move_group.plan()
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    if target == 'bottle in hand':
        target = 'person'
        pub_target.publish(target)

    pub_done.publish(done)

def remove_no_go_zone():
    scene.remove_world_object('no_go_zone')
    print('nogozone removed')

def main():
    global robot, move_group, scene

    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node('arm_to_target', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    rospy.Subscriber("/arm_movement_data", String, arm_movement_callback)

    pub_target.publish(target)

    rospy.spin()


if __name__ == '__main__':
    main()

