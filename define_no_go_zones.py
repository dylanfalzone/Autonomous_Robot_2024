#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from moveit_python import PlanningSceneInterface

def define_no_go_zones():
    # Initialize the planning scene interfaces for each relevant link
    base = PlanningSceneInterface("base_link")
    torso = PlanningSceneInterface("torso_lift_link")
    head = PlanningSceneInterface("torso_lift_link")
    face = PlanningSceneInterface("head_tilt_link")

    # Ground protection
    base.addCube("my_front_ground", 2, 1.1, 0.0, -0.97)
    base.addCube("my_back_ground", 2, -1.2, 0.0, -0.97)
    base.addCube("my_left_ground", 2, 0.0, 1.2, -0.97)
    base.addCube("my_right_ground", 2, 0.0, -1.2, -0.97)


    # Robot torso protection
    base.addBox("base", 0.35, 0.45, 0.1, 0.15, 0.0, 0.32)
    base.addBox("base_right", 0.3, 0.05, 0.35, 0.15, -0.25, 0.22)
    base.addBox("base_left", 0.3, 0.05, 0.35, 0.15, 0.25, 0.22)
    base.addBox("base_front", 0.05, 0.46, 0.35, 0.33, 0.0, 0.22)
"""
    # Robot head protection
    head.addBox("head", 0.4, 0.4, 0.0, 0.1, 0.0, 0.8)

    # Robot body protection
    torso.addBox("torso_left", 0.2, 0.0, 1.0, 0.0, 0.25, 0.5)
    torso.addBox("torso_right", 0.2, 0.0, 1.0, 0.0, -0.25, 0.5)

    # Face protection, properly linked
    face.addBox("face", 0.0, 0.3, 0.19, 0.11, 0.0, 0.0)
"""
def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('no_go_zone_definition', anonymous=True)

    rospy.sleep(1)  # Wait for the interfaces to initialize completely

    # Define and add no-go zones to the planning scene
    define_no_go_zones()

    rospy.sleep(5)  # Keep the node alive long enough to ensure the boxes are added

if __name__ == '__main__':
    main()

