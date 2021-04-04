#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import copy
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def MoveIt():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_test', anonymous=True)


    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = robot.get_group_names()
    print(group_name)
    group = moveit_commander.MoveGroupCommander(group_name[1])

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)

    # rate = rospy.Rate(0.2) # 0.2 hz
def move_test():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_test', anonymous=True)


    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = robot.get_group_names()
    print(group_name)
    group = moveit_commander.MoveGroupCommander(group_name[1])

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)

    while not rospy.is_shutdown():
        

        # will be replaced to receiving msgs!
        # sample values
        move_positon_x = 0
        move_positon_y = 0
        move_positon_z = -0.05
        move_orientation_x = 0
        move_orientation_y = 0
        move_orientation_z = 0
        move_orientation_w = 0

        
        # group.set_named_target('up')
        # plan = group.plan()
        # group.execute(plan, wait=True)
        
        current_ = group.get_current_pose()

        print(current_)

        if (current_.pose.position.z < 0.6):
            group.set_named_target('up')
            plan = group.plan()
            rospy.sleep(2)
            group.execute(plan, wait=True)
            current_ = group.get_current_pose()

        pose_goal = geometry_msgs.msg.Pose()
        # pose_goal = current_
        pose_goal.position.x = current_.pose.position.x + move_positon_x
        pose_goal.position.y = current_.pose.position.y + move_positon_y
        pose_goal.position.z = current_.pose.position.z + move_positon_z

        pose_goal.orientation.x = current_.pose.orientation.x + move_orientation_x
        pose_goal.orientation.y = current_.pose.orientation.y + move_orientation_y
        pose_goal.orientation.z = current_.pose.orientation.z + move_orientation_z
        pose_goal.orientation.w =  current_.pose.orientation.w + move_orientation_w
        group.set_pose_target(pose_goal)

        try:
            plan = group.plan()
            rospy.sleep(2)
            group.execute(plan, wait=True)

        except:
            group.set_named_target('up')
            plan = group.plan()
            group.execute(plan, wait=True)

def cartesian_move_test():

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_test', anonymous=True)


    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = robot.get_group_names()
    print(group_name)
    group = moveit_commander.MoveGroupCommander(group_name[1])

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)
    while not rospy.is_shutdown():
        scale = 1
        waypoints = []

        wpose = group.get_current_pose().pose

        if (wpose.position.z < 0.6):
            group.set_named_target('up')
            plan = group.plan()
            rospy.sleep(2)
            group.execute(plan, wait=True)
            wpose = group.get_current_pose().pose
            
        wpose.position.z -= scale * 0.1  # First move up (z)
        # wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        # waypoints.append(copy.deepcopy(wpose))

        # wpose.position.y -= scale * 0.1  # Third move sideways (y)
        # waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        # return plan, fraction
        rospy.sleep(2)
        group.execute(plan, wait=True)
        

if __name__ == '__main__':
    try:
        # MoveIt()
        # move_test()
        cartesian_move_test()

    except rospy.ROSInterruptException:
        pass