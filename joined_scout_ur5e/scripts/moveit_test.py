#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
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

    rate = rospy.Rate(0.2) # 0.2 hz

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
            rate.sleep()
            group.execute(plan, wait=True)

        except:
            group.set_named_target('up')
            plan = group.plan()
            group.execute(plan, wait=True)

        


if __name__ == '__main__':
    try:
        MoveIt()
    except rospy.ROSInterruptException:
        pass