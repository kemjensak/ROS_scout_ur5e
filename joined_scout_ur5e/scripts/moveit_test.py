#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


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
while True:
    group.set_named_target('up')
    plan = group.plan()
    group.execute(plan, wait=True)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.3
    pose_goal.position.y = 0.3
    pose_goal.position.z = 0.3
    group.set_pose_target(pose_goal)

    plan = group.plan()
    group.execute(plan, wait=True)