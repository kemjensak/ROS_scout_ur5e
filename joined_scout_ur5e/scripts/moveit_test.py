#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import copy
from math import pi
from math import tan
from std_msgs.msg import String
from std_msgs.msg import Int32
from moveit_commander.conversions import pose_to_list
from joined_scout_ur5e.msg import cmd_ur5e

scale = 1
wrist_to_imu = 0.06
wrist_to_fingertip = 0.2

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_test', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = robot.get_group_names()
print(group_name)
group = moveit_commander.MoveGroupCommander(group_name[0])

def CartesianMoveCallback(data):
    rospy.loginfo(rospy.get_caller_id() + "cmd_ur5e Input mode %s", data.mode)
    rospy.loginfo(rospy.get_caller_id() + "cmd_ur5e Input distance %d", data.dist)
    
    waypoints = []
    
    wpose = group.get_current_pose().pose
    joint_goal = group.get_current_joint_values()

    target_val = wrist_to_fingertip * tan(data.dist / wrist_to_imu)

    # for safety in test
    if (wpose.position.z < 0.6):
        group.set_named_target('test')
        plan = group.plan()
        rospy.sleep(2)
        group.execute(plan, wait=True)
        wpose = group.get_current_pose().pose
        
    if data.mode == 1:
        wpose.position.y -= scale * target_val  
        waypoints.append(copy.deepcopy(wpose))

    elif data.mode == 2:
        wpose.position.y += scale * target_val
        waypoints.append(copy.deepcopy(wpose))

    elif data.mode == 3:
        wpose.position.z += scale * target_val 
        waypoints.append(copy.deepcopy(wpose))

    elif data.mode == 4:
        wpose.position.z -= scale * target_val 
        waypoints.append(copy.deepcopy(wpose))

    elif data.mode == 5:
        joint_goal[5] += scale * target_val
        group.go(joint_goal, wait=True)
        return()

    elif data.mode == 6:
        joint_goal[5] -= scale * target_val
        group.go(joint_goal, wait=True)
        return()

    else:
        rospy.loginfo("undefined move mode %s", data)
        return()
    
  
    # wpose.position.z -= scale * 0.1  # First move up (z)
    # wpose.position.y += scale * 0.2  # and sideways (y)
    # waypoints.append(copy.deepcopy(wpose))

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
                                    0.001,        # eef_step
                                    0.0)         # jump_threshold

    rospy.sleep(2)
    group.execute(plan, wait=True)



def cartesian_move_test():
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)

    rospy.Subscriber("cmd_ur5e", cmd_ur5e, CartesianMoveCallback)


    rospy.spin()
        
if __name__ == '__main__':
        cartesian_move_test()
        