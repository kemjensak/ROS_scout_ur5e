

# joined_scout_ur5e
*This repository is for Developing project.*

*So, please be aware that it DOSE NOT working properly*

joined scout2.0 mobile platform and UR5e manipulator


## Installation
On Ubuntu bionic, ROS melodic 

    git clone https://github.com/kemjensak/joined_souct_ur5e.git
    or (to clone with submodules)
    git clone --recurse-submodules https://github.com/kemjensak/joined_souct_ur5e.git
    
Move **ALL FILES(including hidden files) in joined_scout_ur5e** to ***catkin_ws/src***

    git submodule init
    git submodule update


And follow instructions of each package



## To do list

 0. Configure ur_joined_moveit_config package with ~~*robotiq 2F-85 and*(*DONE*)~~ realsense D455(attached to gripper)
 1. Communicate(or control) with (***wrist connected***)2F-85 Gripper through ROS
	 - https://dof.robotiq.com/discussion/1362/wrist-camera-and-2-finger-gripper-in-ros-moveit-and-gazebo?_ga=2.203114449.1961611014.1611926640-791306080.1611816457
	  - https://dof.robotiq.com/discussion/1671/how-do-you-control-the-2f-gripper-using-ros-if-its-connected-to-a-ur5e-robot
	  - https://dof.robotiq.com/discussion/1914/robotiq-gripper-with-ur5-cb3-thru-wrist-connector
	   - https://dof.robotiq.com/discussions/tagged/Other-ROS/p1
	   - https://dof.robotiq.com/discussion/1988/robotiq-2f-grippers-with-universal-robot-ros-driver
	   - [UR_driver's feature(testing now)](https://github.com/kemjensak/joined_souct_ur5e/blob/master/Universal_Robots_ROS_Driver/ur_robot_driver/doc/setup_tool_communication.md)

 2. Find causes of UR's jitter on gazebo



       
 

## Changelog
2021/01/29
 -  Added 2F-85 gripper package

2021/04/04
 -  Added moveit_test.py to control UR5E through IMU data

2021/04/10
 -  Added cmd_ur5e.msg (IMU's mode and distance data)
 -  Added ur_2f_moveit_config (only for arm, grasping controller will be added)
	 
