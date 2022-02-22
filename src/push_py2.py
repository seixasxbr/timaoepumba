#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
move_group = moveit_commander.MoveGroupCommander("arm")
# display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
#
pose_goal = geometry_msgs.msg.Pose()
# pose_goal.orientation.w = 0.0
pose_goal.position.x = 0.0
pose_goal.position.y = 0.0
pose_goal.position.z = 0.9
#
# group_variable_values = group.get_current_joint_values()

# group_variable_values[0] = 10
# group_variable_values[1] = 0
# group_variable_values[3] = 0
# group_variable_values[5] = 0
# group.set_joint_value_target(group_variable_values)

move_group.set_pose_target(pose_goal)

plan2 = move_group.plan()
move_group.go(wait=True)

rospy.sleep(5)

moveit_commander.roscpp_shutdown()