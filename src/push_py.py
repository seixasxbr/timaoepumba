# from msilib.schema import Environment



# from __future__ import print_function
from six.moves import input

import sys
import copy
import numpy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

if __name__ == "__main__":
    rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
    move_group = moveit_commander.MoveGroupCommander("arm")
    pose_goal = geometry_msgs.msg.Pose()
    
# pose_goal.orientation.w = 1.0
pose_goal.position.x = 0.0
pose_goal.position.y = 0.0
pose_goal.position.z = 0.9

move_group.set_pose_target(pose_goal)

plan = move_group.go(wait=True)
# Calling `stop()` ensures that there is no residual movement
# move_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
# move_group.clear_pose_targets()

# move_group.execute(plan, wait=True)
move_group.execute(plan)
rospy.sleep(5)
# current_pose = self.move_group.get_current_pose().pose
# return all_close(pose_goal, current_pose, 0.01)