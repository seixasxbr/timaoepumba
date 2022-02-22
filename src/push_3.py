import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "arm"
group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

pose_goal = geometry_msgs.msg.Pose()
# pose_goal.orientation.w = 0.0
pose_goal.position.x = 0.0
pose_goal.position.y = 0.0
pose_goal.position.z = 0.9
group.set_pose_target(pose_goal)
plan = group.go(wait=True)

group.stop()
group.clear_pose_targets() 