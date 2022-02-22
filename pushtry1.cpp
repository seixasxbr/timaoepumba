#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_tests");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("invisible_link");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  ROS_INFO_NAMED( "Reference frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("End effector link: %s", move_group.getEndEffectorLink().c_str());

  ros::Time start_time = ros::Time::now();

  //JUNTAS
moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);  
  joint_group_positions[0] = 0.0;  
  joint_group_positions[1] = -0.12;  // 
  joint_group_positions[2] = 2.2; // p
  joint_group_positions[3] = 1.57;
  joint_group_positions[4] = 0.68;
  move_group.setJointValueTarget(joint_group_positions);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  move_group.plan(my_plan);
  while(move_group.plan(my_plan).val == -1){
    move_group.plan(my_plan);
  }
  move_group.execute(my_plan);


//   current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);  
//   joint_group_positions[0] = 0.0;  
//   joint_group_positions[1] = 0.0;  // 
//   joint_group_positions[2] = 0.0; // p
//   joint_group_positions[3] = 0.0;
//   joint_group_positions[4] = 0.0;
//   move_group.setJointValueTarget(joint_group_positions);
//   move_group.plan(my_plan);
//   while(move_group.plan(my_plan).val == -1){
//     move_group.plan(my_plan);
//   }
//   move_group.execute(my_plan);

// try
geometry_msgs::Pose target_pose2;
  target_pose2 = target_pose2;
  //PRESS BUTTON
  target_pose2.position.x = 0.0;  
  target_pose2.position.y = 0.0;
  target_pose2.position.z = 1.0;

  move_group.setGoalPositionTolerance(0.1);
  move_group.setGoalOrientationTolerance(1.0); 
  move_group.setPlanningTime(10);

  move_group.setPoseTarget(target_pose2);
  move_group.plan(my_plan);
  while (move_group.plan(my_plan).val != 1){
    move_group.plan(my_plan);
    }
  move_group.execute(my_plan); 

  ros::shutdown();
return 0;
 }