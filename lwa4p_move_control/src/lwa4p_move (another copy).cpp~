
/* Author: Zunchao Zheng*/

#include <iostream>   // std::cout
#include "ros/ros.h"
#include <std_msgs/Int8.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h" 

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <sensor_msgs/JointState.h>

#include <actionlib_msgs/GoalStatusArray.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

using namespace std;

ros::Publisher display_publisher;
moveit_msgs::DisplayTrajectory display_trajectory;

void excuteAction()
{
  std::cout<<"\n enter the action function" << std::endl;

  moveit::planning_interface::MoveGroup group("lwa4p_arm_indigo");
  moveit::planning_interface::MoveGroup::Plan my_plan;
  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
  
  //group.setRandomTarget();
/*
 * pose:
  position:
    x: -0.401106
    y: 0.0559284
    z: 0.453316
  orientation:
    x: 0.385722
    y: 0.451438
    z: 0.796702
    w: 0.112642
*/
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x=0.147209;
  target_pose1.orientation.y=0.852415;
  target_pose1.orientation.z=-0.0479141;
  target_pose1.orientation.w=0.499421;
  target_pose1.position.x = 0.134053;
  target_pose1.position.y = 0.16288;
  target_pose1.position.z = 0.572006;
  group.setPoseTarget(target_pose1);
  /*
  group.setStartStateToCurrentState();
  group.setJointValueTarget(jointstate_in);
  group.setNumPlanningAttempts(10);
  
  std::cout<<"\n pose before move:" << std::endl;
  std::cout<<group.getCurrentPose();
  std::cout<<"\n state before move:" << std::endl;
  std::cout<<*group.getCurrentState();
  */
  bool success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. 
  sleep(5.0);*/

  if (group.move())
  {
    sleep(9.0);
    ROS_INFO_STREAM("manipulator Moving done.");
  }
  else
  {
    sleep(5.0);
    ROS_WARN_STREAM("manipulator Moving failed.");
  }

  std::cout<<"\n pose after move:" << std::endl;
  std::cout<<group.getCurrentPose();
   /*std::cout<<"\n state after move:" << std::endl;
  std::cout<<*group.getCurrentState();
  
  std::vector<double> cpp_joint_state = group.getCurrentJointValues();
  std::cout<<"\n xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx jointstates after move:" << std::endl;
  for ( int i = 0; i < cpp_joint_state.size(); i++) {
            std::cout << cpp_joint_state[i] << " ";
        }
  std::cout<<"\n xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx jointstates after move:" << std::endl;
*/

}

void flagCallback(const std_msgs::Int8& flag_for_action)
{
 if (flag_for_action.data==11){
  ROS_INFO("Recieve 11 and start action:");
  excuteAction();}
else ROS_INFO("wait for 11:");

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lwa4p_move");
  ros::NodeHandle node_handle; 
  ros::AsyncSpinner spinner(1);
  spinner.start();
  /* This sleep is ONLY to allow Rviz to come up */
  //sleep(2.0);
  // Create a publisher for visualizing plans in Rviz.
  display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true); 
  // Create a subscriber for listening to matlab command
  ros::topic::waitForMessage<std_msgs::Int8>("chatter");
  ros::Subscriber sub = node_handle.subscribe("chatter", 1000, flagCallback);
  
  ros::spin();
  return 0;
}


