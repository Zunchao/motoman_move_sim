/* Author: Zunchao Zheng */

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace std;

ros::Publisher display_publisher;
ros::Publisher state_publisher;
moveit_msgs::DisplayTrajectory display_trajectory;

void poseCallback()
{
    const std::string PLANNING_GROUP_mh50 = "motoman_mh50";
    const std::string PLANNING_GROUP_mh5  = "motoman_mh5";

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));

    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));


    moveit::planning_interface::MoveGroupInterface move_group_mh50(PLANNING_GROUP_mh50);
    const robot_state::JointModelGroup* joint_model_group_mh50 =
            move_group_mh50.getCurrentState()->getJointModelGroup(PLANNING_GROUP_mh50);
    moveit::core::RobotStatePtr current_state_mh50 = move_group_mh50.getCurrentState();
    std::vector<double> joint_group_positions_mh50;
    current_state_mh50->copyJointGroupPositions(joint_model_group_mh50, joint_group_positions_mh50);

    moveit::planning_interface::MoveGroupInterface move_group_mh5(PLANNING_GROUP_mh5);
    const robot_state::JointModelGroup* joint_model_group_mh5 =
            move_group_mh5.getCurrentState()->getJointModelGroup(PLANNING_GROUP_mh5);
    moveit::core::RobotStatePtr current_state_mh5 = move_group_mh5.getCurrentState();
    std::vector<double> joint_group_positions_mh5;
    current_state_mh5->copyJointGroupPositions(joint_model_group_mh5, joint_group_positions_mh5);

    move_group_mh50.setPlannerId("RRTConnect");
    move_group_mh50.setPlanningTime(45);
    move_group_mh5.setPlannerId("RRTConnect");
    move_group_mh5.setPlanningTime(45);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    visual_tools.trigger();
    ROS_INFO_NAMED("move_group_mh50", "Reference frame: %s", move_group_mh50.getPlanningFrame().c_str());
    ROS_INFO_NAMED("move_group_mh50", "End effector link: %s", move_group_mh50.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("move_group_mh5", "Reference frame: %s", move_group_mh5.getPlanningFrame().c_str());
    ROS_INFO_NAMED("move_group_mh5", "End effector link: %s", move_group_mh5.getEndEffectorLink().c_str());

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = false;

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.5;
    move_group_mh50.setPoseTarget(target_pose1);

    success = (move_group_mh50.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_mh50);
    robot_state->setJointGroupPositions(joint_model_group_mh50, my_plan.trajectory_.joint_trajectory.points.back().positions);
    ros::Duration(0.2).sleep();
    visual_tools.publishRobotState(robot_state);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


    move_group_mh50.setStartState(*robot_state);

    joint_group_positions_mh50 = { 3.1, 0.7, -0.2, 0.0, 1.0, -0.4 };
    move_group_mh50.setJointValueTarget(joint_group_positions_mh50);

    success = (move_group_mh50.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_mh50);
    robot_state->setJointGroupPositions(joint_model_group_mh50, my_plan.trajectory_.joint_trajectory.points.back().positions);
    visual_tools.publishRobotState(robot_state);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    std::cout<<"\n pose after move:" << move_group_mh50.getPoseTarget(move_group_mh50.getEndEffectorLink()) << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motoman_movegroup");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    //display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    //state_publisher = node_handle.advertise<moveit_msgs::DisplayRobotState>("/move_group/display_robot_state", 1, true);
    poseCallback();

    ros::waitForShutdown();
    return 0;
}
