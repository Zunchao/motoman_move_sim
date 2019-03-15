/* Author: Zunchao Zheng */

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <iostream>
#include <fstream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace std;
std::ofstream traj_file;
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

    const std::vector<std::string> &joint_names_mh50 = joint_model_group_mh50->getJointModelNames();
    const std::vector<std::string> &joint_names_mh5 = joint_model_group_mh5->getJointModelNames();
    const Eigen::Affine3d &end_effector_state_mh50 = current_state_mh50->getGlobalLinkTransform(move_group_mh50.getEndEffectorLink());
    const Eigen::Affine3d &end_effector_state_mh5 = current_state_mh50->getGlobalLinkTransform(move_group_mh5.getEndEffectorLink());

    //move_group_mh50.setPlannerId("RRTConnect");
    move_group_mh50.setPlanningTime(45);
    move_group_mh5.setPlannerId("RRTConnect");
    move_group_mh5.setPlanningTime(45);

    tf2::Quaternion gQuaternion;
    std::vector<float> gOrient;

    traj_file.open ("/home/zheng/test_ws/src/motoman_move_sim/motoman_move_glass/results/traj_results.csv");

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
    ROS_INFO_NAMED("move_group_mh50", "Reference frame: %s", move_group_mh50.getPlanningFrame().c_str());
    ROS_INFO_NAMED("move_group_mh50", "End effector link: %s", move_group_mh50.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("move_group_mh5", "Reference frame: %s", move_group_mh5.getPlanningFrame().c_str());
    ROS_INFO_NAMED("move_group_mh5", "End effector link: %s", move_group_mh5.getEndEffectorLink().c_str());
    std::cout<<"\n mh50 pose after move:" << move_group_mh50.getCurrentPose(move_group_mh50.getEndEffectorLink()) << std::endl;
    std::cout<<"\n mh5 pose after move:" << move_group_mh5.getCurrentPose(move_group_mh5.getEndEffectorLink()) << std::endl;

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = false;

    gQuaternion.setRPY( 0, 0, -M_PI/2 );
    gQuaternion.normalize();
    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = gQuaternion.w();
    target_pose.orientation.x = gQuaternion.x();
    target_pose.orientation.y = gQuaternion.y();
    target_pose.orientation.z = gQuaternion.z();
    target_pose.position.x = 0.5;
    target_pose.position.y = -1.5;
    target_pose.position.z = 0.9;
    move_group_mh50.setPoseTarget(target_pose);

    success = (move_group_mh50.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    for (std::size_t i = 0; i < my_plan.trajectory_.joint_trajectory.points.size(); ++i){
        for(std::size_t j = 0; j < joint_names_mh50.size(); ++j){
            joint_group_positions_mh50[j] = my_plan.trajectory_.joint_trajectory.points[i].positions[j];
        }
        current_state_mh50->setJointGroupPositions(joint_model_group_mh50, joint_group_positions_mh50);
        std::cout << *current_state_mh50->getGlobalLinkTransform(move_group_mh50.getEndEffectorLink()).data() << std::endl;
    }


    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose, "pose1");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_mh50);
    robot_state->setJointGroupPositions(joint_model_group_mh50, my_plan.trajectory_.joint_trajectory.points.back().positions);
    ros::Duration(2).sleep();
    visual_tools.publishRobotState(robot_state);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    move_group_mh50.setStartState(*robot_state);

    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "motoman_mh50_link_6_t";
    ocm.header.frame_id = "world";
    ocm.orientation.y = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;
    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group_mh50.setPathConstraints(test_constraints);

    geometry_msgs::Pose target_pose3 = target_pose;
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose3);

    target_pose3.position.x += 0.8;
    waypoints.push_back(target_pose3);
    target_pose3.position.z -= 0.3;
    waypoints.push_back(target_pose3);
    target_pose3.position.x -= 0.8;
    waypoints.push_back(target_pose3);
    target_pose3.position.z += 0.3;
    waypoints.push_back(target_pose3);

    move_group_mh50.setMaxVelocityScalingFactor(0.1);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.00;
    const double eef_step = 0.02;
    double fraction = move_group_mh50.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    visual_tools.deleteAllMarkers();
    for (std::size_t i = 0; i < waypoints.size(); ++i){
        waypoints[i].position.y -= 0.27;
        waypoints[i].position.x -= 0.4;
        waypoints[i].position.z += 0.15;
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);}
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    /**/

    gQuaternion.setRPY( 0,M_PI/2,  0);
    gQuaternion.normalize();
    std::vector<geometry_msgs::Pose> waypoints2;
    waypoints2.push_back(target_pose3);

    target_pose3.position.y += 1.5;
    target_pose3.position.x -= 0.6;
    target_pose3.orientation.w = gQuaternion.w();
    target_pose3.orientation.x = gQuaternion.x();
    target_pose3.orientation.y = gQuaternion.y();
    target_pose3.orientation.z = gQuaternion.z();
    waypoints2.push_back(target_pose3);
    target_pose3.position.y += 1.5;
    target_pose3.position.x += 0.6;
    target_pose3.orientation.w = gQuaternion.w();
    target_pose3.orientation.x = gQuaternion.x();
    target_pose3.orientation.y = gQuaternion.y();
    target_pose3.orientation.z = gQuaternion.z();
    waypoints2.push_back(target_pose3);

    move_group_mh50.setMaxVelocityScalingFactor(0.1);
    fraction = move_group_mh50.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    visual_tools.deleteAllMarkers();
    robot_state->setJointGroupPositions(joint_model_group_mh50, trajectory.joint_trajectory.points.back().positions);
    ros::Duration(10).sleep();
    visual_tools.publishRobotState(robot_state);

    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    traj_file.close();
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
