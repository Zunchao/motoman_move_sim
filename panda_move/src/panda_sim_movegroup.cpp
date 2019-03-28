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
    const std::string PLANNING_GROUP_pandaarm = "panda_arm";

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    moveit::planning_interface::MoveGroupInterface move_group_pandaarm(PLANNING_GROUP_pandaarm);
    const robot_state::JointModelGroup* joint_model_group_pandaarm =
            move_group_pandaarm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_pandaarm);
    moveit::core::RobotStatePtr current_state_pandaarm = move_group_pandaarm.getCurrentState();
    std::vector<double> joint_group_positions_pandaarm;
    current_state_pandaarm->copyJointGroupPositions(joint_model_group_pandaarm, joint_group_positions_pandaarm);

    const std::vector<std::string> &joint_names_pandaarm = joint_model_group_pandaarm->getJointModelNames();

    const Eigen::Affine3d &end_effector_state_pandaarm = current_state_pandaarm->getGlobalLinkTransform(move_group_pandaarm.getEndEffectorLink());

    const Eigen::Affine3d &link_state_pandaarm_0 = current_state_pandaarm->getGlobalLinkTransform(move_group_pandaarm.getLinkNames()[0]);
    const Eigen::Affine3d &link_state_pandaarm_1 = current_state_pandaarm->getGlobalLinkTransform(move_group_pandaarm.getLinkNames()[1]);
    const Eigen::Affine3d &link_state_pandaarm_2 = current_state_pandaarm->getGlobalLinkTransform(move_group_pandaarm.getLinkNames()[2]);
    const Eigen::Affine3d &link_state_pandaarm_3 = current_state_pandaarm->getGlobalLinkTransform(move_group_pandaarm.getLinkNames()[3]);
    const Eigen::Affine3d &link_state_pandaarm_4 = current_state_pandaarm->getGlobalLinkTransform(move_group_pandaarm.getLinkNames()[4]);
    const Eigen::Affine3d &link_state_pandaarm_5 = current_state_pandaarm->getGlobalLinkTransform(move_group_pandaarm.getLinkNames()[5]);
    const Eigen::Affine3d &link_state_pandaarm_6 = current_state_pandaarm->getGlobalLinkTransform(move_group_pandaarm.getLinkNames()[6]);
    std::vector<double> link_state_pandaarm_ori;

    move_group_pandaarm.setPlannerId("RRTConnect");
    move_group_pandaarm.setPlanningTime(45);

    tf2::Quaternion gQuaternion;
    std::vector<float> gOrient;

    sleep(10.0);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
    ROS_INFO_NAMED("move_group_pandaarm", "Reference frame: %s", move_group_pandaarm.getPlanningFrame().c_str());
    ROS_INFO_NAMED("move_group_pandaarm", "End effector link: %s", move_group_pandaarm.getEndEffectorLink().c_str());
    std::cout<<"\n pandaarm pose after move:" << move_group_pandaarm.getCurrentPose(move_group_pandaarm.getEndEffectorLink()) << std::endl;

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = false;
    geometry_msgs::Pose target_pose;
    // path 1
    move_group_pandaarm.setStartStateToCurrentState();
    gQuaternion.setRPY( -M_PI, -M_PI/2, 0 );
    gQuaternion.normalize();
    target_pose.orientation.w = gQuaternion.w();
    target_pose.orientation.x = gQuaternion.x();
    target_pose.orientation.y = gQuaternion.y();
    target_pose.orientation.z = gQuaternion.z();
    target_pose.position.x = 0.55;
    target_pose.position.y = 0.25;
    target_pose.position.z = 0.65;
    move_group_pandaarm.setPoseTarget(target_pose);

    success = (move_group_pandaarm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    visual_tools.publishRobotState(robot_state);

    if (success){
        //move_group_pandaarm.execute(my_plan);
        sleep(10.0);
        ROS_INFO_STREAM("manipulator Moving done.");
    }
    else{
        ROS_WARN_STREAM("manipulator Moving failed.");
    }

    // path 2
    current_state_pandaarm->enforceBounds();    
    move_group_pandaarm.setStartState(*robot_state);
    //move_group_pandaarm.setStartStateToCurrentState();

    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "panda_link8";
    ocm.header.frame_id = "panda_link0";
    ocm.orientation.w = gQuaternion.w();
    ocm.orientation.x = gQuaternion.x();
    ocm.orientation.y = gQuaternion.y();
    ocm.orientation.z = gQuaternion.z();
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;
    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group_pandaarm.setPathConstraints(test_constraints);

    geometry_msgs::Pose target_pose3 = target_pose;
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose3);

    target_pose3.position.y -= 0.4;
    target_pose3.position.x -= 0.01;
    waypoints.push_back(target_pose3);
    target_pose3.position.z -= 0.15;
    waypoints.push_back(target_pose3);
    target_pose3.position.y += 0.4;
    target_pose3.position.x += 0.01;
    waypoints.push_back(target_pose3);
    target_pose3.position.z += 0.15;
    waypoints.push_back(target_pose3);

    move_group_pandaarm.setMaxVelocityScalingFactor(0.1);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.01;
    const double eef_step = 0.01;
    double fraction = move_group_pandaarm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    visual_tools.deleteAllMarkers();
    for (std::size_t i = 0; i < waypoints.size(); ++i){
        //waypoints[i].position.y -= 0.27;
        //waypoints[i].position.x -= 0.4;
        //waypoints[i].position.z += 0.15;
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);}
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.publishRobotState(robot_state);

    my_plan.trajectory_ = trajectory;
    success = (move_group_pandaarm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s", success ? "" : "FAILED");

    if (success){
        //move_group_pandaarm.execute(my_plan);
        sleep(5.0);
        ROS_INFO_STREAM("manipulator Moving done.");
    }
    else{
        ROS_WARN_STREAM("manipulator Moving failed.");
    }


    // path 4
    move_group_pandaarm.setStartState(*robot_state);
    //move_group_pandaarm.setStartStateToCurrentState();
    gQuaternion.setRPY( -M_PI/30, -M_PI/3, 0 );
    gQuaternion.normalize();
    target_pose.orientation.w = gQuaternion.w();
    target_pose.orientation.x = gQuaternion.x();
    target_pose.orientation.y = gQuaternion.y();
    target_pose.orientation.z = gQuaternion.z();
    target_pose.position.x = 0.4;
    target_pose.position.y = 0.3;
    target_pose.position.z = 0.6;
    move_group_pandaarm.setPoseTarget(target_pose);
    std::cout << "target pose :" << target_pose.orientation << std::endl;

    success = (move_group_pandaarm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    current_state_pandaarm->enforceBounds();

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose, "pose1");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_pandaarm);
    robot_state->setJointGroupPositions(joint_model_group_pandaarm, my_plan.trajectory_.joint_trajectory.points.back().positions);
    my_plan.trajectory_ = trajectory;
    success = (move_group_pandaarm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s", success ? "" : "FAILED");

    if (success){
        //move_group_pandaarm.execute(my_plan);
        sleep(5.0);
        ROS_INFO_STREAM("manipulator Moving done.");
    }
    else{
        ROS_WARN_STREAM("manipulator Moving failed.");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_sim_movegroup");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    //state_publisher = node_handle.advertise<moveit_msgs::DisplayRobotState>("/move_group/display_robot_state", 1, true);
    poseCallback();

    ros::waitForShutdown();
    return 0;
}
