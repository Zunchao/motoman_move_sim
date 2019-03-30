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

void poseCallback_third()
{
    const std::string PLANNING_GROUP_pandaarm = "panda_arm";
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group_pandaarm(PLANNING_GROUP_pandaarm);
    const robot_state::JointModelGroup* joint_model_group_pandaarm =
            move_group_pandaarm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_pandaarm);
    std::vector<double> link_state_pandaarm_ori;

    move_group_pandaarm.setPlannerId("RRTConnect");

    tf2::Quaternion gQuaternion;
    std::vector<float> gOrient;

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = false;
    //path1
    move_group_pandaarm.setStartStateToCurrentState();
    gQuaternion.setRPY( 0, -M_PI,  0 );
    //( -M_PI, -M_PI,  0 )up
    //M_PI/2, -M_PI,  0left
    gQuaternion.normalize();
    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = gQuaternion.w();
    target_pose.orientation.x = gQuaternion.x();
    target_pose.orientation.y = gQuaternion.y();
    target_pose.orientation.z = gQuaternion.z();
    target_pose.position.x = 0.5;
    target_pose.position.y = 0.2;
    target_pose.position.z = 0.25;
    move_group_pandaarm.setPoseTarget(target_pose);

    success = (move_group_pandaarm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    std::cout << "target pose :" << target_pose.orientation << std::endl;
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose, "pose1");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_pandaarm);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to excute path 1");
    if (success){
        move_group_pandaarm.execute(my_plan);
        sleep(5.0);
        ROS_INFO_STREAM("manipulator Moving done.");
    }
    else{
        ROS_WARN_STREAM("manipulator Moving failed.");
    }

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to plan 2");
    // path 2
    move_group_pandaarm.setStartStateToCurrentState();

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
    waypoints.push_back(target_pose3);
    target_pose3.position.x -= 0.2;
    waypoints.push_back(target_pose3);
    target_pose3.position.y += 0.4;
    waypoints.push_back(target_pose3);
    target_pose3.position.x += 0.2;
    waypoints.push_back(target_pose3);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    //The computed trajectory is too short to detect jumps in joint-space Need at least 10 steps, only got 3. Try a lower max_step.
    double fraction = move_group_pandaarm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    my_plan.trajectory_ = trajectory;
    success = (move_group_pandaarm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s", success ? "" : "FAILED");
    visual_tools.deleteAllMarkers();
    for (std::size_t i = 0; i < waypoints.size(); ++i){
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);}
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to excute path 2");
    if (success){
        move_group_pandaarm.execute(my_plan);
        sleep(5.0);
        ROS_INFO_STREAM("manipulator Moving done.");
    }
    else{
        ROS_WARN_STREAM("manipulator Moving failed.");
    }

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to plan 3");
    move_group_pandaarm.setStartStateToCurrentState();
    gQuaternion.setRPY( -M_PI/2,-M_PI/2,  0);
    gQuaternion.normalize();
    std::vector<geometry_msgs::Pose> waypoints2;
    waypoints2.push_back(target_pose3);
    ocm.orientation.w = gQuaternion.w();
    ocm.orientation.x = gQuaternion.x();
    ocm.orientation.y = gQuaternion.y();
    ocm.orientation.z = gQuaternion.z();
    test_constraints.orientation_constraints.push_back(ocm);
    move_group_pandaarm.setPathConstraints(test_constraints);

    target_pose3.position.y += 0.3;
    target_pose3.position.x -= 0.2;
    target_pose3.position.z += 0.35;
    target_pose3.orientation.w = gQuaternion.w();
    target_pose3.orientation.x = gQuaternion.x();
    target_pose3.orientation.y = gQuaternion.y();
    target_pose3.orientation.z = gQuaternion.z();
    waypoints2.push_back(target_pose3);

    move_group_pandaarm.setMaxVelocityScalingFactor(0.1);
    fraction = move_group_pandaarm.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    my_plan.trajectory_ = trajectory;
    success = (move_group_pandaarm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (pose goal) %s", success ? "" : "FAILED");
    visual_tools.deleteAllMarkers();
    for (std::size_t i = 0; i < waypoints.size(); ++i){
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);}
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to excute path 3");
    if (success){
        move_group_pandaarm.execute(my_plan);
        sleep(5.0);
        ROS_INFO_STREAM("manipulator Moving done.");
    }
    else{
        ROS_WARN_STREAM("manipulator Moving failed.");
    }
}
void poseCallback_new()
{
    const std::string PLANNING_GROUP_pandaarm = "panda_arm";
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group_pandaarm(PLANNING_GROUP_pandaarm);
    const robot_state::JointModelGroup* joint_model_group_pandaarm =
            move_group_pandaarm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_pandaarm);
    std::vector<double> link_state_pandaarm_ori;

    move_group_pandaarm.setPlannerId("RRTConnect");

    tf2::Quaternion gQuaternion;
    std::vector<float> gOrient;

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = false;
    //path1
    move_group_pandaarm.setStartStateToCurrentState();
    gQuaternion.setRPY( -M_PI, -M_PI/2, 0 );
    gQuaternion.normalize();
    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = gQuaternion.w();
    target_pose.orientation.x = gQuaternion.x();
    target_pose.orientation.y = gQuaternion.y();
    target_pose.orientation.z = gQuaternion.z();
    target_pose.position.x = 0.35;
    target_pose.position.y = 0.2;
    target_pose.position.z = 0.75;
    move_group_pandaarm.setPoseTarget(target_pose);

    success = (move_group_pandaarm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    std::cout << "target pose :" << target_pose.orientation << std::endl;
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose, "pose1");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_pandaarm);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    if (success){
        move_group_pandaarm.execute(my_plan);
        sleep(5.0);
        ROS_INFO_STREAM("manipulator Moving done.");
    }
    else{
        ROS_WARN_STREAM("manipulator Moving failed.");
    }

    // path 2
    move_group_pandaarm.setStartStateToCurrentState();

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

    target_pose3.position.y -= 0.3;
    target_pose3.position.x -= 0.1;
    waypoints.push_back(target_pose3);
    target_pose3.position.z -= 0.1;
    waypoints.push_back(target_pose3);
    target_pose3.position.y += 0.3;
    target_pose3.position.x += 0.1;
    waypoints.push_back(target_pose3);
    target_pose3.position.z += 0.1;
    waypoints.push_back(target_pose3);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.01;
    const double eef_step = 0.01;
    double fraction = move_group_pandaarm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    my_plan.trajectory_ = trajectory;
    success = (move_group_pandaarm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s", success ? "" : "FAILED");
    visual_tools.deleteAllMarkers();
    for (std::size_t i = 0; i < waypoints.size(); ++i){
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);}
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    if (success){
        move_group_pandaarm.execute(my_plan);
        sleep(5.0);
        ROS_INFO_STREAM("manipulator Moving done.");
    }
    else{
        ROS_WARN_STREAM("manipulator Moving failed.");
    }

    move_group_pandaarm.setStartStateToCurrentState();
    gQuaternion.setRPY( -M_PI/2,-M_PI/2,  0);
    gQuaternion.normalize();
    std::vector<geometry_msgs::Pose> waypoints2;
    waypoints2.push_back(target_pose3);
    ocm.orientation.w = gQuaternion.w();
    ocm.orientation.x = gQuaternion.x();
    ocm.orientation.y = gQuaternion.y();
    ocm.orientation.z = gQuaternion.z();
    test_constraints.orientation_constraints.push_back(ocm);
    move_group_pandaarm.setPathConstraints(test_constraints);

    target_pose3.position.y += 0.5;
    target_pose3.position.x -= 0.3;
    target_pose3.position.z -= 0.1;
    target_pose3.orientation.w = gQuaternion.w();
    target_pose3.orientation.x = gQuaternion.x();
    target_pose3.orientation.y = gQuaternion.y();
    target_pose3.orientation.z = gQuaternion.z();
    waypoints2.push_back(target_pose3);

    move_group_pandaarm.setMaxVelocityScalingFactor(0.1);
    fraction = move_group_pandaarm.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    my_plan.trajectory_ = trajectory;
    success = (move_group_pandaarm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (pose goal) %s", success ? "" : "FAILED");
    visual_tools.deleteAllMarkers();
    for (std::size_t i = 0; i < waypoints.size(); ++i){
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);}
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    if (success){
        move_group_pandaarm.execute(my_plan);
        sleep(5.0);
        ROS_INFO_STREAM("manipulator Moving done.");
    }
    else{
        ROS_WARN_STREAM("manipulator Moving failed.");
    }

    traj_file.close();
}
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
    //move_group_pandaarm.setPlanningTime(45);

    tf2::Quaternion gQuaternion;
    std::vector<float> gOrient;

    traj_file.open ("/home/zheng/test_ws/src/motoman_move_sim/panda_move/results/traj_results.csv");

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

    gQuaternion.setRPY( -M_PI, -M_PI/2, 0 );
    gQuaternion.normalize();
    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = gQuaternion.w();
    target_pose.orientation.x = gQuaternion.x();
    target_pose.orientation.y = gQuaternion.y();
    target_pose.orientation.z = gQuaternion.z();
    target_pose.position.x = 0.35;
    target_pose.position.y = 0.2;
    target_pose.position.z = 0.75;
    move_group_pandaarm.setPoseTarget(target_pose);
    std::cout << "target pose :" << target_pose.orientation << std::endl;

    success = (move_group_pandaarm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    current_state_pandaarm->enforceBounds();
    traj_file <<  "Traj No. 1" << std::endl;
    for (std::size_t i = 0; i < my_plan.trajectory_.joint_trajectory.points.size(); ++i){
        traj_file <<  "Traj No.1 pathpoint " << i << std::endl;
        for(std::size_t j = 0; j < joint_names_pandaarm.size(); ++j){
            joint_group_positions_pandaarm[j] = my_plan.trajectory_.joint_trajectory.points[i].positions[j];
            current_state_pandaarm->setJointGroupPositions(joint_model_group_pandaarm, joint_group_positions_pandaarm);
            current_state_pandaarm->setVariablePositions(move_group_pandaarm.getJointNames(), joint_group_positions_pandaarm);
            current_state_pandaarm->getGlobalLinkTransform(move_group_pandaarm.getLinkNames()[j]);
            //current_state_pandaarm->getJointTransform(move_group_pandaarm.getJointNames()[j]);
            current_state_pandaarm->updateLinkTransforms();
            current_state_pandaarm->update();
            }
        traj_file << move_group_pandaarm.getLinkNames()[0] << " position : " << link_state_pandaarm_0.translation().transpose() << std::endl;
        traj_file << " rotation matrix : " << std::endl;
        traj_file << link_state_pandaarm_0.rotation() << std::endl;
        traj_file << move_group_pandaarm.getLinkNames()[1] << " position : " << link_state_pandaarm_1.translation().transpose() << std::endl;
        traj_file << " rotation matrix : " << std::endl;
        traj_file << link_state_pandaarm_1.rotation() << std::endl;
        traj_file << move_group_pandaarm.getLinkNames()[2] << " position : " << link_state_pandaarm_2.translation().transpose() << std::endl;
        traj_file << " rotation matrix : " << std::endl;
        traj_file << link_state_pandaarm_2.rotation() << std::endl;
        traj_file << move_group_pandaarm.getLinkNames()[3] << " position : " << link_state_pandaarm_3.translation().transpose() << std::endl;
        traj_file << " rotation matrix : " << std::endl;
        traj_file << link_state_pandaarm_3.rotation() << std::endl;
        traj_file << move_group_pandaarm.getLinkNames()[4] << " position : " << link_state_pandaarm_4.translation().transpose() << std::endl;
        traj_file << " rotation matrix : " << std::endl;
        traj_file << link_state_pandaarm_4.rotation() << std::endl;
        traj_file << move_group_pandaarm.getLinkNames()[5] << " position : " << link_state_pandaarm_5.translation().transpose() << std::endl;
        traj_file << " rotation matrix : " << std::endl;
        traj_file << link_state_pandaarm_5.rotation() << std::endl;
        traj_file << move_group_pandaarm.getLinkNames()[6] << " position : " << link_state_pandaarm_6.translation().transpose() << std::endl;
        traj_file << " rotation matrix : " << std::endl;
        traj_file << link_state_pandaarm_6.rotation() << std::endl;
    }

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose, "pose1");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_pandaarm);
    robot_state->setJointGroupPositions(joint_model_group_pandaarm, my_plan.trajectory_.joint_trajectory.points.back().positions);
    visual_tools.publishRobotState(robot_state);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    if (success){
        move_group_pandaarm.execute(my_plan);
        sleep(5.0);
        ROS_INFO_STREAM("manipulator Moving done.");
    }
    else{
        ROS_WARN_STREAM("manipulator Moving failed.");
    }

    move_group_pandaarm.setStartState(*robot_state);

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

    target_pose3.position.y -= 0.3;
    target_pose3.position.x -= 0.1;
    waypoints.push_back(target_pose3);
    target_pose3.position.z -= 0.1;
    waypoints.push_back(target_pose3);
    target_pose3.position.y += 0.3;
    target_pose3.position.x += 0.1;
    waypoints.push_back(target_pose3);
    target_pose3.position.z += 0.1;
    waypoints.push_back(target_pose3);

    //move_group_pandaarm.setMaxVelocityScalingFactor(0.01);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_pandaarm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    traj_file <<  "Traj No. 2" << std::endl;
    for (std::size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i){
        traj_file <<  "Traj No.2 pathpoint " << i << std::endl;
        for(std::size_t j = 0; j < joint_names_pandaarm.size(); ++j){
            joint_group_positions_pandaarm[j] = trajectory.joint_trajectory.points[i].positions[j];
            current_state_pandaarm->setJointGroupPositions(joint_model_group_pandaarm, joint_group_positions_pandaarm);
            current_state_pandaarm->getGlobalLinkTransform(move_group_pandaarm.getLinkNames()[j]);
            current_state_pandaarm->updateLinkTransforms();
            current_state_pandaarm->update();
        }
        traj_file << move_group_pandaarm.getLinkNames()[0] << " position : " << link_state_pandaarm_0.translation().transpose() << std::endl;
        traj_file << " rotation matrix : " << std::endl;
        traj_file << link_state_pandaarm_0.rotation() << std::endl;
        traj_file << move_group_pandaarm.getLinkNames()[1] << " position : " << link_state_pandaarm_1.translation().transpose() << std::endl;
        traj_file << " rotation matrix : " << std::endl;
        traj_file << link_state_pandaarm_1.rotation() << std::endl;
        traj_file << move_group_pandaarm.getLinkNames()[2] << " position : " << link_state_pandaarm_2.translation().transpose() << std::endl;
        traj_file << " rotation matrix : " << std::endl;
        traj_file << link_state_pandaarm_2.rotation() << std::endl;
        traj_file << move_group_pandaarm.getLinkNames()[3] << " position : " << link_state_pandaarm_3.translation().transpose() << std::endl;
        traj_file << " rotation matrix : " << std::endl;
        traj_file << link_state_pandaarm_3.rotation() << std::endl;
        traj_file << move_group_pandaarm.getLinkNames()[4] << " position : " << link_state_pandaarm_4.translation().transpose() << std::endl;
        traj_file << " rotation matrix : " << std::endl;
        traj_file << link_state_pandaarm_4.rotation() << std::endl;
        traj_file << move_group_pandaarm.getLinkNames()[5] << " position : " << link_state_pandaarm_5.translation().transpose() << std::endl;
        traj_file << " rotation matrix : " << std::endl;
        traj_file << link_state_pandaarm_5.rotation() << std::endl;
        traj_file << move_group_pandaarm.getLinkNames()[6] << " position : " << link_state_pandaarm_6.translation().transpose() << std::endl;
        traj_file << " rotation matrix : " << std::endl;
        traj_file << link_state_pandaarm_6.rotation() << std::endl;
    }
    robot_state->setJointGroupPositions(joint_model_group_pandaarm, trajectory.joint_trajectory.points.back().positions);
    my_plan.trajectory_ = trajectory;
    success = (move_group_pandaarm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s", success ? "" : "FAILED");
    visual_tools.deleteAllMarkers();
    for (std::size_t i = 0; i < waypoints.size(); ++i){
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);}
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    if (success){
        move_group_pandaarm.execute(my_plan);
        sleep(5.0);
        ROS_INFO_STREAM("manipulator Moving done.");
    }
    else{
        ROS_WARN_STREAM("manipulator Moving failed.");
    }
    /**/

    gQuaternion.setRPY( -M_PI/2,-M_PI/2,  0);
    gQuaternion.normalize();
    std::vector<geometry_msgs::Pose> waypoints2;
    waypoints2.push_back(target_pose3);

    target_pose3.position.y += 0.5;
    target_pose3.position.x -= 0.3;
    target_pose3.position.z -= 0.1;
    target_pose3.orientation.w = gQuaternion.w();
    target_pose3.orientation.x = gQuaternion.x();
    target_pose3.orientation.y = gQuaternion.y();
    target_pose3.orientation.z = gQuaternion.z();
    waypoints2.push_back(target_pose3);

    move_group_pandaarm.setMaxVelocityScalingFactor(0.1);
    fraction = move_group_pandaarm.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    traj_file <<  "Traj No. 3" << std::endl;
    for (std::size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i){
        traj_file <<  "Traj No.3 pathpoint " << i << std::endl;
        for(std::size_t j = 0; j < joint_names_pandaarm.size(); ++j){
            joint_group_positions_pandaarm[j] = trajectory.joint_trajectory.points[i].positions[j];
            current_state_pandaarm->setJointGroupPositions(joint_model_group_pandaarm, joint_group_positions_pandaarm);
            current_state_pandaarm->getGlobalLinkTransform(move_group_pandaarm.getLinkNames()[j]);
            current_state_pandaarm->updateLinkTransforms();
            current_state_pandaarm->update();
        }
        traj_file << move_group_pandaarm.getLinkNames()[0] << " position : " << link_state_pandaarm_0.translation().transpose() << std::endl;
        traj_file << " rotation matrix : " << std::endl;
        traj_file << link_state_pandaarm_0.rotation() << std::endl;
        traj_file << move_group_pandaarm.getLinkNames()[1] << " position : " << link_state_pandaarm_1.translation().transpose() << std::endl;
        traj_file << " rotation matrix : " << std::endl;
        traj_file << link_state_pandaarm_1.rotation() << std::endl;
        traj_file << move_group_pandaarm.getLinkNames()[2] << " position : " << link_state_pandaarm_2.translation().transpose() << std::endl;
        traj_file << " rotation matrix : " << std::endl;
        traj_file << link_state_pandaarm_2.rotation() << std::endl;
        traj_file << move_group_pandaarm.getLinkNames()[3] << " position : " << link_state_pandaarm_3.translation().transpose() << std::endl;
        traj_file << " rotation matrix : " << std::endl;
        traj_file << link_state_pandaarm_3.rotation() << std::endl;
        traj_file << move_group_pandaarm.getLinkNames()[4] << " position : " << link_state_pandaarm_4.translation().transpose() << std::endl;
        traj_file << " rotation matrix : " << std::endl;
        traj_file << link_state_pandaarm_4.rotation() << std::endl;
        traj_file << move_group_pandaarm.getLinkNames()[5] << " position : " << link_state_pandaarm_5.translation().transpose() << std::endl;
        traj_file << " rotation matrix : " << std::endl;
        traj_file << link_state_pandaarm_5.rotation() << std::endl;
        traj_file << move_group_pandaarm.getLinkNames()[6] << " position : " << link_state_pandaarm_6.translation().transpose() << std::endl;
        traj_file << " rotation matrix : " << std::endl;
        traj_file << link_state_pandaarm_6.rotation() << std::endl;
    }
    my_plan.trajectory_ = trajectory;
    success = (move_group_pandaarm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (pose goal) %s", success ? "" : "FAILED");
    visual_tools.deleteAllMarkers();
    robot_state->setJointGroupPositions(joint_model_group_pandaarm, trajectory.joint_trajectory.points.back().positions);
    visual_tools.publishRobotState(robot_state);
    for (std::size_t i = 0; i < waypoints.size(); ++i){
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);}
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    if (success){
        move_group_pandaarm.execute(my_plan);
        sleep(5.0);
        ROS_INFO_STREAM("manipulator Moving done.");
    }
    else{
        ROS_WARN_STREAM("manipulator Moving failed.");
    }

    traj_file.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_sim_movegroup");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    //display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    //state_publisher = node_handle.advertise<moveit_msgs::DisplayRobotState>("/move_group/display_robot_state", 1, true);
    poseCallback_third();
    //poseCallback_new();
    ros::waitForShutdown();
    return 0;
}
