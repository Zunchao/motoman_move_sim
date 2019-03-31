
/* Author: Zunchao Zheng*/

#include <iostream>   // std::cout
#include "ros/ros.h"
#include <std_msgs/Int8.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <iostream>
#include <fstream>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h" 

#include <moveit/move_group_interface/move_group_interface.h>
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

void poseCallback()
{
    const std::string PLANNING_GROUP_lwa4p = "arm2";
    moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP_lwa4p);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    bool success = false;
    //group.setPlannerId("RRTConnect");
    //group.setPlanningTime(45);
    group.setNumPlanningAttempts(10);
    //group.allowReplanning();

    tf2::Quaternion gQuaternion;
    geometry_msgs::Pose target_pose1;

    sleep(5.0);

    //group.setMaxVelocityScalingFactor(0.1);
    group.setStartStateToCurrentState();
    gQuaternion.setRPY( -M_PI/3, M_PI/6, 0 );
    gQuaternion.normalize();
    target_pose1.orientation.w = gQuaternion.w();
    target_pose1.orientation.x = gQuaternion.x();
    target_pose1.orientation.y = gQuaternion.y();
    target_pose1.orientation.z = gQuaternion.z();
    target_pose1.position.x = 0.1;
    target_pose1.position.y = 0.5;
    target_pose1.position.z = 0.5;
    group.setPoseTarget(target_pose1);

    success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    if (success){
        group.execute(my_plan);
        sleep(5.0);
        ROS_INFO_STREAM("manipulator Moving done.");
    }
    else{
        ROS_WARN_STREAM("manipulator Moving failed.");
    }
/*
    group.setStartStateToCurrentState();
    gQuaternion.setRPY( 0, M_PI/2, 0 );
    gQuaternion.normalize();
    target_pose1.orientation.w = gQuaternion.w();
    target_pose1.orientation.x = gQuaternion.x();
    target_pose1.orientation.y = gQuaternion.y();
    target_pose1.orientation.z = gQuaternion.z();
    target_pose1.position.x = 0.34;
    target_pose1.position.y = 0.16;
    target_pose1.position.z = 0.60;
    group.setPoseTarget(target_pose1);

    success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s", success ? "" : "FAILED");

    if (success){
        group.execute(my_plan);
        sleep(10.0);
        ROS_INFO_STREAM("manipulator Moving done.");
    }
    else{
        ROS_WARN_STREAM("manipulator Moving failed.");
    }

    group.setStartStateToCurrentState();
    gQuaternion.setRPY( 0, M_PI/2, 0 );
    gQuaternion.normalize();
    target_pose1.orientation.w = gQuaternion.w();
    target_pose1.orientation.x = gQuaternion.x();
    target_pose1.orientation.y = gQuaternion.y();
    target_pose1.orientation.z = gQuaternion.z();
    target_pose1.position.x = 0.4;
    target_pose1.position.y = -0.1;
    target_pose1.position.z = 0.40;
    group.setPoseTarget(target_pose1);

    success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (pose goal) %s", success ? "" : "FAILED");

    if (success){
        group.execute(my_plan);
        sleep(5.0);
        ROS_INFO_STREAM("manipulator Moving done.");
    }
    else{
        ROS_WARN_STREAM("manipulator Moving failed.");
    }

*/
    group.setStartStateToCurrentState();
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "arm_6_link";
    ocm.header.frame_id = "world";
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
    group.setPathConstraints(test_constraints);

    geometry_msgs::Pose target_pose3 = target_pose1;
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose3);

    target_pose3.position.z -= 0.3;
    target_pose3.position.y -= 0.1;
    waypoints.push_back(target_pose3);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    my_plan.trajectory_ = trajectory;
    success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (pose goal) %s", success ? "" : "FAILED");

    if (success){
        group.execute(my_plan);
        group.move();
        sleep(5.0);
        ROS_INFO_STREAM("manipulator Moving done.");
    }
    else{
        ROS_WARN_STREAM("manipulator Moving failed.");
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lwa4p_move");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    poseCallback();

    ros::waitForShutdown();
    return 0;
}


