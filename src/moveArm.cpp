#include "moveArm.hpp"

#include <vector>
#include <string>

moveArm::moveArm(ros::NodeHandle &nodeHandle): nh( nodeHandle ){
    joint_state_subscriber = nh.subscribe(
            "/ariac/joint_states", 10,
            &moveArm::jointStateCallback, this);
    joint_trajectory_publisher = nh.advertise<trajectory_msgs::JointTrajectory>(
            "/ariac/arm/command", 10);
    gripper = nh.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");
    gripperStateSubscriber = nh.subscribe("/ariac/gripper/state", 10, &moveArm::gripperStateCallback, this);
    called = false;
    attached = false;
    while(!called && ros::ok()) {
        //ROS_INFO("Waiting for joint feedback...");
        ros::spinOnce();
        ros::Duration(0.02).sleep();
    }
    if (!gripper.exists()) {
        gripper.waitForExistence();
    }
    attach.request.enable = 1;
    detach.request.enable = 0;
    arrivalTime = 1;
}

void moveArm::jointStateCallback(const sensor_msgs::JointState::ConstPtr &jointStateMsg)
{
    current_joint_states = *jointStateMsg;
    current_joint_states.name.pop_back();
    current_joint_states.effort.pop_back();
    current_joint_states.position.pop_back();
    current_joint_states.velocity.pop_back();
    called = true;
}

void moveArm::gripperStateCallback(const osrf_gear::VacuumGripperState::ConstPtr &msg) {
    currentGripperState = *msg;
    attached = msg->attached;
}

osrf_gear::VacuumGripperState moveArm::getGripperState() {
    ros::spinOnce();
    return currentGripperState;
}

bool moveArm::isGripperAttached() {
    ros::spinOnce();
    return attached;
}

bool moveArm::waitForGripperAttach(double timeout) {
    timeout = timeout <= 0? FLT_MAX:timeout;
    ros::spinOnce();
    while((!attached) && timeout > 0 && ros::ok()) {
        ROS_INFO("Retry grasp");
        release();
        ros::Duration(0.2).sleep();
        ros::spinOnce();
        grab();
        ros::Duration(0.8).sleep();
        ros::spinOnce();
        timeout -= 1.0;
    }
    return attached;
}

void moveArm::sendJointsValue(std::vector<double> joints) {
    trajectory_msgs::JointTrajectory msg;
    msg.header.stamp = ros::Time::now();
    msg.joint_names = current_joint_states.name;
    msg.points.resize(1);
    msg.points[0].positions = joints;
    msg.points[0].time_from_start = ros::Duration(arrivalTime);
    ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher.publish(msg);
    ros::Duration(arrivalTime).sleep();                         // wait for finish
    ros::spinOnce();
}

std::vector<double> moveArm::getJointsState() {
    called = false;
    while(!called && ros::ok()) {
        //ROS_INFO("Waiting for joint feedback...");
        ros::spinOnce();
        ros::Duration(0.02).sleep();
    }
    std::vector<double> joints = current_joint_states.position;
    return joints;
}

void moveArm::grab() {
    //ROS_INFO("enable gripper");
    gripper.call(attach);
}

void moveArm::release() {
    //ROS_INFO("release gripper");
    gripper.call(detach);
}