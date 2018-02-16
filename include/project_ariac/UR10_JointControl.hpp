#pragma once

#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <vector>

class UR10_JointControl {
 public:
  UR10_JointControl();
  ~UR10_JointControl();
  UR10_JointControl(ros::NodeHandle nh_);
  void jointStateCallback(const sensor_msgs::JointState& msg);
  void gripperAction(const bool action);
  void jointPosePublisher(const std::vector<float>& joints);

 private:
  sensor_msgs::JointState joint_state;
  ros::Publisher joint_trajectory_pub;
  ros::Subscriber joint_state_sub;
  ros::ServiceClient gripper;
  ros::NodeHandle nh;
};

namespace gripper {
const bool OPEN = false;
const bool CLOSE = true;
}  // namespace gripper
