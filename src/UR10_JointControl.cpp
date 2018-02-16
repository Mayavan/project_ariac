#include "project_ariac/UR10_JointControl.hpp"

UR10_JointControl::UR10_JointControl() {
    ROS_WARN("UR10_JointControl init!");
}

UR10_JointControl::~UR10_JointControl() {}

UR10_JointControl::UR10_JointControl(ros::NodeHandle nh_) {
  nh = nh_;
  joint_trajectory_pub =
      nh.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm/command", 10);
  joint_state_sub = nh.subscribe(
      "/ariac/joint_states", 10, &UR10_JointControl::jointStateCallback, this);
  gripper = nh.serviceClient<osrf_gear::VacuumGripperControl>(
      "/ariac/gripper/control");

  ROS_INFO_STREAM("UR10_JointControl init complete!");
}

void UR10_JointControl::jointStateCallback(const sensor_msgs::JointState& msg) {
  joint_state = msg;
}

void UR10_JointControl::gripperAction(const bool action) {
  osrf_gear::VacuumGripperControl srv;

  srv.request.enable = action ? gripper::CLOSE : gripper::OPEN;

  gripper.call(srv);

  if (srv.response.success)
    ROS_INFO_STREAM("Gripper Action Successfully Exicuted!");
  else
    ROS_ERROR("Gripper Action Failed!");
}

void UR10_JointControl::jointPosePublisher(const std::vector<float>& taget_joint) {

  trajectory_msgs::JointTrajectory target_msg;
  target_msg.header.stamp = ros::Time::now();
  target_msg.joint_names = joint_state.name;
  target_msg.joint_names.pop_back();
  target_msg.points.resize(1);
  target_msg.points[0].positions.resize(target_msg.joint_names.size());

  for (int i = 0; i < target_msg.joint_names.size(); ++i) {
    target_msg.points[0].positions[i] = taget_joint[i];
  }

  target_msg.points[0].time_from_start = ros::Duration(1.0);

  ROS_INFO_STREAM("Populated jointPosePublisher msg:\n" << target_msg);

  joint_trajectory_pub.publish(target_msg);
  ros::Duration(0.2).sleep();
}
