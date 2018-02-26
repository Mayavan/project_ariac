/**
 * @file UR10_Control.cpp
 * @author     Ravi Bhadeshiya
 * @version    0.1
 * @brief      Class for controlling ur10 arm
 *
 * @copyright  BSD 3-Clause License (c) 2018 Ravi Bhadeshiya
 *
 * Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may
be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "project_ariac/UR10_Control.hpp"

UR10_Control::UR10_Control(const ros::NodeHandle& server)
    : ur10_("manipulator") {

  starting_joint_angle_.resize(7);
  server.param("elbow_joint", starting_joint_angle_[0], 0.0);
  server.param("linear_arm_actuator_joint", starting_joint_angle_[1], 3.0);
  server.param("shoulder_lift_joint", starting_joint_angle_[2], -1.0);
  server.param("shoulder_pan_joint", starting_joint_angle_[3], 1.9);
  server.param("wrist_1_joint", starting_joint_angle_[4], 4.0);
  server.param("wrist_2_joint", starting_joint_angle_[5], 4.7);
  server.param("wrist_3_joint", starting_joint_angle_[6], 0.0);

  server.param("z_offSet_pickUp", z_offSet_pickUp_, 0.030);

  std::string planner;
  server.param<std::string>("planner", planner, "RRTConnectkConfigDefault");
  ur10_.setPlannerId(planner);

  double planning_time;
  server.param("planning_time", planning_time, 2.5);
  ur10_.setPlanningTime(planning_time);

  int planning_attempt;
  server.param("planning_attempt", planning_attempt, 20);
  ur10_.setNumPlanningAttempts(planning_attempt);

  gripper_ = nh_.serviceClient<osrf_gear::VacuumGripperControl>(
      "/ariac/gripper/control");

  ros::Duration(0.5).sleep();

  this->goToStart();

  auto transform = this->getTransfrom("/world", "/ee_link");

  home_.orientation.x = transform.getRotation().x();
  home_.orientation.y = transform.getRotation().y();
  home_.orientation.z = transform.getRotation().z();
  home_.orientation.w = transform.getRotation().w();
  target_ = home_;

  transform = this->getTransfrom("/world", "/agv1_load_point_frame");

  agv_.position.x = transform.getOrigin().x();
  agv_.position.y = transform.getOrigin().y();
  agv_.position.z = transform.getOrigin().z() + 2 * z_offSet_pickUp_;
  agv_.orientation = home_.orientation;

  gripper_callback_ = nh_.subscribe("/ariac/gripper/state/", 10,
                                    &UR10_Control::gripperStatusCallback, this);
}

UR10_Control::~UR10_Control() {}

tf::StampedTransform UR10_Control::getTransfrom(const std::string& src,
                                                const std::string& target) {
  tf::StampedTransform transform;
  tf::TransformListener listener;

  listener.waitForTransform(src, target, ros::Time(0), ros::Duration(20));
  try {
    listener.lookupTransform(src, target, ros::Time(0), transform);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(0.5).sleep();
  }

  return transform;
}

void UR10_Control::setTarget(const geometry_msgs::Pose& target) {
  target_.position = target.position;
  target_.position.z += z_offSet_pickUp_;
  ur10_.setPoseTarget(target_);
}

bool UR10_Control::plan() {
  // bool success = ur10_.plan(planner);
  ROS_INFO("Planning start!");
  bool success = (ur10_.plan(planner_) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
    ROS_INFO("Planned success.");
  else
    ROS_INFO("Planned unsucess!");
  return success;
}

void UR10_Control::move() {
  ros::AsyncSpinner spinner(4);
  spinner.start();
  if (this->plan()) {
    ur10_.move();
    // ur10_.execute(planner);
    // ur10_.asyncExecute(planner);
    ros::Duration(0.5).sleep();
  }
}

void UR10_Control::goToStart() {
  // std::vector<double> target = {0, 3 , -1, 1.9, 4, 4.7, 0};
  // ur10_.setJointValueTarget(target);
  ur10_.setJointValueTarget(starting_joint_angle_);
  this->move();
  ros::Duration(0.5).sleep();
}

bool UR10_Control::place() {
  ros::Rate rate(1);
  std::vector<std::vector<double>> poses = {
      {1.76, 0.38, -1.38, 2.76, 3.27, -1.51, 0.00}};  //,
  // {2.76, 0.38, -1.38, 1.5, 3.27, -1.51, 0.00},
  // {2.76, 1.56, -1.38, 1.5, 3.27, -1.51, 0.00},
  // {2.76, 1.56, -0.63, 1.5, 3.27, -1.51, 0.00}};

  for (const auto& itr : poses) {
    ur10_.setJointValueTarget(itr);
    this->move();
    ros::Duration(0.5).sleep();
  }

  this->setTarget(agv_);
  this->move();
  // if gripper attached false before placing return
  // placing failed report
  ros::spinOnce();
  rate.sleep();

  if (gripper_state_.attached == false) {
    for (const auto& itr : poses) {
      ur10_.setJointValueTarget(itr);
      this->move();
      ros::Duration(0.5).sleep();
    }
    return false;
  }

  this->gripperAction(gripper::OPEN);

  ros::Duration(0.5).sleep();

  for (const auto& itr : poses) {
    ur10_.setJointValueTarget(itr);
    this->move();
    ros::Duration(0.5).sleep();
  }

  return true;
}

void UR10_Control::gripperAction(const bool action) {
  osrf_gear::VacuumGripperControl srv;

  srv.request.enable = action ? gripper::CLOSE : gripper::OPEN;

  gripper_.call(srv);

  ros::Duration(1).sleep();

  if (srv.response.success)
    ROS_INFO_STREAM("Gripper Action Successfully Exicuted!");
  else
    ROS_ERROR("Gripper Action Failed!");
}

bool UR10_Control::gripperPickup(const bool action) {
  this->gripperAction(action);

  ros::Rate rate(0.5);
  int count = 0;
  double Delta = 0.005;

  // Check gripper state first
  ros::spinOnce();
  rate.sleep();

  while (!gripper_state_.attached) {
    target_.position.z -= Delta;

    ROS_INFO_STREAM("Picking up:" << target_.position.z << " count:" << count);

    if (count > z_offSet_pickUp_ / Delta) break;
    count++;

    // this->setTarget(target_); // logical mistake
    // we are adding offset while set target
    this->move();

    // Wait for gripper state call back
    ros::spinOnce();
    rate.sleep();
  }

  return gripper_state_.attached;
}

void UR10_Control::gripperStatusCallback(
    const osrf_gear::VacuumGripperState::ConstPtr& gripper_status) {
  gripper_state_.attached = gripper_status->attached;
  gripper_state_.enabled = gripper_status->enabled;
}

bool UR10_Control::pickAndPlace(const geometry_msgs::Pose& target_) {
  // ur10.goToStart();
  ROS_INFO("Setting Target..");
  this->setTarget(target_);

  ROS_INFO("Moving");
  this->move();

  ROS_INFO("Picking Up");

  auto result = this->gripperPickup(gripper::CLOSE);
  // if gripper not able to pick
  if (!result) return result;

  ROS_INFO("Placing");
  this->goToStart();
  result = this->place();

  ROS_INFO("Home");
  this->goToStart();

  return result;
}
