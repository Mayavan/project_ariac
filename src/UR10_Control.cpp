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

UR10_Control::UR10_Control() : ur10_("manipulator") {
  // ur10_.setPlanningTime(5);
  ur10_.setPlanningTime(2.5);
  ur10_.setNumPlanningAttempts(20);
  ur10_.setPlannerId("RRTConnectkConfigDefault");

  ros::Duration(1.0).sleep();

  this->goToStart();

  auto transform = this->getTransfrom("/world", "/ee_link");

  home_.orientation.x = transform.getRotation().x();
  home_.orientation.y = transform.getRotation().y();
  home_.orientation.z = transform.getRotation().z();
  home_.orientation.w = transform.getRotation().w();
  target_ = home_;

  gripper_ = nh_.serviceClient<osrf_gear::VacuumGripperControl>(
      "/ariac/gripper/control");

  transform = this->getTransfrom("/world", "/agv1_load_point_frame");

  agv_.position.x = transform.getOrigin().x();
  agv_.position.y = transform.getOrigin().y();
  agv_.position.z = transform.getOrigin().z() + 2 * z_offSet_pickUp_;
  agv_.orientation = home_.orientation;
}

UR10_Control::~UR10_Control() {}

tf::StampedTransform UR10_Control::getTransfrom(const std::string& src,
                                                const std::string& target) {
  tf::StampedTransform transform;
  tf::TransformListener listener;
  listener.waitForTransform(src, target, ros::Time(0), ros::Duration(20));
  try {
    listener.lookupTransform(src, target, ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
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
    ros::Duration(1.0).sleep();
  }
}

void UR10_Control::goToStart() {
  std::vector<double> target = {0, 3, -1, 1.9, 4, 4.7, 0};
  ur10_.setJointValueTarget(target);
  this->move();
  ros::Duration(1.0).sleep();
}
void UR10_Control::place() {
  // ROS_INFO_STREAM(agv.position);

  std::vector<std::vector<double>> poses = {
      {1.76, 0.38, -1.38, 2.76, 3.27, -1.51, 0.00}};  //,
  // {2.76, 0.38, -1.38, 1.5, 3.27, -1.51, 0.00},
  // {2.76, 1.56, -1.38, 1.5, 3.27, -1.51, 0.00},
  // {2.76, 1.56, -0.63, 1.5, 3.27, -1.51, 0.00}};

  for (const auto& itr : poses) {
    ur10_.setJointValueTarget(itr);
    this->move();
    ros::Duration(1.0).sleep();
  }

  this->setTarget(agv_);
  this->move();

  this->gripperAction(gripper::OPEN);

  ros::Duration(1.0).sleep();

  for (const auto& itr : poses) {
    ur10_.setJointValueTarget(itr);
    this->move();
    ros::Duration(1.0).sleep();
  }
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

void UR10_Control::pickAndPlace(const geometry_msgs::Pose& target_) {
  // ur10.goToStart();
  ROS_INFO("Setting Target..");
  this->setTarget(target_);
  ROS_INFO("Moving");
  this->move();
  ROS_INFO("Picking Up");

  this->gripperAction(gripper::CLOSE);
  ROS_INFO("Placing");

  this->goToStart();

  this->place();

  ROS_INFO("Home");
  this->goToStart();
}
