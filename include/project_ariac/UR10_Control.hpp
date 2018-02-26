/**
 * @file UR10_Control.hpp
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
#pragma once

#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <string>
#include <vector>

class UR10_Control {
 public:
  explicit UR10_Control(const ros::NodeHandle& server);
  ~UR10_Control();
  void setTarget(const geometry_msgs::Pose& target);
  void gripperAction(const bool action);
  bool gripperPickup(const bool action);
  void gripperStatusCallback(
      const osrf_gear::VacuumGripperState::ConstPtr& gripper_status);
  void move();
  void goToStart();
  bool place();
  tf::StampedTransform getTransfrom(const std::string& src,
                                    const std::string& target);
  bool pickAndPlace(const geometry_msgs::Pose& target_);

 protected:
  bool plan();
  void gripperStatus(
      const osrf_gear::VacuumGripperState::ConstPtr& gripper_status);

 private:
  moveit::planning_interface::MoveGroupInterface ur10_;
  moveit::planning_interface::MoveGroupInterface::Plan planner_;
  geometry_msgs::Pose target_, home_, agv_;
  ros::NodeHandle nh_;
  ros::ServiceClient gripper_;
  double z_offSet_pickUp_;
  std::vector<double> starting_joint_angle_;
  ros::Subscriber gripper_callback_;
  bool gripper_attached_;
  osrf_gear::VacuumGripperState gripper_state_;
};

namespace gripper {
const bool OPEN = false;
const bool CLOSE = true;
}  // namespace gripper
