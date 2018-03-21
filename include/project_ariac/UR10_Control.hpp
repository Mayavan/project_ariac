/**
 * @file UR10_Control.hpp
 * @author     Ravi Bhadeshiya
 * @version    2.0
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
#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>

#include <memory>
#include <string>
#include <vector>

#include "project_ariac/Sensor.hpp"
#include "project_ariac/pickup.h"
#include "project_ariac/place.h"

typedef osrf_gear::VacuumGripperState::ConstPtr GripperState;

class UR10_Control {
 public:
  explicit UR10_Control(const ros::NodeHandle& server);
  ~UR10_Control();
  void gripperAction(const bool action);

  void move(const geometry_msgs::Pose& target);
  void move(const std::vector<double>& target_joint);
  void move(const std::vector<geometry_msgs::Pose>& waypoints,
            double velocity_factor = 1.0, double eef_step = 0.1,
            double jump_threshold = 0.0);
  tf::StampedTransform getTransfrom(const std::string& src,
                                    const std::string& target);

 protected:
  void gripperStatusCallback(const GripperState& gripper_status);
  bool pickupSrvCB(project_ariac::pickup::Request& req,
                   project_ariac::pickup::Response& res);
  bool placeSrvCB(project_ariac::place::Request& req,
                  project_ariac::place::Response& res);

 private:
  ros::NodeHandle nh_;
  geometry_msgs::Pose target_, home_, agv_;

  ros::Subscriber gripper_sensor_;
  ros::ServiceServer pickupServer_, placeServer_;
  ros::ServiceClient gripper_;

  GripperState gripper_state_;

  moveit::planning_interface::MoveGroupInterface ur10_;
  moveit::planning_interface::MoveGroupInterface::Plan planner_;

  double z_offSet_;
  std::vector<double> home_joint_angle_;
  bool gripperPickCheck, gripperPlaceCheck;
};

namespace gripper {
const bool OPEN = false;
const bool CLOSE = true;
}  // namespace gripper
