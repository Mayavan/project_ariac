/**
 * @file UR10_Control.hpp
 * @author     Ravi Bhadeshiya
 * @version    0.1
 * @brief      Class for controlling ur10 arm
 *
 * @copyright  MIT License (c) 2017 Ravi Bhadeshiya
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#pragma once

#include <geometry_msgs/Pose.h>
// #include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>
#include <tf/transform_listener.h>
#include <vector>

class UR10_Control {
 public:
  UR10_Control();
  ~UR10_Control();
  void set_target(const geometry_msgs::Pose& target_);;
  void gripperAction(const bool action);
  bool plan();
  void move();
  void goToStart();
  void place();

  tf::StampedTransform getTransfrom(const std::string& src, const std::string& target);
  void pickAndPlace(const geometry_msgs::Pose& target_);
 private:
  moveit::planning_interface::MoveGroupInterface ur10_;
  moveit::planning_interface::MoveGroupInterface::Plan planner;
  geometry_msgs::Pose target, home, agv;
  ros::NodeHandle nh;
  ros::ServiceClient gripper_;
  bool success;
  double z_offSet_pickUp = 0.026;
};

namespace gripper {
const bool OPEN = false;
const bool CLOSE = true;
}  // namespace gripper

