/**
 * @file UR10_Control.cpp
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
#include "project_ariac/UR10_Control.hpp"

UR10_Control::UR10_Control() : ur10_("manipulator") {
  // ur10_.setPlanningTime(5);
  ur10_.setPlanningTime(10);
  ur10_.setNumPlanningAttempts(10);
  ur10_.setPlannerId("RRTConnectkConfigDefault");

  ros::Duration(1.0).sleep();
  this->goToStart();

  tf::StampedTransform transform;
  tf::TransformListener listener;
  listener.waitForTransform("world", "ee_link", ros::Time(0), ros::Duration(2));
  try {
    listener.lookupTransform("/world", "ee_link", ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  home.orientation.x = transform.getRotation().x();
  home.orientation.y = transform.getRotation().y();
  home.orientation.z = transform.getRotation().z();
  home.orientation.w = transform.getRotation().w();
  target = home;

  gripper_ = nh.serviceClient<osrf_gear::VacuumGripperControl>(
      "/ariac/gripper/control");
}

UR10_Control::~UR10_Control() {}

void UR10_Control::set_target(const geometry_msgs::Pose& target_) {
  target.position = target_.position;
  target.position.z += z_offSet_pickUp;
  ur10_.setPoseTarget(target);
}

bool UR10_Control::plan() {
  // bool success = ur10_.plan(planner);
  ROS_INFO("Planning start!");
  bool success = (ur10_.plan(planner) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
    ROS_INFO("Planned success.");
  else
    ROS_INFO("Planned unsucess!");
  return success;
}

void UR10_Control::move() {
  ros::AsyncSpinner spinner(1);
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
  std::vector<std::vector<double>> poses = {
      {2.76, 0.28, -1.38, 2.76, 3.27, -1.51, 0.00},
      {2.76, 0.38, -1.38, 1.5, 3.27, -1.51, 0.00},
      {2.76, 1.56, -1.38, 1.5, 3.27, -1.51, 0.00},
      {2.76, 1.56, -0.63, 1.5, 3.27, -1.51, 0.00}};

  for (const auto& itr : poses) {
    ur10_.setJointValueTarget(itr);
    this->move();
    ros::Duration(0.1).sleep();
  }

  ROS_INFO("Droping");
  this->gripperAction(gripper::OPEN);

  ur10_.setJointValueTarget({0, 1.56, -1.38, 1.5, 3.27, -1.51, 0.00});
  this->move();
  ros::Duration(1).sleep();
}
void UR10_Control::gripperAction(const bool action) {
  osrf_gear::VacuumGripperControl srv;

  srv.request.enable = action ? gripper::CLOSE : gripper::OPEN;

  gripper_.call(srv);

  if (srv.response.success)
    ROS_INFO_STREAM("Gripper Action Successfully Exicuted!");
  else
    ROS_ERROR("Gripper Action Failed!");

  ros::Duration(1).sleep();
}
