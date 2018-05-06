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

#include <control_msgs/JointTrajectoryControllerState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <memory>
#include <string>
#include <vector>

#include "project_ariac/Interface.hpp"
#include "project_ariac/Sensor.hpp"

namespace UR10 {
typedef osrf_gear::VacuumGripperState GripperState;
typedef std::shared_ptr<planning_scene::PlanningScene> PlanningScenePtr;
typedef osrf_gear::LogicalCameraImage::ConstPtr CameraMsg;
typedef Sensor<CameraMsg> Camera;
typedef std::shared_ptr<Camera> CameraPtr;
typedef sensor_msgs::JointState JointState;
typedef control_msgs::JointTrajectoryControllerState::ConstPtr ArmStatePtr;
enum Gripper_State { OPEN = 0, CLOSE = 1 };
} // namespace UR10

class UR10_Control : public Interface {
public:
  explicit UR10_Control(const ros::NodeHandle &server);
  ~UR10_Control();
  void gripperAction(UR10::Gripper_State action);
  void publishJointsValue(const std::vector<double> &joints,
                          std::size_t time = 1);
  bool move(const geometry_msgs::Pose &target);
  bool move(const std::vector<double> &target_joint);
  bool move(const std::vector<geometry_msgs::Pose> &waypoints,
            double velocity_factor = 1.0, double eef_step = 0.01,
            double jump_threshold = 0.0);

  void jointPosePublisher(const std::vector<double> &target_joint);

  bool pickup(const geometry_msgs::Pose &target);
  bool robust_pickup(const geometry_msgs::PoseStamped &pose,
                     std::string partType, int max_try = 2);
  bool conveyor_pickup(const geometry_msgs::Pose &target, double speed = -0.2);

  bool place(const std::vector<geometry_msgs::Pose> &targets);
  bool place(geometry_msgs::Pose target, int agv = 0);
  bool robust_place(const geometry_msgs::Pose &target, const std::string &ref,
                    int agv = 0, int max_try = 5);

  std::vector<double> getHomeJoint();
  geometry_msgs::Pose getHomePose();
  geometry_msgs::Pose getAgvPosition(const int &agv);
  bool checkQuality();
  void flip_pulley();
  std::vector<double> home_joint_angle_, conveyer_joint_, agv_waypoint_[2];

protected:
  void gripperStatusCallback(const UR10::GripperState &gripper_status);
  void jointStateCallback(const sensor_msgs::JointState &msg);
  void armStateCB(const UR10::ArmStatePtr &state);
  bool move();

private:
  ros::NodeHandle nh_;

  ros::Subscriber gripper_sensor_, arm_state_subscriber_, joint_state_sub;
  ros::Publisher joint_trajectory_publisher_;
  ros::ServiceClient gripper_;
  ros::Publisher joint_trajectory_pub;
  UR10::GripperState gripper_state_;
  UR10::CameraPtr quality_sensor_1_, quality_sensor_2_;
  UR10::JointState joint_state;

  moveit::planning_interface::MoveGroupInterface ur10_;
  moveit::planning_interface::MoveGroupInterface::Plan planner_;

  control_msgs::JointTrajectoryControllerState arm_state_;
  trajectory_msgs::JointTrajectory joint_traj_msg_;

  geometry_msgs::Pose target_, home_, agv_[2];

  double z_offSet_;
  bool pickup_monitor_, place_monitor_;
};
