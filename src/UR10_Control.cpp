/**
 * @file UR10_Control.cpp
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
#include "project_ariac/UR10_Control.hpp"

UR10_Control::UR10_Control(const ros::NodeHandle &server)
    : ur10_("manipulator"), pickup_monitor_(false), place_monitor_(false) {
  // init the move group interface
  // Set the planning param
  int planning_attempt, planning_time;
  std::string planner, end_link, base_link, scene_file;
  home_joint_angle_.resize(7);

  server.param("elbow_joint", home_joint_angle_[0], 0.0);
  server.param("linear_arm_actuator_joint", home_joint_angle_[1], 3.0);
  server.param("shoulder_lift_joint", home_joint_angle_[2], -1.0);
  server.param("shoulder_pan_joint", home_joint_angle_[3], 1.9);
  server.param("wrist_1_joint", home_joint_angle_[4], 4.0);
  server.param("wrist_2_joint", home_joint_angle_[5], 4.7);
  server.param("wrist_3_joint", home_joint_angle_[6], 0.0);
  server.param("z_offSet_", z_offSet_, 0.030);
  server.param("planning_time", planning_time, 100);
  server.param("planning_attempt", planning_attempt, 20);
  server.param<std::string>("planner", planner, "RRTConnectkConfigDefault");
  server.param<std::string>("end_link", end_link, "ee_link");
  server.param<std::string>("base_link", base_link, "world");
  server.param<std::string>("scene", scene_file, " ");

  ur10_.setPlannerId(planner);
  ur10_.setPlanningTime(planning_time);
  ur10_.setNumPlanningAttempts(planning_attempt);
  ur10_.allowReplanning(true);
  ur10_.setMaxAccelerationScalingFactor(0.5);
  ur10_.setMaxVelocityScalingFactor(0.5);

  // ur10_.setEndEffector("vacuum_gripper_link");
  move(home_joint_angle_); // Home condition
  ros::Duration(0.5).sleep();
  // Find pose of home position
  home_ = this->getTransfrom(base_link, end_link);
  target_ = home_;

  agv_[0] = this->getTransfrom("world", "agv1_load_point_frame");
  agv_[0].position.z += 0.5;
  agv_[0].orientation = home_.orientation;

  agv_[1] = this->getTransfrom("world", "agv2_load_point_frame");
  agv_[1].position.z += 0.5;
  agv_[1].orientation = home_.orientation;

  // Init the gripper control and feedback
  gripper_ = nh_.serviceClient<osrf_gear::VacuumGripperControl>(
      "/ariac/gripper/control");
  gripper_sensor_ = nh_.subscribe("/ariac/gripper/state/", 10,
                                  &UR10_Control::gripperStatusCallback, this);
  ros::Duration(0.5).sleep();
}

UR10_Control::~UR10_Control() { ur10_.stop(); }

bool UR10_Control::move() {
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Planning start");

  bool success = (ur10_.plan(planner_) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success) {
    ROS_INFO("Planned success.");
    // ur10_.move();
    ur10_.execute(planner_);
  } else {
    ROS_WARN("Planned unsucess!");
  }

  return success;
}

bool UR10_Control::move(const geometry_msgs::Pose &target) {
  ur10_.setPoseTarget(target);
  return move();
}

bool UR10_Control::move(const std::vector<double> &target_joint) {
  ur10_.setJointValueTarget(target_joint);
  return move();
}

bool UR10_Control::move(const std::vector<geometry_msgs::Pose> &waypoints,
                        double velocity_factor, double eef_step,
                        double jump_threshold) {
  // ros::AsyncSpinner spinner(1);
  // spinner.start();
  moveit_msgs::RobotTrajectory trajectory;
  ur10_.setMaxVelocityScalingFactor(velocity_factor);

  double fraction = ur10_.computeCartesianPath(waypoints, eef_step,
                                               jump_threshold, trajectory);
  planner_.trajectory_ = trajectory;

  ROS_INFO("UR10 control Move %.2f%% acheived..", fraction * 100.0);

  if (fraction > 0.9) {
    ur10_.execute(planner_);
    return true;
  }
  return false;
}

void UR10_Control::gripperAction(const bool action) {
  osrf_gear::VacuumGripperControl srv;

  srv.request.enable = action ? gripper::CLOSE : gripper::OPEN;

  gripper_.call(srv);

  ros::Duration(0.5).sleep();

  if (srv.response.success)
    ROS_INFO_STREAM("Gripper Action Successfully Exicuted!");
  else
    ROS_ERROR("Gripper Action Failed!");
}

void UR10_Control::gripperStatusCallback(const GripperState &gripper_status) {
  gripper_state_ = gripper_status;
  if ((pickup_monitor_ && gripper_state_.attached) ||
      (place_monitor_ && !gripper_state_.attached))
    ur10_.stop();
}

bool UR10_Control::pickup(const geometry_msgs::Pose &target) {
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // Lock the orientation

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.reserve(2);

  target_.position = target.position;
  target_.position.z += 0.4;
  waypoints.push_back(target_);

  target_.position.z = target.position.z + z_offSet_;
  waypoints.push_back(target_);

  gripperAction(gripper::CLOSE);
  // should stop after part is being picked
  pickup_monitor_ = true;
  if (!move(waypoints, 1.0, 0.001))
    return false;
  // move({target_}, 0.1, 0.001);  // Grasp move
  ros::Duration(1.0).sleep();
  pickup_monitor_ = false;
  target_.position.z += 0.5;
  move({target_});
  // should attach after execution
  // it means, robot pick up part
  ros::spinOnce();
  ros::Duration(0.5).sleep();
  return gripper_state_.attached;
  // return res.result;i
}

bool UR10_Control::robust_pickup(const geometry_msgs::PoseStamped &pose,
                                 int max_try) {
  bool result;
  // hover below camera
  // auto t = getTransfrom("world", "bin_6_frame");
  // t.orientation = home_.orientation;
  // t.position.z = 0.2;
  // move({t});
  do {
    // PoseStamped had header and getPose will give pose with world
    ROS_INFO_STREAM("Pick up try:" << max_try);
    auto target_pick = getPose(pose);
    result = pickup(target_pick.pose);
    max_try--;
    // it will try until sucess or max try
  } while (!result && max_try > 0);
  return result;
}

bool UR10_Control::place(geometry_msgs::Pose target, int agv) {
  // Lock the orientation
  // initConstraint();
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.reserve(3);

  // target_.position.z += 0.5;
  // waypoints.push_back(target_);

  target_.position = agv_[agv].position;
  target_.position.y = target.position.y;
  waypoints.push_back(target_);

  target.position.z += 2 * z_offSet_;
  // TODO(harish) final orientance incorrect (Just yaw fix)
  // orientation convert to rpy and assign yaw and convert back to euler
  tf::Quaternion q, final;
  tf::Matrix3x3 m;
  double r_h, p_h, y_h, y_t;
  q = {target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w};
  m.setRotation(q);
  m.getRPY(r_h, p_h, y_t);
  q = {home_.orientation.x, home_.orientation.y, home_.orientation.z, home_.orientation.w};
  m.setRotation(q);
  m.getRPY(r_h, p_h, y_h);
  final = tf::createQuaternionFromRPY(r_h, p_h, y_t);
  target.orientation.x = final.getX();
  target.orientation.y = final.getY();
  target.orientation.z = final.getZ();
  target.orientation.w = final.getW();
  waypoints.push_back(target);

  // it will stop the motion,
  // if robot drop part
  return place(waypoints);
}

bool UR10_Control::place(const std::vector<geometry_msgs::Pose> &targets) {
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // it will stop the motion,
  // if robot drop part
  place_monitor_ = true;
  move(targets, 0.5, 0.001);
  place_monitor_ = false;
  // should attach before openning
  // robot didn't drop part

  ros::spinOnce();
  bool result = gripper_state_.attached;
  gripperAction(gripper::OPEN);
  if (result) {
    //   // ready for next part
    //   // target_.position.z += 0.1;
    move({target_, home_});
    //   // move(home_joint_angle_);
  }
  return result;
}

bool UR10_Control::robust_place(const geometry_msgs::Pose &target,
                                const std::string &ref, int agv) {
  bool result;

  auto target_place = getPose(target, ref);

  result = place(target_place, agv);

  if (!result) {
    ROS_WARN_STREAM("Robust Place failed..!");
    //   robust_pickup("find the part");
    //   place(target_place, agv);
  } else {
    // move(home_joint_angle_);
  }

  return result;
}

void UR10_Control::initConstraint() {
  moveit_msgs::Constraints ur10_constraints;
  moveit_msgs::JointConstraint jcm;
  auto joints = {"elbow_joint",         "linear_arm_actuator_joint",
                 "shoulder_lift_joint", "shoulder_pan_joint",
                 "wrist_1_joint",       "wrist_2_joint",
                 "wrist_3_joint"};

  size_t count = 0;
  for (const auto &joint : joints) {
    jcm.joint_name = joint;
    jcm.position = home_joint_angle_[count];
    jcm.tolerance_above = 1.0;
    jcm.tolerance_below = -3.0;
    jcm.weight = 1.0;
    ur10_constraints.joint_constraints.push_back(jcm);
    count++;
  }
  ur10_.setPathConstraints(ur10_constraints);
}
