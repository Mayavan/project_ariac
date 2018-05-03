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
  // ur10_.setGoalTolerance(0.001);
  ur10_.setEndEffector("vacuum_gripper_link");
  move(home_joint_angle_); // Home condition

  agv_waypoint_[0] = home_joint_angle_;
  agv_waypoint_[0][1] = 1.57;

  agv_waypoint_[1] = home_joint_angle_;
  agv_waypoint_[1][1] = 4.71;

  ros::Duration(0.5).sleep();
  // Find pose of home position
  home_ = this->getTransfrom(base_link, end_link);
  target_ = home_;

  agv_[0] = this->getTransfrom("world", "logical_camera_1_kit_tray_1_frame");
  agv_[0].position.z += 0.5;
  agv_[0].position.y -= 0.5;
  agv_[0].orientation = home_.orientation;

  agv_[1] = this->getTransfrom("world", "logical_camera_2_kit_tray_2_frame");
  agv_[1].position.z += 0.5;
  agv_[1].position.y += 0.5;
  agv_[1].orientation = home_.orientation;

  // Init the gripper control and feedback
  gripper_ = nh_.serviceClient<osrf_gear::VacuumGripperControl>(
      "/ariac/gripper/control");
  gripper_sensor_ = nh_.subscribe("/ariac/gripper/state/", 10,
                                  &UR10_Control::gripperStatusCallback, this);

  // Init the arm control and feedback
  joint_trajectory_pub =
      nh_.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm/command", 10);
  joint_state_sub = nh_.subscribe(
      "/ariac/joint_states", 10, &UR10_Control::jointStateCallback, this);

  quality_sensor_1_ =
      std::make_shared<UR10::Camera>(server, "/ariac/quality_control_sensor_1");
  quality_sensor_2_ =
      std::make_shared<UR10::Camera>(server, "/ariac/quality_control_sensor_2");
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

void UR10_Control::gripperAction(UR10::Gripper_State action) {
  osrf_gear::VacuumGripperControl srv;

  srv.request.enable = action;

  gripper_.call(srv);

  ros::Duration(0.5).sleep();

  if (srv.response.success)
    ROS_INFO_STREAM("Gripper Action Successfully Exicuted!");
  else
    ROS_ERROR("Gripper Action Failed!");
}

void UR10_Control::gripperStatusCallback(
    const UR10::GripperState &gripper_status) {
  gripper_state_ = gripper_status;
  if ((pickup_monitor_ && gripper_state_.attached) ||
      (place_monitor_ && !gripper_state_.attached))
    ur10_.stop();
}

bool UR10_Control::pickup(const geometry_msgs::Pose &target) {
  // Lock the orientation

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.reserve(2);

  target_.position = target.position;
  target_.position.z += 0.4;
  waypoints.push_back(target_);

  target_.position.z = target.position.z + z_offSet_;
  waypoints.push_back(target_);

  gripperAction(UR10::Gripper_State::CLOSE);
  // should stop after part is being picked
  pickup_monitor_ = true;

  ros::AsyncSpinner spinner(1);
  spinner.start();
  if (!move(waypoints))
    return false;
  // move({target_}, 0.1, 0.001);  // Grasp move
  ros::Duration(1.0).sleep();
  pickup_monitor_ = false;

  target_.position.z = home_.position.z;
  if (!move({target_})) {
    move(home_joint_angle_); // reset position
    return false;
  }
  // should attach after execution
  // it means, robot pick up part
  ros::spinOnce();
  ros::Duration(2.0).sleep();
  return gripper_state_.attached;
  // return res.result;i
}

bool UR10_Control::robust_pickup(const geometry_msgs::PoseStamped &pose,
                                 std::string partType, int max_try) {
  bool result;
  // hover below camera
  // auto t = getTransfrom("world", "bin_6_frame");
  // t.orientation = home_.orientation;
  // t.position.z = 0.2;
  // move({t});
  if (partType == "gear_part" || partType == "piston_rod_part") {
    z_offSet_ = 0.02;
  } else if (partType == "disk_part") {
    z_offSet_ = 0.025;
  } else if (partType == "pulley_part") {
    z_offSet_ = 0.085;
  } else {
    z_offSet_ = 0.038;
  }

  auto target_pick = getPose(pose);
  target_pick.pose.position.z += 0.002;
  do {
    // PoseStamped had header and getPose will give pose with world
    ROS_INFO_STREAM("Pick up try:" << max_try);
    target_pick.pose.position.z -= 0.002;
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
  waypoints.reserve(2);
  // target_.position.z += 0.1;
  // waypoints.push_back(target_);

  target_.position = agv_[agv].position;
  target_.position.y = target.position.y;
  waypoints.push_back(target_);

  target.position.z += 2 * z_offSet_;

  double roll, pitch, yaw, dummy;
  tf::Matrix3x3(tf::Quaternion(target.orientation.x, target.orientation.y,
                               target.orientation.z, target.orientation.w))
      .getRPY(roll, pitch, yaw);
  tf::Matrix3x3(tf::Quaternion(home_.orientation.x, home_.orientation.y,
                               home_.orientation.z, home_.orientation.w))
      .getRPY(roll, pitch, dummy);
  target.orientation =
      tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

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
  if (!move(targets, 1.0, 0.005))
    return false;
  place_monitor_ = false;
  // should attach before openning
  // robot didn't drop part
  ros::spinOnce();
  bool result = gripper_state_.attached;

  if (checkQuality()) {
    ROS_WARN("Faulty part detected!!");
    target_.position.y = 2.5;
    move({target_});
    result = false;
  }
  gripperAction(UR10::Gripper_State::OPEN);

  if (result) {
    move({target_, home_});
  } else {
    move({home_});
  }

  return result;
}

bool UR10_Control::robust_place(const geometry_msgs::Pose &target,
                                const std::string &ref, int agv, int max_try) {
  bool result;
  auto target_place = getPose(target, ref);
  agv_waypoint_[agv][0] = target_.position.y;
  do {
    if (max_try < 2) {
      move(agv_waypoint_[agv]);
    }
    result = place(target_place, agv);
    max_try--;
  } while (!result && max_try > 0 && gripper_state_.attached);

  if (!result) {
    ROS_WARN_STREAM("Robust Place failed..!");
    //   robust_pickup("find the part");
    //   place(target_place, agv);
  } else {
    // move(home_joint_angle_);
  }

  return result;
}

std::vector<double> UR10_Control::getHomeJoint() { return home_joint_angle_; }

geometry_msgs::Pose UR10_Control::getHomePose() { return home_; };

geometry_msgs::Pose UR10_Control::getAgvPosition(const int &agv) {
  return agv_[agv];
}

bool UR10_Control::checkQuality() {
  auto message1 = quality_sensor_1_->getMessage(); 
  auto message2 = quality_sensor_2_->getMessage();
  if(message1->models.size() > 0 || message2->models.size() > 0)
    return true;
  else
    return false;
}

void UR10_Control::jointPosePublisher(const std::vector<double>& target_joint) {

  trajectory_msgs::JointTrajectory target_msg;
  target_msg.header.stamp = ros::Time::now();
  target_msg.joint_names = joint_state.name;
  target_msg.joint_names.pop_back();
  target_msg.points.resize(1);
  target_msg.points[0].positions.resize(target_msg.joint_names.size());

  for (int i = 0; i < target_msg.joint_names.size(); ++i) {
    target_msg.points[0].positions[i] = target_joint[i];
  }

  target_msg.points[0].time_from_start = ros::Duration(1.0);

  ROS_INFO_STREAM("Populated jointPosePublisher msg:\n" << target_msg);

  joint_trajectory_pub.publish(target_msg);
  ros::Duration(1.4).sleep();
}

void UR10_Control::jointStateCallback(const sensor_msgs::JointState& msg) {
  joint_state = msg;
}

void UR10_Control::flip_pulley() {
  std::vector<double> wp1 = {2.243808742159853, 0.16169273710449183, -1.7694873064883794, 1.598143032604276, 5.038274633936634, 4.73679572242765, 0.03684661763998598};
  std::vector<double> wp2 = {2.3289613974599206, 0.2426605618523817, -1.5408388030807245, 1.5960846338517403, 5.449625032208066, 4.752812594024332, 0.010975675089456516};
  // Place from first side
  std::vector<double> wp3 = {2.360798294157197, 0.1859866186964006, -1.2283683580430704, 1.2047579602040495, 5.103293282235604, 4.362086570708567, -0.004970785204934103};
  // Step back
  std::vector<double> wp4 = {2.34880743534395, 0.22548707985445882, -1.2257864526747948, 1.2079535887800814, 5.119238786846012, 4.365146771837186, -0.005380604795716337};
  
  //arm turnout
  std::vector<double> wp5 = {2.287224870686398, 0.19767039827869637, -1.1912042032635934, 0.9013114658623609, 5.149520202311113, 4.441786081024084, -0.016770811960842735};

  //arm side
  std::vector<double> wp6 = {2.2007938731566874, 0.15170497864935303, -1.1437623223726359, 0.7628351932910915, 5.187959682960177, 4.363291305553873, -0.02214615241800555};

  //slide
  std::vector<double> wp7 = {2.2026058386721594, -0.06668689898951524, -1.1287201901052004, 0.6902596139987214, 5.169831897643232, 4.290738006613766, -0.025356202879857292};

  //move in
  std::vector<double> wp8 = {2.6579125554873855, -0.47715362642037573, -1.2916730129504108, 0.9353993159140144, 4.882354647966725, 4.535688334508005, -0.016017252904395818};

  //rotate gripper
  std::vector<double> wp9 = {2.175422344537143, -0.9009763234963939, -1.0619403346946434, 1.2862107544049977, 5.13440554453595, 1.288540182420343, 0.0036720685817206444};

  //close in
  std::vector<double> wp10 = {1.9989557267823095, -0.32759369047971226, -1.0162950170243512, 1.321540017454888, 5.2655885748216145, 1.3238577029887244, 0.0024484093827004116};

  //touch
  std::vector<double> wp11 = {1.9914384552160271, -0.303274804021313, -1.0140756352714888, 1.322913316324362, 5.2709040697580845, 1.325063365888818, 0.0022407007024041192};


  ROS_INFO_STREAM("Before move no plan..");
  jointPosePublisher(wp1);
  jointPosePublisher(wp2);
  jointPosePublisher(wp3);

  gripperAction(UR10::Gripper_State::OPEN);
  ros::Duration(2.0).sleep();

  jointPosePublisher(wp4);
  jointPosePublisher(wp5);
  jointPosePublisher(wp6);
  jointPosePublisher(wp7);
  jointPosePublisher(wp8);
  jointPosePublisher(wp9);
  jointPosePublisher(wp10);
  jointPosePublisher(wp11);
  
  gripperAction(UR10::Gripper_State::CLOSE);

  ros::spinOnce();
  ros::Duration(2.0).sleep();
  ros::spinOnce();
  int max_try = 5;
  int try_count = 0;
  while ((!gripper_state_.attached) && (max_try > try_count) )
  {
    wp11[1] += 0.01;
    jointPosePublisher(wp11);
    try_count ++;
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    ros::spinOnce();
  }

}

