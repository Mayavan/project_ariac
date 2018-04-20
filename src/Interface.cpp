/**
 * @file Interface.cpp
 * @author     Ravi Bhadeshiya
 * @version    2.0
 * @brief      Class for Interface
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
#include "project_ariac/Interface.hpp"

Interface::Interface() {
  listener_ptr_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
}

Interface::~Interface() {}

geometry_msgs::Pose Interface::getTransfrom(const std::string &src,
                                            const std::string &target) {
  // auto time = ros::Time(0);
  // buffer_.waitForTransform(src, target, ros::Time(0), ros::Duration(10));
  try {
    transformStamped_ = buffer_.lookupTransform(src, target, ros::Time::now(),
                                                ros::Duration(10));
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(0.5).sleep();
  }
  geometry_msgs::Pose pose;
  // position
  pose.position.x = std::move(transformStamped_.transform.translation.x);
  pose.position.y = std::move(transformStamped_.transform.translation.y);
  pose.position.z = std::move(transformStamped_.transform.translation.z);
  // orientation
  pose.orientation.x = std::move(transformStamped_.transform.rotation.x);
  pose.orientation.y = std::move(transformStamped_.transform.rotation.y);
  pose.orientation.z = std::move(transformStamped_.transform.rotation.z);
  pose.orientation.w = std::move(transformStamped_.transform.rotation.w);

  return std::move(pose);
}

geometry_msgs::PoseStamped
Interface::getPose(const geometry_msgs::PoseStamped &inPose, std::string ref) {
  geometry_msgs::PoseStamped outPose_;
  try {
    transformStamped_ = buffer_.lookupTransform(
        ref, inPose.header.frame_id, ros::Time(0), ros::Duration(10));
    tf2::doTransform(inPose, outPose_, transformStamped_);
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(0.5).sleep();
  }

  return std::move(outPose_);
}

geometry_msgs::Pose Interface::getPose(const geometry_msgs::Pose &inPose,
                                       const std::string &frame,
                                       std::string ref) {
  geometry_msgs::PoseStamped inPose_;
  inPose_.header.frame_id = frame;
  inPose_.pose = inPose;

  geometry_msgs::PoseStamped outPose_ = getPose(inPose_, ref);
  return std::move(outPose_.pose);
}

bool Interface::is_same(const geometry_msgs::Pose &p1,
                        const geometry_msgs::Pose &p2) {
  double dist = std::pow(p1.position.x - p2.position.x, 2) +
                std::pow(p1.position.y - p2.position.y, 2) +
                std::pow(p1.position.z - p2.position.z, 2);
  double angle = std::pow(p1.orientation.x - p2.orientation.x, 2) +
                 std::pow(p1.orientation.y - p2.orientation.y, 2) +
                 std::pow(p1.orientation.z - p2.orientation.z, 2) +
                 std::pow(p1.orientation.w - p2.orientation.w, 2);
  return dist < std::pow(0.1, 2) && angle < std::pow(0.1, 2);
}
