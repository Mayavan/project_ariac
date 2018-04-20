/**
 * @file Interface.hpp
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
#pragma once
// ROS interface
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <memory>
#include <ros/ros.h>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class Interface {
public:
  Interface();
  ~Interface();
  geometry_msgs::Pose getTransfrom(const std::string &src,
                                   const std::string &target);
  geometry_msgs::PoseStamped getPose(const geometry_msgs::PoseStamped &inPose,
                                     std::string ref = "world");
  geometry_msgs::Pose getPose(const geometry_msgs::Pose &inPose,
                              const std::string &frame,
                              std::string ref = "world");
  bool is_same(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2);

protected:
  tf2_ros::Buffer buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_ptr_;
  geometry_msgs::TransformStamped transformStamped_;
};
