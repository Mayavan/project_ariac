/**
 * @file Manager.hpp
 * @author     Ravi Bhadeshiya
 * @version    0.1
 * @brief      Class for controlling ariac order
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

#include <geometry_msgs/PoseArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <list>
#include <map>
#include <string>

using database = std::map<std::string, std::list<std::string>>;

class Manager {
 public:
  Manager(ros::NodeHandle& nh);
  ~Manager();
  bool isReady();
  void print(const database& parts);
  database getOrder();
  bool isOrderReady();
  std::string getPart(const std::string& partType);
  osrf_gear::Order::ConstPtr order_msg_;
 protected:
  void logical_camera_callback_1(
      const osrf_gear::LogicalCameraImage::ConstPtr &image_msg);
  void logical_camera_callback_2(
      const osrf_gear::LogicalCameraImage::ConstPtr &image_msg);
  void order_callback(const osrf_gear::Order::ConstPtr &order_msg);

 private:
  ros::Subscriber logical_camera_1_, logical_camera_2_, orders_subscriber_;
  database inventory_, order_;
  ros::NodeHandle nh_;
  bool l1_flag_ = false, l2_flag_ = false, order_complete_ = false;
};
