/**
 * @file Manager.cpp
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

#include "project_ariac/Manager.hpp"

Manager::Manager(ros::NodeHandle& nh) : nh_(nh) {
  logical_camera_1_ = nh_.subscribe("/ariac/logical_camera_1", 10,
                                    &Manager::logical_camera_callback_1, this);

  logical_camera_2_ = nh_.subscribe("/ariac/logical_camera_2", 10,
                                    &Manager::logical_camera_callback_2, this);

  orders_subscriber_ =
      nh_.subscribe("/ariac/orders", 10, &Manager::order_callback, this);

  // ROS_INFO_STREAM("Init");
}

Manager::~Manager() {}

void Manager::print(const database& parts) {
  ROS_INFO_STREAM("size:" << parts.size());
  for (const auto& i : parts) {
    ROS_INFO_STREAM(i.first);
    for (const auto& j : i.second) {
      ROS_INFO_STREAM(j);
    }
  }
}

void Manager::logical_camera_callback_1(
    const osrf_gear::LogicalCameraImage::ConstPtr& image_msg) {
  // ROS_INFO_STREAM("logical_camera_1 in");
  if (l1_flag_) return;
  size_t count = 1;
  for (const auto& itr : image_msg->models) {
    std::string partFrame =
        "logical_camera_1_" + itr.type + "_" + std::to_string(count) + "_frame";
    inventory_[itr.type].push_back(partFrame);
    count++;
  }
  ROS_INFO_STREAM("logical_camera_1 complete");
  // print(inventory_);
  l1_flag_ = true;
}

void Manager::logical_camera_callback_2(
    const osrf_gear::LogicalCameraImage::ConstPtr& image_msg) {
  // ROS_INFO_STREAM("logical_camera_2 in");
  if (l2_flag_) return;
  size_t count = 1;
  for (const auto& itr : image_msg->models) {
    std::string partFrame =
        "logical_camera_2_" + itr.type + "_" + std::to_string(count) + "_frame";
    inventory_[itr.type].push_back(partFrame);
    count++;
  }
  ROS_INFO_STREAM("logical_camera_2 complete");
  // print(inventory_);
  l2_flag_ = true;
}

void Manager::order_callback(const osrf_gear::Order::ConstPtr& order_msg) {
  ROS_INFO_STREAM("order callback in");
  for (const auto& kit : order_msg->kits) {
    for (const auto& itr : kit.objects) {
      // order_[itr.type].push_back(inventory_[itr.type].front());
      // inventory_[itr.type].pop_front();
      order_[itr.type].push_back(getPart(itr.type));
    }
  }
  ROS_INFO_STREAM("order callback complete");
  // print(order_);
  order_complete_ = true;
  // }
}

std::string Manager::getPart(const std::string& partType) {
  std::string part = inventory_[partType].front();
  inventory_[partType].pop_front();
  return part;
}

bool Manager::isReady() { return l1_flag_ && l2_flag_; }

database Manager::getOrder() { return order_; }

bool Manager::isOrderReady() { return order_complete_; }
