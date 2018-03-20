/**
 * @file Manager.cpp
 * @author     Ravi Bhadeshiya
 * @version    2.0
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

Manager::Manager(const ros::NodeHandle& nh) {
  nh_ = std::make_shared<ros::NodeHandle>(nh);
  rate = std::make_shared<ros::Rate>(0.5);
  // Init Camera to see
  logical_camera_1_ = std::make_shared<Camera>(nh, "/ariac/logical_camera_1");
  logical_camera_2_ = std::make_shared<Camera>(nh, "/ariac/logical_camera_2");
  // Init order manager
  order_manager_ = std::make_shared<Order>(nh, "/ariac/orders");
  ROS_DEBUG_STREAM("Manager is init..");
}

Manager::~Manager() {}

void Manager::checkInventory() {
  // Wait until camera see
  while (!logical_camera_1_->isPopulated() &&
         !logical_camera_2_->isPopulated()) {
    ROS_INFO_STREAM("Scanning the item");
    ros::spinOnce();
    rate->sleep();
  }

  // reset inventory for update
  inventory_.clear();

  // add parts from Camera 1
  size_t count = 1;
  auto image_msg = logical_camera_1_->getMessage();
  for (const auto& part : image_msg->models) {
    std::string partFrame =
        "logical_camera_1_" + part.type + "_" + std::to_string(count) + "_frame";
    inventory_[part.type].push_back(partFrame);
    count++;
  }

  // add parts from Camera 2
  size_t count = 1;
  auto image_msg = logical_camera_2_->getMessage();
  for (const auto& part : image_msg->models) {
    std::string partFrame =
        "logical_camera_2_" + part.type + "_" + std::to_string(count) + "_frame";
    inventory_[part.type].push_back(partFrame);
    count++;
  }
}

void Manager::finishOrder() {
  // wait for order
  while (!order_manager_->isPopulated()) {
    ROS_INFO_STREAM("Waiting for order");
    ros::spinOnce();
    rate->sleep();
  }
  auto order_msg = order_manager_->getMessage();
  for (const auto& kit : order_msg->kits) {
    for (const auto& part : kit.objects) {
// TODO(ravib)
      sucess = false;
      do {
        // sucess = ur10 pick and place action using getPart(part.type);
      } while (!sucess);
    }
  }
}

std::string Manager::getPart(const std::string& partType) {
  std::string part = inventory_[partType].front();
  inventory_[partType].pop_front();
  return part;
}
