/**
 * @file Conveyor.cpp
 * @author     Ravi Bhadeshiya
 * @version    2.0
 * @brief      Class for Conveyor
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
#include "project_ariac/Conveyor.hpp"

Conveyor::Conveyor() {}

Conveyor::Conveyor(const ros::NodeHandle &nh)
 {
  nh_ = std::make_shared<ros::NodeHandle>(nh);
  sensor_subscriber_ =
      nh_->subscribe("/ariac/logical_camera_1", 10, &Conveyor::callback, this);
  pub_part = nh_->advertise<osrf_gear::LogicalCameraImage>("project_ariac/conveyer",10);
  last_time_ = ros::Time::now();
}

Conveyor::~Conveyor() {}

void Conveyor::callback(const conveyor::CameraMsg &msg) {
  current_time_ = ros::Time::now();
  double dt = current_time_.toSec() - last_time_.toSec();
  osrf_gear::LogicalCameraImage parts_on_conv ;
  osrf_gear::Model model;

  for (auto &part_list : inventory_) {
    ROS_INFO_STREAM(part_list.first << ":" << part_list.second.size());
    for (auto part = part_list.second.begin(); part != part_list.second.end();
         ++part) {
      // update the part postion in inventory
      // conveyor only move in one direction(-ve y axis)
      part->position.y -= SPEED_ * dt;
      model.type = part_list.first;
      model.pose = *part;
	
      if (part->position.y < -3.0) {
        part_list.second.erase(part);
       } else {
       parts_on_conv.models.emplace_back(model);
	   }
    }
  }

  for (const auto &part_in_view : msg->models) {
    auto P = getPose(part_in_view.pose, "logical_camera_1_frame");
    auto itr = inventory_.find(part_in_view.type);
    bool found = false;
    if (itr != inventory_.end()) {
      for (const auto &part : itr->second) {
        if (is_same(P, part)) {
          found = true;
          break;
        }
      }
    }
    if (!found) {
      inventory_[part_in_view.type].emplace_back(P);
      model.type = part_in_view.type;
      model.pose = P;
      parts_on_conv.models.emplace_back(model);
  }
  
  }
  //conveyor::CameraMsg partMsg = *msg; 
  pub_part.publish(parts_on_conv);
  counter += 1;
  last_time_ = current_time_;
}
