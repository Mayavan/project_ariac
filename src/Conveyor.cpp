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

Conveyor::Conveyor(const ros::NodeHandle &nh) {
  nh_ = std::make_shared<ros::NodeHandle>(nh);
  sensor_subscriber_ =
      nh_->subscribe("/ariac/logical_camera_1", 10, &Conveyor::callback, this);
  pub_part = nh_->advertise<osrf_gear::LogicalCameraImage>(
      "project_ariac/conveyer", 10);
  service_ =
      nh_->advertiseService("conveyer/getSpeed", &Conveyor::findSpeed, this);
  tfBroadcastTimer = nh.createTimer(ros::Duration(0.05),
                                    &Conveyor::broadcast_tf_callback, this);
  last_time_ = ros::Time::now();
}

Conveyor::~Conveyor() {}

void Conveyor::callback(const conveyor::CameraMsg &msg) {
  current_time_ = ros::Time::now();
  dt_ = current_time_.toSec() - last_time_.toSec();
  osrf_gear::LogicalCameraImage parts_on_conv;

  if (SPEED_ > 0.0 && msg->models.size() > 0) {

    distances_.push_back(msg->models.front().pose.position.y);

    if (distances_.size() == MAX_Element) {
      SPEED_ = findAvgSpeed(distances_, dt_);
      ROS_WARN_STREAM(SPEED_ << " speed detected");
    }
  } else if (SPEED_ < 0) {

    this->update(parts_on_conv);

    for (const auto &part_in_view : msg->models) {
      part_ = getPose(part_in_view.pose, "logical_camera_1_frame");
      itr = inventory_.find(part_in_view.type);
      bool found = false;
      if (itr != inventory_.end()) {
        for (const auto &part : itr->second) {
          if (is_same(part_, part)) {
            found = true;
            break;
          }
        }
      }
      if (!found) {
        inventory_[part_in_view.type].emplace_back(part_);
        model_.type = part_in_view.type;
        model_.pose = part_;
        parts_on_conv.models.emplace_back(model_);
      }
    }
  }
  pub_part.publish(parts_on_conv);

  counter++;
  last_time_ = current_time_;
}

void Conveyor::update(osrf_gear::LogicalCameraImage &msg) {
  for (auto &part_list : inventory_) {
    ROS_INFO_STREAM(part_list.first << ":" << part_list.second.size());
    for (auto part = part_list.second.begin(); part != part_list.second.end();
         ++part) {
      // update the part postion in inventory
      part->position.y += SPEED_ * dt_;
      // conveyor only move in one direction(-ve y axis)
      if (part->position.y < -3.0) {
        // remove part if it's unrechable
        part_list.second.erase(part);
      } else {
        // else publish part
        model_.type = part_list.first;
        model_.pose = *part;
        msg.models.emplace_back(model_);
      }
    }
  }
}

double Conveyor::findAvgSpeed(const std::vector<double> &dist,
                              const double &dt) {
  size_t size = dist.size();
  double *speeds = new double[size];

  std::adjacent_difference(
      dist.begin(), dist.end(), speeds,
      [dt](const double &l, const double &r) { return (l - r) / dt; });

  double sum = 0;
  for (size_t i = 1; i < size; i++) {
    sum += speeds[i];
  }

  delete[] speeds;
  return sum / double(size - 1);
}

bool Conveyor::findSpeed(project_ariac::getSpeed::Request &req,
                         project_ariac::getSpeed::Response &res) {
  res.speed = getSpeed();
  return true;
}

double Conveyor::getSpeed() { return SPEED_ < 0 ? SPEED_ : 0; }

void Conveyor::broadcast_tf_callback(const ros::TimerEvent &event) {
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;
  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "/world";
  static_transformStamped.child_frame_id = "/logical_camera_1_kit_tray_1_frame";
  static_transformStamped.transform.translation.x = 0.3;
  static_transformStamped.transform.translation.y = 3.15;
  static_transformStamped.transform.translation.z = 0.75;
  static_transformStamped.transform.rotation.x = 0;
  static_transformStamped.transform.rotation.y = 0;
  static_transformStamped.transform.rotation.z = 1;
  static_transformStamped.transform.rotation.w = 0;
  static_broadcaster.sendTransform(static_transformStamped);
  static_transformStamped.child_frame_id = "/logical_camera_2_kit_tray_2_frame";
  static_transformStamped.transform.translation.x = 0.3;
  static_transformStamped.transform.translation.y = -3.15;
  static_transformStamped.transform.translation.z = 0.75;
  static_transformStamped.transform.rotation.x = 0;
  static_transformStamped.transform.rotation.y = 0;
  static_transformStamped.transform.rotation.z = 1;
  static_transformStamped.transform.rotation.w = 0;
  static_broadcaster.sendTransform(static_transformStamped);
}
