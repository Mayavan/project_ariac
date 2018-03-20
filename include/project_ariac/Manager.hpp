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
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <list>
#include <map>
#include <memory>
#include <string>

typedef osrf_gear::LogicalCameraImage::ConstPtr CameraMsg;
typedef Sensor<CameraMsg> Camera;
typedef std::shared_ptr<Camera> CameraPtr;

typedef osrf_gear::Order::ConstPtr OrderMsg;
typedef Sensor<OrderMsg> Order;
typedef std::shared_ptr<Order> OrderPtr;

typedef std::shared_ptr<ros::NodeHandle> NodePtr;
typedef std::shared_ptr<ros::Rate> RatePtr;
typedef std::map<std::string, std::list<std::string>> Database;

class Manager {
  explicit Manager(const ros::NodeHandle& nh);
  ~Manager();
  void checkInventory();
  void finishOrder();
  std::string getPart(const std::string& partType);

 private:
  NodePtr nh_;
  CameraPtr logical_camera_1_, logical_camera_2_;
  OrderPtr order_manager_;
  RatePtr rate;
  Database inventory_
};
