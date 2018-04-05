/**
 * @file Manager.hpp
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
#pragma once
// ROS interface
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
// Ariac interface
#include <osrf_gear/AGVControl.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <std_srvs/Trigger.h>
// STL
#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>
// Custom abstraction
#include "project_ariac/Sensor.hpp"
// Custom types
typedef osrf_gear::LogicalCameraImage::ConstPtr CameraMsg;
typedef Sensor<CameraMsg> Camera;
typedef std::shared_ptr<Camera> CameraPtr;

typedef osrf_gear::Order::ConstPtr OrderMsg;
typedef Sensor<OrderMsg> Order;
typedef std::shared_ptr<Order> OrderPtr;

typedef std::shared_ptr<ros::NodeHandle> NodePtr;
typedef std::shared_ptr<ros::Rate> RatePtr;
typedef std::map<std::string, std::list<std::string>> Database;

/**
 * @brief      Class for manager.
 */
class Manager {
 public:
  explicit Manager(const ros::NodeHandle& nh);
  ~Manager();
  void checkInventory();
  // void finishOrder();
  std::string getPart(const std::string& partType);
  // ARIAC interface
  void start_competition(std::string topic = "/ariac/start_competition") const;
  void end_competition(std::string topic = "/ariac/end_competition") const;
  OrderMsg getTheOrderMsg();
  void send_order(std::string agv = "/ariac/agv1",
                  std::string kit_id = "order_0_kit_0") const;
  std::vector<geometry_msgs::Pose> look_over_tray(const std::string& part_type);

  geometry_msgs::Pose findPose(const geometry_msgs::Pose& inPose,
                               const std::string& header);

  float distance(const geometry_msgs::Pose& current,
                 const geometry_msgs::Pose& target);

 private:
  NodePtr nh_;
  CameraPtr logical_camera_1_, logical_camera_2_, logical_camera_3_;
  OrderPtr order_manager_;
  RatePtr rate_;
  Database inventory_;
  tf::TransformListener listener_;
};
