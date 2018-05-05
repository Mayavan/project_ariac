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
// Ariac interface
#include <osrf_gear/AGVControl.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/GetMaterialLocations.h>
#include <osrf_gear/StorageUnit.h>
#include <osrf_gear/Order.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
// STL
// #include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>
// Custom abstraction
#include "project_ariac/Interface.hpp"
#include "project_ariac/Sensor.hpp"
#include "project_ariac/getSpeed.h"

namespace manager {
// Custom types
typedef osrf_gear::LogicalCameraImage::ConstPtr CameraMsg;
typedef sensor_msgs::JointState::ConstPtr JointMsg;
typedef osrf_gear::Order::ConstPtr OrderMsg;

typedef Sensor<std_msgs::String::ConstPtr> Agv;
typedef Sensor<JointMsg> ArmState;
typedef Sensor<CameraMsg> Camera;
typedef Sensor<OrderMsg> Order;

typedef std::shared_ptr<ros::NodeHandle> NodePtr;
typedef std::shared_ptr<ArmState> ArmStatePtr;
typedef std::shared_ptr<ros::Rate> RatePtr;
typedef std::shared_ptr<Camera> CameraPtr;
typedef std::shared_ptr<Order> OrderPtr;
typedef std::shared_ptr<Agv> AgvPtr;
/**
 * @brief      map < part_type, list of poseStamped>
 */
typedef std::map<int, std::vector<geometry_msgs::PoseStamped>> Database;
} // namespace manager

/**
 * @brief      Class for manager.
 */
class Manager : public Interface {
public:
  explicit Manager(const ros::NodeHandle &nh);
  ~Manager();
  void Inventory();
  manager::Database processedOrder();
  geometry_msgs::PoseStamped getPart(const std::string &partType);
  // ARIAC interface
  void start_competition(std::string topic = "/ariac/start_competition") const;
  void end_competition(std::string topic = "/ariac/end_competition") const;
  void send_order(std::string agv = "/ariac/agv1",
                  std::string kit_id = "order_0_kit_0") const;
  std::string get_BinId(std::string part, std::string topic = "/ariac/material_locations") const;
  manager::OrderMsg getTheOrderMsg();
  bool isHighOrder();
  std::vector<geometry_msgs::PoseStamped>
  look_over_tray(const geometry_msgs::Pose &target, const std::string &partType,
                 const int &agv);

  std::vector<osrf_gear::KitObject> kit_check(const osrf_gear::Kit &kit,
                                              std::string camera);

  bool isAgvReady(const int &no);
  int pick_agv();
  double getConveyorSpeed();

private:
  void initialize();
  NodePtr nh_;
  manager::CameraPtr logical_camera_1_, logical_camera_2_, logical_camera_3_,
      logical_camera_4_, logical_camera_5_, logical_camera_6_, conveyor_;
  manager::AgvPtr agv_[2];
  manager::OrderPtr order_manager_;
  manager::RatePtr rate_;
  manager::ArmStatePtr arm_state_;
  manager::Database inventory_;
  std::vector<std::vector<double>> config1, config2, config3, config4;
  int configType;
};
