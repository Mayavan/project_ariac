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

Manager::Manager(const ros::NodeHandle &nh) {
  nh_ = std::make_shared<ros::NodeHandle>(nh);
  rate_ = std::make_shared<ros::Rate>(0.5);
  // Init Cameras
  logical_camera_1_ = std::make_shared<Camera>(nh, "/ariac/logical_camera_1");
  logical_camera_2_ = std::make_shared<Camera>(nh, "/ariac/logical_camera_2");
  logical_camera_3_ = std::make_shared<Camera>(nh, "/ariac/logical_camera_3");
  logical_camera_4_ = std::make_shared<Camera>(nh, "/ariac/logical_camera_4");

  agv_[0] = std::make_shared<Agv>(nh, "/ariac/agv1/state");
  agv_[1] = std::make_shared<Agv>(nh, "/ariac/agv2/state");

  arm_state_ = std::make_shared<ArmState>(nh, "/ariac/joint_states");
  // Init order manager
  order_manager_ = std::make_shared<Order>(nh, "/ariac/orders");
  ROS_DEBUG_STREAM("Manager is init..");
}

Manager::~Manager() { inventory_.clear(); }

void Manager::checkInventory() {
  // Wait until camera see
  do {
    ROS_INFO_STREAM("Scanning the item");
    ros::spinOnce();
    rate_->sleep();
  } while (!logical_camera_1_->isPopulated() &&
           !logical_camera_2_->isPopulated());

  // reset inventory for update
  inventory_.clear();
  std::string frame;
  CameraMsg image_msg;

  for (auto itr : {logical_camera_1_, logical_camera_2_}) {
    // add parts from Camera
    image_msg = itr->getMessage();
    frame = itr->getSensorFrame();
    for (const auto &part : image_msg->models) {
      geometry_msgs::PoseStamped p_;
      p_.pose = part.pose;
      p_.header.frame_id = frame;
      inventory_[part.type].emplace_back(p_);
    }
  }
}

geometry_msgs::PoseStamped Manager::getPart(const std::string &partType) {
  // geometry_msgs::PoseStamped part = inventory_[partType].front();
  // assumption inventory has enough part
  if (inventory_[partType].empty())
    checkInventory();
  auto it = inventory_[partType].begin();
  geometry_msgs::PoseStamped part = *it;
  inventory_[partType].erase(it);
  return part;
}
/// Start the competition by waiting for and then calling the start ROS
/// Service.
void Manager::start_competition(std::string topic) const {
  // Create a Service client for the correct service, i.e.
  // '/ariac/start_competition'.
  ros::ServiceClient start_client =
      nh_->serviceClient<std_srvs::Trigger>(topic);
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would
  // fail.
  if (!start_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger srv;  // Combination of the "request" and the "response".
  start_client.call(srv); // Call the start Service.
  if (!srv.response.success) { // If not successful, print out why.
    ROS_ERROR_STREAM(
        "Failed to start the competition: " << srv.response.message);
  } else {
    ROS_INFO("Competition started!");
  }
}

/// End the competition by waiting for and then calling the start ROS
/// Service.
void Manager::end_competition(std::string topic) const {
  // Create a Service client for the correct service, i.e.
  // '/ariac/end_competition'.
  ros::ServiceClient start_client =
      nh_->serviceClient<std_srvs::Trigger>(topic);
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would
  // fail.
  if (!start_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready to finish.");
  }
  ROS_INFO("Requesting competition end...");
  std_srvs::Trigger srv;  // Combination of the "request" and the "response".
  start_client.call(srv); // Call the start Service.
  if (!srv.response.success) { // If not successful, print out why.
    ROS_ERROR_STREAM("Failed to end the competition: " << srv.response.message);
  } else {
    ROS_INFO("Competition ended!");
  }
}

void Manager::send_order(std::string agv, std::string kit_id) const {
  // Create a Service client for the correct service, i.e. '/ariac/agv1'.
  ros::ServiceClient agv_client =
      nh_->serviceClient<osrf_gear::AGVControl>(agv);
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would
  // fail.
  if (!agv_client.exists()) {
    ROS_INFO("Waiting for the AGV client to be ready...");
    agv_client.waitForExistence();
    ROS_INFO("AGV client is now ready.");
  }
  ROS_INFO("Requesting AGV to complete order...");
  osrf_gear::AGVControl srv; // Combination of the "request" and the "response".
  srv.request.kit_type = kit_id;
  agv_client.call(srv); // Call the start Service.
  if (!srv.response.success) {
    ROS_ERROR_STREAM("Failed to send");
  }
}

// void Manager::processCameraMsg(const CameraMsg& msg){
//   return;
// }

OrderMsg Manager::getTheOrderMsg() {
  while (!order_manager_->isPopulated()) {
    ROS_WARN_STREAM("Waiting for order");
    ros::spinOnce();
    rate_->sleep();
  }

  return order_manager_->getMessage();
}

std::vector<geometry_msgs::Pose>
Manager::look_over_tray(const geometry_msgs::Pose &target,
                        const std::string &partType, const int &agv) {
  auto camera = agv == 0 ? logical_camera_3_ : logical_camera_4_;

  do {
    ROS_INFO_STREAM("Looking over tray...");
    ros::spinOnce();
    rate_->sleep();
  } while (!camera->isPopulated());

  std::vector<geometry_msgs::Pose> v;
  v.reserve(1);

  geometry_msgs::PoseStamped temp, result;
  temp.header.frame_id = camera->getSensorFrame();
  auto msg = camera->getMessage();
  auto frame = camera->getSensorFrame();
  for (const auto &part : msg->models) {
    if (part.type.compare(partType) == 0) {
      temp.pose = part.pose;
      result = getPose(temp);
      if (!is_same(result.pose, target)) {
        v.emplace_back(result.pose);
        return v;
      }
    }
  }
  return v;
}

bool Manager::isAgvReady(const int &no) {
  while (!agv_[no]->isPopulated()) {
    ROS_INFO_STREAM("Agv status check....");
    ros::spinOnce();
    rate_->sleep();
  }
  auto msg = agv_[no]->getMessage();
  return msg->data.compare("ready_to_deliver") == 0;
}

int Manager::pick_agv() {
  ROS_INFO_STREAM("Checking agvs state.");
  while (!isAgvReady(0) || !isAgvReady(1)) {
    ROS_WARN_STREAM("Both Agvs are busy!!");
    ros::spinOnce();
    rate_->sleep();
  }
  return isAgvReady(0) ? 0 : 1;
}
