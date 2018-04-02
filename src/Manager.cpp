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
  rate_ = std::make_shared<ros::Rate>(0.5);
  // Init Cameras
  logical_camera_1_ = std::make_shared<Camera>(nh, "/ariac/logical_camera_1");
  logical_camera_2_ = std::make_shared<Camera>(nh, "/ariac/logical_camera_2");
  logical_camera_3_ = std::make_shared<Camera>(nh, "/ariac/logical_camera_3");
  // Init order manager
  order_manager_ = std::make_shared<Order>(nh, "/ariac/orders");
  ROS_DEBUG_STREAM("Manager is init..");
}

Manager::~Manager() { inventory_.clear(); }

void Manager::checkInventory() {
  // Wait until camera see
  while (!logical_camera_1_->isPopulated() &&
         !logical_camera_2_->isPopulated()) {
    ROS_INFO_STREAM("Scanning the item");
    ros::spinOnce();
    rate_->sleep();
  }

  // reset inventory for update
  inventory_.clear();

  // add parts from Camera 1
  size_t count = 1;
  auto image_msg = logical_camera_1_->getMessage();
  for (const auto& part : image_msg->models) {
    std::string partFrame = "logical_camera_1_" + part.type + "_" +
                            std::to_string(count) + "_frame";
    ROS_DEBUG_STREAM(partFrame);
    inventory_[part.type].push_back(partFrame);
    count++;
  }

  // add parts from Camera 2
  count = 1;
  image_msg = logical_camera_2_->getMessage();
  for (const auto& part : image_msg->models) {
    std::string partFrame = "logical_camera_2_" + part.type + "_" +
                            std::to_string(count) + "_frame";
    ROS_DEBUG_STREAM(partFrame);
    inventory_[part.type].push_back(partFrame);
    count++;
  }
}

// void Manager::finishOrder() {
//   // wait for order
//   bool result;
//   for (const auto& kit : order_msg->kits) {
//     for (const auto& part : kit.objects) {
//       // TODO(ravib)
//       // part.type and part.pose;
//     }
//   }
// }

std::string Manager::getPart(const std::string& partType) {
  std::string part = inventory_[partType].front();
  inventory_[partType].pop_front();
  return part;
}

/// Start the competition by waiting for and then calling the start ROS Service.
void Manager::start_competition(std::string topic) const {
  // Create a Service client for the correct service, i.e.
  // '/ariac/start_competition'.
  ros::ServiceClient start_client =
      nh_->serviceClient<std_srvs::Trigger>(topic);
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!start_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger srv;   // Combination of the "request" and the "response".
  start_client.call(srv);  // Call the start Service.
  if (!srv.response.success) {  // If not successful, print out why.
    ROS_ERROR_STREAM(
        "Failed to start the competition: " << srv.response.message);
  } else {
    ROS_INFO("Competition started!");
  }
}

/// End the competition by waiting for and then calling the start ROS Service.
void Manager::end_competition(std::string topic) const {
  // Create a Service client for the correct service, i.e.
  // '/ariac/end_competition'.
  ros::ServiceClient start_client =
      nh_->serviceClient<std_srvs::Trigger>(topic);
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!start_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready to finish.");
  }
  ROS_INFO("Requesting competition end...");
  std_srvs::Trigger srv;   // Combination of the "request" and the "response".
  start_client.call(srv);  // Call the start Service.
  if (!srv.response.success) {  // If not successful, print out why.
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
  // Calling the Service using the client before the server is ready would fail.
  if (!agv_client.exists()) {
    ROS_INFO("Waiting for the AGV client to be ready...");
    agv_client.waitForExistence();
    ROS_INFO("AGV client is now ready.");
  }
  ROS_INFO("Requesting AGV to complete order...");
  osrf_gear::AGVControl
      srv;  // Combination of the "request" and the "response".
  srv.request.kit_type = kit_id;
  agv_client.call(srv);         // Call the start Service.
  if (!srv.response.success) {  // If not successful, print out why.
    ROS_ERROR_STREAM("Failed to send");
  }
}

// void Manager::processCameraMsg(const CameraMsg& msg){
//   return;
// }

geometry_msgs::Pose Manager::findPose(const geometry_msgs::Pose& inPose,
                                      const std::string& header) {
  geometry_msgs::PoseStamped inPose_;
  geometry_msgs::PoseStamped outPose_;

  inPose_.header.frame_id = header;
  inPose_.pose = inPose;
  inPose_.header.stamp = ros::Time(0);

  listener_.waitForTransform("/world", header, inPose_.header.stamp,
                             ros::Duration(10));
  try {
    listener_.transformPose("/world", inPose_, outPose_);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(0.5).sleep();
  }
  return outPose_.pose;
}

OrderMsg Manager::getTheOrderMsg() {
  while (!order_manager_->isPopulated()) {
    ROS_WARN_STREAM("Waiting for order");
    ros::spinOnce();
    rate_->sleep();
  }

  return order_manager_->getMessage();
}

std::vector<geometry_msgs::Pose> Manager::look_over_tray(
    const std::string& part_type) {
  ros::spinOnce();
  rate_->sleep();
  while (!logical_camera_3_->isPopulated()) {
    ROS_INFO_STREAM("Looking over tray....");
    ros::spinOnce();
    rate_->sleep();
  }
  std::vector<geometry_msgs::Pose> temp;
  auto image_msg = logical_camera_3_->getMessage();
  for (const auto& part : image_msg->models) {
    if (part.type.compare(part_type) == 0) {
      temp.push_back(part.pose);
    }
  }
}

float Manager::distance(const geometry_msgs::Pose& current,
                        const geometry_msgs::Pose& target) {
  return std::sqrt(std::pow(current.position.x - target.position.x, 2.0) +
                   std::pow(current.position.y - target.position.y, 2.0) +
                   std::pow(current.position.z - target.position.z, 2.0));
}
