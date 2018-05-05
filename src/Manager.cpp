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

  conveyor_ = std::make_shared<manager::Camera>(nh, "project_ariac/conveyer");

  agv_[0] = std::make_shared<manager::Agv>(nh, "/ariac/agv1/state");
  agv_[1] = std::make_shared<manager::Agv>(nh, "/ariac/agv2/state");

  arm_state_ = std::make_shared<manager::ArmState>(nh, "/ariac/joint_states");
  // Init order manager
  order_manager_ = std::make_shared<manager::Order>(nh, "/ariac/orders");

  ROS_DEBUG_STREAM("Manager is init..");
  configType = 0;
  initialize();
}

Manager::~Manager() { inventory_.clear(); }


void Manager::initialize() {
  config1.push_back({-0.0666047, 0.199258, 0.00536679, 0, 0, 1, 0});
  config1.push_back({0.0668974, 0.201598, 0.00413541, 0, 0, 1, 0});
  config1.push_back({-0.199113, 0.0678644, 00446563, 0, 0, 1, 0});
  config1.push_back({-0.0672372, 0.0647241, 0.00591626, 0, 0, 1, 0});
  config1.push_back({0.0661366, 0.0671152, 0.00519474, 0, 0, 1, 0});
  config1.push_back({0.198912,  0.0650551, 0.00377798, 0, 0, 1, 0});
  config1.push_back({-0.197683, -0.0657141,  0.00484823, 0, 0, 1, 0});
  config1.push_back({-0.0640238,  -0.0684328,  0.00364693, 0, 0, 1, 0});
  config1.push_back({0.0662084, -0.0673079,  0.0060487, 0, 0, 1, 0});
  config1.push_back({0.199804, -0.0661817,  0.00610662, 0, 0, 1, 0});
  config1.push_back({-0.199279, -0.199742, 0.00490696, 0, 0, 1, 0});
  config1.push_back({-0.0675575,  -0.200984, 0.00419706, 0, 0, 1, 0});
  config1.push_back({0.0660187, -0.200665, 0.00626196, 0, 0, 1, 0});
  config1.push_back({0.200801,  -0.200029, 0.00566913, 0, 0, 1, 0});
  config2.push_back({-0.0670033, 0.199958,  0.00363661, 0, 0, 1, 0});
  config2.push_back({0.0651708, 0.198379,  0.00375392, 0, 0, 1, 0});
  config2.push_back({0.199832,  0.199404,  0.00503828, 0, 0, 1, 0});
  config2.push_back({-0.199653, -0.00108894, 0.0042945, 0, 0, 1, 0});
  config2.push_back({-0.0668228, 0.00133815, 0.00624269, 0, 0, 1, 0});
  config2.push_back({0.0671111, 0.000249411, 0.00520442, 0, 0, 1, 0});
  config2.push_back({0.199493,  0.00021796,  0.00498405, 0, 0, 1, 0});
  config2.push_back({-0.200136, -0.200951, 0.00390516, 0, 0, 1, 0});
  config2.push_back({-0.0668235,  -0.198879, 0.00580873, 0, 0, 1, 0});
  config2.push_back({0.0671068, -0.19988,  0.00458785, 0, 0, 1, 0});
  config2.push_back({0.19891, -0.198302, 0.00399941, 0, 0, 1, 0});
  config3.push_back({-0.0996682,  0.100102,  0.00587586, 0, 0, 1, 0});
  config3.push_back({-0.0996682,  0.100102,  0.00587586, 0, 0, 1, 0});
  config3.push_back({-0.10264,  -0.101338, 0.00608485, 0, 0, 1, 0});
  config3.push_back({0.0992714, -0.100459, 0.00359961, 0, 0, 1, 0});
  config4.push_back({-0.149318, 0.150378,  0.00395726, 0, 0, 1, 0});
  config4.push_back({0.100341,  0.150105,  0.00510879, 0, 0, 1, 0});
  config4.push_back({-0.149719, 0.0252709, 0.00539186, 0, 0, 1, 0});
  config4.push_back({0.0975749, 0.0251252, 0.00374488, 0, 0, 1, 0});
  config4.push_back({-0.150346, -0.101535, 0.00386775, 0, 0, 1, 0});
  config4.push_back({0.100247,  -0.100091, 0.00384834, 0, 0, 1, 0});
}

void Manager::Inventory() {
  inventory_.clear();
  for (int i = 0; i < 4; i++) {
    geometry_msgs::PoseStamped pos;
    if(i == 0) {
      for (const auto &part : config1) {
        pos.pose.position.x = part[0];
        pos.pose.position.y = part[1];
        pos.pose.position.z = part[2];
        pos.pose.orientation.x = part[3];
        pos.pose.orientation.y = part[4];
        pos.pose.orientation.z = part[5];
        pos.pose.orientation.w = part[6];
        inventory_[i].emplace_back(pos);
      }
    }
    if(i == 1) {
      for (const auto &part : config2) {
        pos.pose.position.x = part[0];
        pos.pose.position.y = part[1];
        pos.pose.position.z = part[2];
        pos.pose.orientation.x = part[3];
        pos.pose.orientation.y = part[4];
        pos.pose.orientation.z = part[5];
        pos.pose.orientation.w = part[6];
        inventory_[i].emplace_back(pos);
      }
    }
    if(i == 2) {
      for (const auto &part : config3) {
        pos.pose.position.x = part[0];
        pos.pose.position.y = part[1];
        pos.pose.position.z = part[2];
        pos.pose.orientation.x = part[3];
        pos.pose.orientation.y = part[4];
        pos.pose.orientation.z = part[5];
        pos.pose.orientation.w = part[6];
        inventory_[i].emplace_back(pos);
      }
    }
    if(i == 3) {
      for (const auto &part : config4) {
        pos.pose.position.x = part[0];
        pos.pose.position.y = part[1];
        pos.pose.position.z = part[2];
        pos.pose.orientation.x = part[3];
        pos.pose.orientation.y = part[4];
        pos.pose.orientation.z = part[5];
        pos.pose.orientation.w = part[6];
        inventory_[i].emplace_back(pos);
      }
    }
  }
}

geometry_msgs::PoseStamped Manager::getPart(const std::string &partType) {
  geometry_msgs::PoseStamped part;
  if (partType == "belt") {
    for (const auto &itr_part : conveyor_->getMessage()->models) {
      if (itr_part.type == partType) {
        // find pickable part
        if (itr_part.pose.position.y > 0 && itr_part.pose.position.y < 2.0) {
          part.pose = itr_part.pose;
          part.header.frame_id = "world";
          ROS_INFO_STREAM("part found on conveyor");
          return part;
        }
      }
    }
  } else {
    if (!inventory_[configType].empty()) {
      auto it = inventory_[configType].begin();
      part = *it;
      inventory_[configType].erase(it);
      ROS_INFO_STREAM("part found on bin");
      part.header.frame_id = partType;
      return part;
    }
  }
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

double Manager::getConveyorSpeed() {
  auto getspeed =
      nh_->serviceClient<project_ariac::getSpeed>("/conveyer/getSpeed");
  if (!getspeed.exists()) {
    ROS_INFO("Waiting for the getSpeed client to be ready...");
    getspeed.waitForExistence();
    ROS_INFO("conveyor speed client is now ready.");
  }
  ROS_INFO("Requesting speed to complete pick...");
  project_ariac::getSpeed srv;
  getspeed.call(srv);

  return srv.response.speed;
}

std::string Manager::get_BinId(std::string part, std::string topic) const {
  // Create a Service client for the correct service, i.e. '/ariac/agv1'.
  //geometry_msgs::PoseStamped part;
  std::string id = "invalid";
  ros::ServiceClient bin_client =
      nh_->serviceClient<osrf_gear::GetMaterialLocations>(topic);
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would
  // fail.    
  if (!bin_client.exists()) {
    ROS_INFO("Waiting for the AGV client to be ready...");
    bin_client.waitForExistence();
    ROS_INFO("AGV client is now ready.");
  }
  osrf_gear::GetMaterialLocations srv;
  srv.request.material_type = part;
  bin_client.call(srv);
  for (const auto& i : srv.response.storage_units) {
    std::size_t found1 = i.unit_id.find("bin");
    std::size_t found2 = i.unit_id.find("belt");
    if (found1 != std::string::npos) {    
      id = i.unit_id + "_frame";
      break;
    }  
    else if (found2 != std::string::npos)
      id = "belt";
  }
  return id;
}

manager::OrderMsg Manager::getTheOrderMsg() {
  while (!order_manager_->isPopulated()) {
    ROS_WARN_STREAM("Waiting for order");
    ros::spinOnce();
    rate_->sleep();
  }
  return order_manager_->getMessage();
}

bool Manager::isHighOrder() {
  if (order_manager_->getCounter() > 1) {
    order_manager_->setCounter();
    return true;
  }
  return false;
}

std::vector<geometry_msgs::PoseStamped>
Manager::look_over_tray(const geometry_msgs::Pose &target,
                        const std::string &partType, const int &agv) {
  auto camera = agv == 0 ? logical_camera_1_ : logical_camera_2_;

  do {
    ROS_INFO_STREAM("Looking over tray...");
    ros::spinOnce();
    rate_->sleep();
  } while (!camera->isPopulated());

  std::vector<geometry_msgs::PoseStamped> v;
  v.reserve(1);

  geometry_msgs::PoseStamped temp, result;
  temp.header.frame_id = camera->getSensorFrame();
  auto msg = camera->getMessage();
  for (const auto &part : msg->models) {
    if (part.type.compare(partType) == 0) {
      temp.pose = part.pose;
      result = getPose(temp);
      if (!is_same(result.pose, target)) {
        result.header.frame_id = "world";
        v.emplace_back(result);
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
  if (isAgvReady(1))
    return 1;
  if (isAgvReady(0))
    return 0;
  do {
    ros::spinOnce();
    ROS_WARN_STREAM("Both Agvs are busy!!");
    rate_->sleep();
  } while (!isAgvReady(0) && !isAgvReady(1));

  return isAgvReady(1) ? 1 : 0;
}
