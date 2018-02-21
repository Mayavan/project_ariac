
/**
 * @file project_ariac_node.cpp
 * @author     Ravi Bhadeshiya
 * @version    1.0
 * @brief      The main node for project_ariac
 *
 * @copyright  BSD 3-Clause License (c) 2018 Ravi Bhadeshiya
 **/

#include <osrf_gear/AGVControl.h>
#include <osrf_gear/Order.h>
#include <std_srvs/Trigger.h>

#include "project_ariac/Manager.hpp"
#include "project_ariac/UR10_Control.hpp"

/// Start the competition by waiting for and then calling the start ROS Service.
void start_competition(ros::NodeHandle& node) {
  // Create a Service client for the correct service, i.e.
  // '/ariac/start_competition'.
  ros::ServiceClient start_client =
      node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
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
void end_competition(ros::NodeHandle& node) {
  // Create a Service client for the correct service, i.e.
  // '/ariac/end_competition'.
  ros::ServiceClient start_client =
      node.serviceClient<std_srvs::Trigger>("/ariac/end_competition");
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

void send_order(ros::NodeHandle& node) {
  // Create a Service client for the correct service, i.e. '/ariac/agv1'.
  ros::ServiceClient agv_client =
      node.serviceClient<osrf_gear::AGVControl>("/ariac/agv1");
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
  srv.request.kit_type = "order_0_kit_0";
  agv_client.call(srv);         // Call the start Service.
  if (!srv.response.success) {  // If not successful, print out why.
    ROS_ERROR_STREAM("Failed to send");
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ariac_example_node");

  ros::NodeHandle node;

  UR10_Control ur10;
  // target.position.x = -0.5;
  // target.position.y = -0.735;
  // target.position.z = 0.724;

  // ur10.pickAndPlace(target);

  Manager mangement(node);

  ROS_INFO_STREAM("Manager is ready");
  ros::Rate rate(1.0);

  while (!mangement.isReady()) {
    ROS_INFO_STREAM("wait for scanning process");
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO_STREAM("Starting competition");

  start_competition(node);

  while (!mangement.isOrderReady()) {
    ROS_INFO_STREAM("wait for order");
    ros::spinOnce();
    rate.sleep();
  }

  geometry_msgs::Pose target;
  tf::StampedTransform transform;

  for (const auto& part : mangement.getOrder()) {
    ROS_INFO_STREAM(part.first);
    for (const auto& itr : part.second) {
      transform = ur10.getTransfrom("/world", itr);

      target.position.x = transform.getOrigin().x();
      target.position.y = transform.getOrigin().y();
      target.position.z = transform.getOrigin().z();

      ROS_INFO_STREAM(">>>>>>>" << itr);
      ur10.pickAndPlace(target);
    }
  }

  send_order(node);
  end_competition(node);

  return 0;
}
