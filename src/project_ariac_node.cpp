
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
#include <fstream>
#include <string>
#include "project_ariac/Manager.hpp"

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

int main(int argc, char** argv) {
  ros::init(argc, argv, "project_ariac_node");

  ros::NodeHandle node;
  ros::NodeHandle private_node_handle("~");

  std::string fileName, outputDir;
  private_node_handle.param<std::string>("fileName", fileName, " ");
  private_node_handle.param<std::string>("ouputDir", outputDir, " ");

  start_competition(node);

  Manager manager(node);

  while (!manager.isOrderReady()) {
    ros::Duration(1.0).sleep();
    ros::spinOnce();
  }

  auto order = manager.order_msg_;
  int gear, piston;
  for (const auto& kit : order->kits) {
    for (const auto& itr : kit.objects) {
      if (itr.type == "gear")
        gear++;
      else if (iter.type == "piston")
        piston++;
    }
  }

  std::fstream inputFile(fileName);
  std::stringstream outFile;
  // std::fstream outFile(outputDir+"/updated_ariac-problem.pddl");
  if (file.good() && !file.eof()) {
    ROS_INFO_STREAM("File opened!");
    string line;
    stringstream output;
    while (getline(inputFile, line)) {
      stringstream ss;
      auto index = line.find("(=(No-of-parts-in-order order)");
      if (index != string::npos) {inputFinputFileile
        ss << "    (=(No-of-parts-in-order order) " << gear + piston << ")"
           << std::endl;
        for (size_t i = 1; i <= gear; i++) {
          ss << "    (orderContain order gear" << i << ")" << std::endl;
        }
        for (size_t i = 1; i <= piston; i++) {
          ss << "    (orderContain order piston" << i << ")" << std::endl;
        }
      }
      auto index1 = line.find("orderContain");
      if (index1 == std::string::npos && index == std::string::npos) ss << line;
      outFile << ss.str() << std::endl;
      ROS_INFO_STREAM(ss.str());
    }
    inputFile.str(std::string());
    inputFile << outFile.str();
    ROS_INFO_STREAM("Done!!");
  } else {
    ROS_ERROR_STREAM("Unable to load file:" << fileName);
  }

  inputFile.close();
  // outFile.close();
  return 0;
}
