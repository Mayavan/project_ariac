
/**
 * @file project_ariac_node.cpp
 * @author     Ravi Bhadeshiya
 * @version    2.0
 * @brief      The main node for project_ariac
 *
 * @copyright  BSD 3-Clause License (c) 2018 Ravi Bhadeshiya
 **/
#include <ros/ros.h>
#include <sstream>
#include "project_ariac/Manager.hpp"
#include "project_ariac/UR10_Control.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "project_ariac_node");

  ros::NodeHandle node;
  ros::NodeHandle private_node_handle("~");

  bool run, result;
  int count = 0;
  private_node_handle.param("run", run, false);

  UR10_Control ur10(private_node_handle);

  geometry_msgs::Pose target_pick, target_place;
  tf::StampedTransform transform;

  Manager m(node);
  m.start_competition();
  m.checkInventory();
  auto ariac_order = m.getTheOrderMsg();
  // wait for order
  for (const auto& kit : ariac_order->kits) {
    int piston = 1, gear = 1;
    for (const auto& part : kit.objects) {
      // TODO(ravib)
      ROS_INFO_STREAM("Pickup :" << part.type);
      auto p = m.getPart(part.type);
      result = ur10.robust_pickup(p);

      result =
          ur10.robust_place(part.pose, "logical_camera_3_kit_tray_1_frame", 0);
    }
    std::stringstream ss("order_0_kit_");
    ss << count;
    m.send_order("/ariac/agv1", ss.str());
    count++;
    // ur10.move(ur10.home_joint_angle_);
    ros::Duration(20.0).sleep();
  }
  return 0;
}
