
/**
 * @file project_ariac_node.cpp
 * @author     Ravi Bhadeshiya
 * @version    2.0
 * @brief      The main node for project_ariac
 *
 * @copyright  BSD 3-Clause License (c) 2018 Ravi Bhadeshiya
 **/
#include <ros/ros.h>
#include "project_ariac/Manager.hpp"
#include "project_ariac/UR10_Control.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "project_ariac_node");

  ros::NodeHandle node;
  ros::NodeHandle private_node_handle("~");

  bool run, result;
  private_node_handle.param("run", run, false);

  UR10_Control ur10(private_node_handle);

  geometry_msgs::Pose target;
  tf::StampedTransform transform;

  Manager m(node);
  m.start_competition();
  m.checkInventory();
  auto ariac_order = m.getTheOrderMsg();
  // wait for order
  for (const auto& kit : ariac_order->kits) {
    for (const auto& part : kit.objects) {
      // TODO(ravib)
      // part.type and part.pose;
      transform = ur10.getTransfrom("/world", m.getPart(part.type));
      target.position.x = transform.getOrigin().x();
      target.position.y = transform.getOrigin().y();
      target.position.z = transform.getOrigin().z();

      do {
        ROS_INFO_STREAM("Pickup :" << part.type);
        result = ur10.pickup(target);
        ur10.move(ur10.home_);
        if (!result) {
          ROS_WARN_STREAM("Pickup failed");
        }
      } while (!result);

      if (result) {
        ROS_INFO_STREAM("Place :" << part.type);
        target = m.findPose(part.pose, "logical_camera_3_frame");
        if (!result) ROS_WARN_STREAM("Palce failed");
        result = ur10.place(target);
      }
    }
    m.send_order();
  }
  return 0;
}

