
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
      // part.type and part.pose;
      do {
        ROS_INFO_STREAM("Pickup :" << part.type);
        transform = ur10.getTransfrom("/world", m.getPart(part.type));
        target_pick.position.x = transform.getOrigin().x();
        target_pick.position.y = transform.getOrigin().y();
        target_pick.position.z = transform.getOrigin().z();

        result = ur10.pickup(target_pick);
        //
        if (!result) {
          ROS_WARN_STREAM("Pickup failed");
        }
        ur10.move(ur10.home_);
      } while (!result);

      if (result) {
        ROS_INFO_STREAM("Place :" << part.type);
        target_place =
            m.findPose(part.pose, "logical_camera_3_kit_tray_1_frame");
        result = ur10.place(target_place);
        if (!result) {
          ROS_WARN_STREAM("Place failed");
          int count = part.type == "gear_part" ? gear : piston;
          std::string partFrame = "logical_camera_3_" + part.type + "_" +
                                  std::to_string(count) + "_frame";
          ROS_WARN_STREAM(partFrame);
          transform = ur10.getTransfrom("/world", partFrame);
          target_pick.position.x = transform.getOrigin().x();
          target_pick.position.y = transform.getOrigin().y();
          target_pick.position.z = transform.getOrigin().z();

          auto target = ur10.agv_;
          target.position.z += 0.5;
          ur10.move({target});
          result = ur10.pickup(target_pick);
          target_place.position.z += 0.025;
          target_place.orientation = ur10.home_.orientation;
          std::vector<geometry_msgs::Pose> waypoint = {target_place};
          ur10.move({target});
          result = ur10.place(waypoint);
          ur10.move({ur10.home_});
        } else {
          if (part.type == "gear_part")
            gear++;
          else
            piston++;
        }
      }
    }
    std::stringstream ss("order_0_kit_");
    ss << count;
    m.send_order("/ariac/agv1", ss.str());
    count++;
    ros::Duration(20.0).sleep();
  }
  return 0;
}
