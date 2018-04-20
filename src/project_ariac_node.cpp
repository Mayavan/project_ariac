
/**
 * @file project_ariac_node.cpp
 * @author     Ravi Bhadeshiya
 * @version    2.0
 * @brief      The main node for project_ariac
 *
 * @copyright  BSD 3-Clause License (c) 2018 Ravi Bhadeshiya
 **/
#include "project_ariac/Manager.hpp"
#include "project_ariac/UR10_Control.hpp"
#include <ros/ros.h>
#include <sstream>
/**
 * @TODO(ravib)
 * pick up moving part
 * choose agv method for multitasking
 * refactor high level logic
 * remove unnecessary shared_ptr and make it unique_ptr
 * doxygen doc comment
 */

#define CAMERA_OVER_TRAY_ONE 3
#define CAMERA_OVER_TRAY_TWO 4

int main(int argc, char **argv) {
  ros::init(argc, argv, "project_ariac_node");

  ros::NodeHandle node;
  ros::NodeHandle private_node_handle("~");

  bool run, result;
  int count = 0;
  // private_node_handle.param("run", run, false);
  UR10_Control ur10(private_node_handle);

  geometry_msgs::Pose target_pick, target_place;
  tf::StampedTransform transform;

  Manager m(node);
  // start
  m.start_competition();
  // update inventory
  m.checkInventory();
  // take order
  auto ariac_order = m.getTheOrderMsg();
  // wait for order
  for (const auto &kit : ariac_order->kits) {
    auto tasks = kit.objects;
    // TODO(ravib)
    // not proper but its work around for competition
    // refactor need
    int agv = m.pick_agv();
    // Do until every part is placed
    while (!tasks.empty()) {
      auto part = tasks.front();
      ROS_INFO_STREAM("Pickup :" << part.type);
      auto p = m.getPart(part.type);
      // pick up part
      result = ur10.robust_pickup(p, 1); // max_try = 1
      ROS_INFO_STREAM("Pick up complete:" << result);
      // if scuess than place or pick another part

      std::string camera_frame =
          agv ? "logical_camera_" + std::to_string(CAMERA_OVER_TRAY_TWO) +
                    "_kit_tray_2_frame"
              : "logical_camera_" + std::to_string(CAMERA_OVER_TRAY_ONE) +
                    "_kit_tray_1_frame";
      if (result) {
        // place
        result = ur10.robust_place(part.pose, camera_frame, agv);
        // place failed
        if (!result) {
          // check over tray
          p.pose = m.getPose(part.pose, camera_frame);
          auto v = m.look_over_tray(p.pose, part.type, agv);
          // if tray has part than pick and place to correct postion
          if (!v.empty()) {
            ROS_INFO_STREAM("Incorrect postion on tray found");
            if (ur10.pickup(v.front()))
              if (ur10.place(p.pose, agv))
                tasks.erase(tasks.begin());
          }
        } else {
          tasks.erase(
              tasks.begin()); // part place sucess thn remove from tasks list
        }
      }
    }
    // after completing all task send order
    std::string ss = "order_0_kit_" + std::to_string(count);
    ROS_WARN_STREAM(ss);
    m.send_order("/ariac/agv" + std::to_string(agv + 1), ss);
    count++;
    ur10.move(ur10.home_joint_angle_);
    // ros::Duration(20.0).sleep(); // wait for agv to avilable
  }
  m.end_competition();
  return 0;
}
