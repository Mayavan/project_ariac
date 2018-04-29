
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
#include <vector>
/**
 * @TODO(ravib)
 * pick up moving part
 * choose agv method for multitasking
 * refactor high level logic
 * remove unnecessary shared_ptr and make it unique_ptr
 * doxygen doc comment
 */

#define CAMERA_OVER_TRAY_ONE 1
#define CAMERA_OVER_TRAY_TWO 2

int main(int argc, char **argv) {
  ros::init(argc, argv, "project_ariac_node");

  ros::NodeHandle node;
  ros::NodeHandle private_node_handle("~");

  bool run, result, priority_order;
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
  std::vector<osrf_gear::KitObject> pending_task;
  std::vector<manager::OrderMsg> ariac_order = {m.getTheOrderMsg()};
  // wait for order
  // for (const auto &kit : ariac_order->kits) {
  //   auto tasks = kit.objects;
  priority_order = false;
  while (!ariac_order.empty()) {
    auto kits = ariac_order.back()->kits;
    // ur10.move(ur10.home_joint_angle_);
    while (!kits.empty()) {
      // TODO(ravib)
      // not proper but its work around for competition
      // refactor needo
      auto &tasks = kits.front().objects;
      // int agv = m.pick_agv();
      if (!priority_order && !pending_task.empty()) {
        tasks = pending_task;
      }
      int agv = priority_order ? 1 : 0;
      priority_order = false;
      // Do until every part is placed
      while (!tasks.empty()) {
        auto part = tasks.front();
        ROS_INFO_STREAM("Pickup :" << part.type);
        auto p = m.getPart(part.type);
        // pick up part
        result = ur10.robust_pickup(p, part.type); // max_try = 1

        ROS_INFO_STREAM("Pick up complete:" << result);
        // if success than place or pick another part

        std::string camera_frame =
            agv ? "logical_camera_" + std::to_string(CAMERA_OVER_TRAY_TWO) +
                      "_kit_tray_2_frame"
                : "logical_camera_" + std::to_string(CAMERA_OVER_TRAY_ONE) +
                      "_kit_tray_1_frame";
        if (result) {
          // place
          result = ur10.robust_place(part.pose, camera_frame, agv);
          // place failed
          if (result) {
            tasks.erase(
                tasks.begin()); // part place sucess thn remove from tasks list
          }
        }
        if (m.isHighOrder()) {
          priority_order = true; // complete the high priority order with agv0;
          ariac_order.push_back(m.getTheOrderMsg());
          pending_task = tasks;
          break;
        }
      }
      if (priority_order)
        break;
      // after completing all task send kit
      m.send_order("/ariac/agv" + std::to_string(agv + 1),
                   kits.front().kit_type);
      kits.erase(kits.begin());
      // ros::Duration(20.0).sleep(); // wait for agv to avilable
    }
    if (!priority_order) {
      ariac_order.pop_back();
    }
  }
  ros::Duration(5.0).sleep();
  while (!m.isAgvReady(0) && !m.isAgvReady(1)) {
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }
  m.end_competition();
  return 0;
}