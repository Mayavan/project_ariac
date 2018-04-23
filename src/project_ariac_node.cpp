
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
 * TODO(ravib)
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

  bool result, run;
  private_node_handle.param("run", run, false);
  UR10_Control ur10(private_node_handle);

  geometry_msgs::Pose target_pick, target_place;
  tf::StampedTransform transform;

  Manager m(node);

  // start
  m.start_competition();
  // update inventory
  m.checkInventory();
  // take order
  std::vector<manager::OrderMsg> ariac_order = {m.getTheOrderMsg()};
  // wait for order
  while (!ariac_order.empty()) {

    // BUG modifing local copy bug
    auto kits = ariac_order.back()->kits;

    while (!kits.empty()) {
      // TODO(ravib)
      // not proper but its work around for competition
      // refactor need
      auto tasks = kits.front().objects;
      // int agv = m.pick_agv();
      int agv = 1;
      // Do until every part is placed
      while (!tasks.empty()) {
        auto part = tasks.front();

        geometry_msgs::PoseStamped p;

        do {
          ROS_INFO_STREAM("Finding :" << part.type);
          p = m.getPart(part.type);
        } while (p.header.frame_id == "invalid");
        // pick up part

        ROS_INFO_STREAM("Pickup :" << part.type);
        // is it over conveyor?
        if (p.header.frame_id == "world") {
          result = ur10.conveyor_pickup(p.pose);
        } else {
          result = ur10.robust_pickup(p, 1); // max_try = 1
        }

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
              if (ur10.robust_pickup(v.front(), 1))
                if (ur10.place(p.pose, agv))
                  tasks.erase(tasks.begin());
            }
          } else {
            tasks.erase(
                tasks.begin()); // part place sucess thn remove from tasks list
          }
        }
        // check high priority and push_back and break;
      }
      // after completing all task send kit
      m.send_order("/ariac/agv" + std::to_string(agv + 1),
                   kits.front().kit_type);
      kits.erase(kits.begin());
      ur10.move(ur10.home_joint_angle_);
      ros::Duration(20.0).sleep(); // wait for agv to avilable
    }
    ariac_order.pop_back();
  }

  m.end_competition();
  return 0;
}
