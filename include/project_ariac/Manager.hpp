#pragma once

// #include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <set>
// #include <cwru_ariac/Part.h>
#include <osrf_gear/Order.h>
#include <string>
#include <list>
#include <map>

using partlist = map<std::string, std::list<std::string>>;

class Manager {
 public:
  Manager();
  ~Manager();
  void logical_camera_callback_1(
      const osrf_gear::LogicalCameraImage::ConstPtr &image_msg);
  void logical_camera_callback_2(
      const osrf_gear::LogicalCameraImage::ConstPtr &image_msg);
  void order_callback(const osrf_gear::Order::ConstPtr &order_msg);
  bool isReady();
  map<std::string, std::list<std::string>> getOrder();
 private:
  ros::Subscriber logical_camera_1, logical_camera_2,orders_subscriber;
  partlist inventory, order;
  ros::NodeHandle nh;
  bool l1_flag = false, l2_flag = false;
  size_t count_logical_1, count_logical_1;

};
