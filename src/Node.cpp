/**
 * @file Node.cpp
 * @author     Ravi Bhadeshiya
 * @version    2.0
 * @brief      testing
 *
 * @copyright  BSD 3-Clause License (c) 2018 Ravi Bhadeshiya
 **/
#include "project_ariac/Conveyor.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_node");
  ros::NodeHandle node;
  ROS_INFO_STREAM("Init");
  Conveyor con(node);
  ROS_INFO_STREAM("Finish");
  ros::spin();
  return 0;
}

