/**
 * @file Node.cpp
 * @author     Ravi Bhadeshiya
 * @version    2.0
 * @brief      testing
 *
 * @copyright  BSD 3-Clause License (c) 2018 Ravi Bhadeshiya
 **/
#include "project_ariac/UR10_Control.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_node");
  ros::NodeHandle node;

  geometry_msgs::Pose target_;
  target_.position.x = -0.5;
  target_.position.y = -0.735;
  target_.position.z = 0.724;




  ros::spin();
  return 0;
}
