/**
 * @file Node.cpp
 * @author     Ravi Bhadeshiya
 * @version    2.0
 * @brief      testing
 *
 * @copyright  BSD 3-Clause License (c) 2018 Ravi Bhadeshiya
 **/
#include <osrf_gear/LogicalCameraImage.h>
#include "project_ariac/Sensor.hpp"

typedef osrf_gear::LogicalCameraImage::ConstPtr CameraMsg;
typedef Sensor<CameraMsg> Camera;

int main(int argc, char** argv) {
  ros::init(argc, argv, "project_ariac_node");

  ros::NodeHandle node;
  std::shared_ptr<Camera> camera;
  camera = std::make_shared<Camera>(node, "ariac/logical_camera_1");

  ros::spin();
  return 0;
}
