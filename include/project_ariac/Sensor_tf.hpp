/**
 * @file Camera_tf.hpp
 * @author     Ravi Bhadeshiya
 * @version    0.1
 * @brief      Camera tf transformation
 *
 * @copyright  MIT License (c) 2017 Ravi Bhadeshiya
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#pragma once

#include <ros/ros.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
// #include <cwru_ariac/Part.h>
#include <string>

class Sensor_tf{
public:
  Sensor_tf();
  Sensor_tf(ros::NodeHandle nh_);
  Sensor_tf(ros::NodeHandle nh_, const std::string& topic);
  ~Sensor_tf();
  void logical_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
private:
  std::string topic = "/ariac/logical_camera_1";
  tf::TransformListener tf_listener;
  ros::NodeHandle nh;
  ros::Publisher parts_pose_publisher;
  ros::Subscriber logical_camera_subscriber;
};
