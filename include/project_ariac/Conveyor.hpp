/**
 * @file Conveyor.hpp
 * @author     Ravi Bhadeshiya
 * @version    2.0
 * @brief      Conveyor class
 *
 * @copyright  BSD 3-Clause License (c) 2018 Ravi Bhadeshiya
 *
 * Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may
be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once
// ROS
#include <geometry_msgs/Pose.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <ros/ros.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "project_ariac/Interface.hpp"
#include "project_ariac/Sensor.hpp"

namespace conveyor {
/**
 * @brief      map < part_type, list of poseStamped>
 */
typedef std::map<std::string, std::vector<geometry_msgs::Pose>> Database;
typedef osrf_gear::LogicalCameraImage::ConstPtr CameraMsg;
typedef std::shared_ptr<ros::NodeHandle> NodePtr;
} // namespace conveyor

class Conveyor : public Interface, public Sensor<conveyor::CameraMsg> {
public:
  Conveyor();
  explicit Conveyor(const ros::NodeHandle &nh);
  ~Conveyor();
  void callback(const conveyor::CameraMsg &msg);
  double getSpeed();

protected:
  double findAvgSpeed(const std::vector<double> &dist, const double &dt);
  void update(osrf_gear::LogicalCameraImage &msg);

private:
  ros::Publisher pub_part;
  ros::Time last_time_, current_time_;
  conveyor::Database inventory_;
  double SPEED_ = 1.0, dt_;

  // callback
  std::vector<double> distances_;
  int MAX_Element = 11; // 10hz
  osrf_gear::Model model_;
  geometry_msgs::Pose part_;
  conveyor::Database::iterator itr;
};
