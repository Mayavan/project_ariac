/**
 * @file Sensor.hpp
 * @author     Ravi Bhadeshiya
 * @version    2.0
 * @brief      Sensor Abstraction
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
#include <memory>
#include <ros/ros.h>
#include <string>

typedef std::shared_ptr<ros::NodeHandle> NodePtr;

template <class T> class Sensor {
public:
  Sensor();
  explicit Sensor(const ros::NodeHandle &nh, const std::string &topic);
  virtual ~Sensor();
  virtual T getMessage();
  virtual void callback(const T &msg);
  bool isPopulated();
  size_t getCounter();
  void setCounter();
  std::string getSensorFrame();

protected:
  ros::Subscriber sensor_subscriber_;
  NodePtr nh_;
  T msg_ = NULL;
  std::string topic_;
  size_t counter;
};

template <class T> Sensor<T>::Sensor() : counter(0) {}

template <class T>
Sensor<T>::Sensor(const ros::NodeHandle &nh, const std::string &topic)
    : counter(0), topic_(topic) {
  nh_ = std::make_shared<ros::NodeHandle>(nh);
  sensor_subscriber_ = nh_->subscribe(topic, 10, &Sensor<T>::callback, this);
  ROS_DEBUG_STREAM("Sensor subscribed to " << topic << std::endl);
}

template <class T> Sensor<T>::~Sensor() { counter = 0; }

template <class T> T Sensor<T>::getMessage() {
  ros::spinOnce();
  ros::Duration(0.1).sleep();
  return msg_;
}

template <class T> void Sensor<T>::callback(const T &msg) {
  msg_ = msg;
  counter += 1;
}

template <class T> bool Sensor<T>::isPopulated() { return (counter > 0); }

template <class T> size_t Sensor<T>::getCounter() { return counter; }

template <class T> void Sensor<T>::setCounter() { counter = 1; }

template <class T> std::string Sensor<T>::getSensorFrame() {
  return topic_.substr(topic_.find_last_of('/') + 1) + "_frame";
}
