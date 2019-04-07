// Copyright 2019 Bold Hearts
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef IMU_FILTER_MADGWICK__IMU_FILTER_MADGWICK_HPP_
#define IMU_FILTER_MADGWICK__IMU_FILTER_MADGWICK_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "imu_filter_madgwick/orientation.hpp"

namespace imu_filter_madgwick
{

class IMUFilterMadgwickPublisher : public rclcpp::Node
{
public:
  IMUFilterMadgwickPublisher();

  virtual ~IMUFilterMadgwickPublisher();

private:
  Orientation3d d_orientation;

  rclcpp::Time lastUpdateTime_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuRawSub_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub_;
};
}  // namespace imu_filter_madgwick

#endif  // IMU_FILTER_MADGWICK__IMU_FILTER_MADGWICK_HPP_
