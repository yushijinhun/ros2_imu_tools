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

#ifndef IMU_FUSION_MADGWICK__IMU_FUSION_MADGWICK_HPP_
#define IMU_FUSION_MADGWICK__IMU_FUSION_MADGWICK_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "imu_fusion_madgwick/orientation.hpp"

namespace imu_fusion_madgwick
{

class IMUFusionMadgwick : public rclcpp::Node
{
public:
  IMUFusionMadgwick();

  virtual ~IMUFusionMadgwick();

private:
  Orientation3d d_orientation;

  using Imu = sensor_msgs::msg::Imu;
  using Quaternion = geometry_msgs::msg::Quaternion;
  Quaternion orientation;

  bool use_fixed_dt;
  double dt;
  float gyroMeasError;
  rclcpp::Time lastUpdateTime_;

  rclcpp::Subscription<Imu>::SharedPtr imuRawSub_;

  rclcpp::Publisher<Imu>::SharedPtr imuPub_;

  void reset();
  void reset(const Quaternion & quaternion);
};
}  // namespace imu_fusion_madgwick

#endif  // IMU_FUSION_MADGWICK__IMU_FUSION_MADGWICK_HPP_
