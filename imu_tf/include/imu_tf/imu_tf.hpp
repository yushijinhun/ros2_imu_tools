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

#ifndef IMU_TF__IMU_TF_HPP_
#define IMU_TF__IMU_TF_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.h>

#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <string>

namespace imu_tf
{

class IMUTF : public rclcpp::Node
{
public:
  IMUTF();

  virtual ~IMUTF();

  using Imu = sensor_msgs::msg::Imu;
  using Quaternion = geometry_msgs::msg::Quaternion;

private:

  rclcpp::Subscription<Imu>::SharedPtr sub_;

  void transform_to_worldframe(
    geometry_msgs::msg::Quaternion const & orientation,
    std::string const & source_frame, std::string const & target_frame);

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

};


}  // namespace IMU_TF

#endif  // IMU_TF__IMU_TF_HPP_
