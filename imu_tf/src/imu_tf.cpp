// Copyright 2020 Bold Hearts
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

#include "imu_tf/imu_tf.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.h>

#include <tf2_ros/transform_broadcaster.h>

#include <rclcpp/logging.hpp>

#include <string>

namespace imu_tf
{

IMUTF::IMUTF()
: rclcpp::Node{"imu_tf"},
  tf_broadcaster_(this)
{
  auto source_frame = declare_parameter<std::string>("source_frame", "base_link");
  auto target_frame = declare_parameter<std::string>("target_frame", "base_link_oriented");
  RCLCPP_INFO(get_logger(), "TF source frame: " + source_frame);
  RCLCPP_INFO(get_logger(), "TF target frame: " + target_frame);

  RCLCPP_INFO(get_logger(), "Subscribe to /imu/data");

  sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data",
    rclcpp::SensorDataQoS(),
    [ = ](sensor_msgs::msg::Imu::SharedPtr imuMsg) {
      RCLCPP_DEBUG(get_logger(), "Received IMU message and broadcast new TF");

      transform_to_worldframe(imuMsg->orientation, source_frame, target_frame);
    });
}

IMUTF::~IMUTF() {}

void IMUTF::transform_to_worldframe(
  geometry_msgs::msg::Quaternion const & orientation,
  std::string const & source_frame,
  std::string const & target_frame)
{
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = now();
  transformStamped.header.frame_id = target_frame;
  transformStamped.child_frame_id = source_frame;

  transformStamped.transform.rotation = orientation;

  tf_broadcaster_.sendTransform(transformStamped);

  RCLCPP_DEBUG(
    get_logger(),
    "Transform target frame (" + target_frame + ") to source frame (" + source_frame + ")"
  );
}

}  // namespace imu_tf
