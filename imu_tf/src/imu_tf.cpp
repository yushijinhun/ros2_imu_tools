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

#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <rclcpp/logging.hpp>

#include <string>

namespace imu_tf
{

IMUTF::IMUTF()
: rclcpp::Node{"imu_tf"},
  tf_listener_(tf_buffer_),
  tf_broadcaster_(this)
{
  auto source_frame = declare_parameter<std::string>("source_frame", "torso");
  auto target_frame = declare_parameter<std::string>("target_frame", "base_link");
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
  tf2::TimePoint timePoint;    // getNow, maybe use imuMsg->header.stamp?

  try {
    transformStamped = tf_buffer_.lookupTransform(target_frame, source_frame, timePoint);
    transformStamped.transform.rotation = orientation;
    tf_broadcaster_.sendTransform(transformStamped);
    RCLCPP_DEBUG(
      get_logger(),
      "Transform target frame (" + target_frame + ") to source frame (" + source_frame + ")");
  } catch (tf2::TransformException ex) {
    RCLCPP_WARN(get_logger(), ex.what());
  }
}

}  // namespace imu_tf
