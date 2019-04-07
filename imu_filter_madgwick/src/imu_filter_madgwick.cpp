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

#include "imu_filter_madgwick/imu_filter_madgwick.hpp"

namespace imu_filter_madgwick
{

IMUFilterMadgwickPublisher::IMUFilterMadgwickPublisher()
: rclcpp::Node{"imu_filter_madgwick_publisher"}
{
  imuPub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data");

  imuRawSub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data_raw",
    [ = ](sensor_msgs::msg::Imu::SharedPtr imuRawMsg) {
      imuPub_->publish(imuRawMsg.get());
    });
}

IMUFilterMadgwickPublisher::~IMUFilterMadgwickPublisher()
{}


}  // namespace imu_filter_madgwick
