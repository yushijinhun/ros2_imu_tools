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

#include "imu_fusion_madgwick/imu_fusion_madgwick.hpp"

namespace imu_fusion_madgwick
{

IMUFusionMadgwick::IMUFusionMadgwick()
: rclcpp::Node{"imu_filter_madgwick_publisher"},
  lastUpdateTime_(now())
{
  get_parameter_or_set("gyroMeasError", gyroMeasError, 3.14159265358979f * (5.0f / 180.0f));

  // gain is unused
  // float beta;
  // get_parameter_or_set("gyroMeasError", beta, sqrt(3.0f / 4.0f) * gyroMeasError);

  // static sampling periodin seconds
  get_parameter_or_set("use_fixed_dt", use_fixed_dt, false);
  get_parameter_or_set("fixed_dt", dt, 0.008);


  RCLCPP_INFO(get_logger(), "Initialise robot orientation");
  d_orientation.reset();

  imuPub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data");

  RCLCPP_INFO(get_logger(), "Subscribe for /imu/data_raw");
  imuRawSub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data_raw",
    [ = ](sensor_msgs::msg::Imu::SharedPtr imuRawMsg) {
      if (!use_fixed_dt) {
        // TODO(scheunemann) auto time = imuRawMsg.get()->header.stamp;
        dt = (now() - lastUpdateTime_).seconds();
        lastUpdateTime_ = now();
      }

      Eigen::Matrix<double, 3, 1> const & referenceDirMes = Eigen::Matrix<double, 3, 1>{
        imuRawMsg.get()->linear_acceleration.x,
        imuRawMsg.get()->linear_acceleration.y,
        imuRawMsg.get()->linear_acceleration.z
      };

      Eigen::Matrix<double, 3, 1> const & angularRate = Eigen::Matrix<double, 3, 1>{
        imuRawMsg.get()->angular_velocity.x,
        imuRawMsg.get()->angular_velocity.y,
        imuRawMsg.get()->angular_velocity.z
      };

      d_orientation.integrate(angularRate, dt, referenceDirMes, gyroMeasError);

      imuRawMsg->orientation.w = d_orientation.getQuaternion().w();
      imuRawMsg->orientation.x = d_orientation.getQuaternion().x();
      imuRawMsg->orientation.y = d_orientation.getQuaternion().y();
      imuRawMsg->orientation.z = d_orientation.getQuaternion().z();

      imuRawMsg->orientation_covariance.at(0) = 0;

      RCLCPP_DEBUG(get_logger(), "Received raw IMU message and publish IMU message");
      imuPub_->publish(imuRawMsg.get());
    });
}

IMUFusionMadgwick::~IMUFusionMadgwick()
{}

void IMUFusionMadgwick::reset()
{
  Quaternion quaternion;

  // identity
  quaternion.x = 0;
  quaternion.y = 0;
  quaternion.z = 0;
  quaternion.w = 1;

  orientation = quaternion;
}

void IMUFusionMadgwick::reset(const Quaternion & quaternion)
{
  orientation = quaternion;
}

}  // namespace imu_fusion_madgwick
