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
: rclcpp::Node{"imu_fusion_madgwick"},
  lastUpdateTime_(now()),
  orientation_{Eigen::Quaterniond::Identity()}
{
  get_parameter_or_set("gyro_measuring_error", gyroMeasError, 3.14159265358979f * (5.0f / 180.0f));

  // gain is unused
  // float beta;
  // get_parameter_or_set("gyroMeasError", beta, sqrt(3.0f / 4.0f) * gyroMeasError);

  // static sampling periodin seconds
  get_parameter_or_set("use_fixed_dt", use_fixed_dt, false);
  get_parameter_or_set("fixed_dt", dt, 0.008);


  RCLCPP_INFO(get_logger(), "Initialise robot orientation");
  reset();

  imuPub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data");

  RCLCPP_INFO(get_logger(), "Subscribe for /imu/data_raw");
  imuRawSub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data_raw",
    [ = ](sensor_msgs::msg::Imu::SharedPtr imuMsg) {
      if (!use_fixed_dt) {
        // TODO(scheunemann) auto time = imuRawMsg->header.stamp;
        dt = (now() - lastUpdateTime_).seconds();
        lastUpdateTime_ = now();
      }

      //
      integrate(imuMsg->angular_velocity, dt, imuMsg->linear_acceleration, gyroMeasError);

      // set new orientation
      imuMsg->orientation = getQuaternion();

      // indicate that message
      // has valid orientation
      imuMsg->orientation_covariance.at(0) = 0;

      RCLCPP_DEBUG(get_logger(), "Received raw IMU message and publish IMU message");
      imuPub_->publish(imuMsg);
    });
}

IMUFusionMadgwick::~IMUFusionMadgwick()
{}

geometry_msgs::msg::Quaternion IMUFusionMadgwick::getQuaternion() const
{
  Quaternion quat;

  quat.x = orientation_.x();
  quat.y = orientation_.y();
  quat.z = orientation_.z();
  quat.w = orientation_.w();

  return quat;
}

void IMUFusionMadgwick::reset()
{
  orientation_ = Eigen::Quaterniond::Identity();
}

void IMUFusionMadgwick::reset(const Quaternion & quaternion)
{
  orientation_.w() = quaternion.w;
  orientation_.x() = quaternion.x;
  orientation_.y() = quaternion.y;
  orientation_.z() = quaternion.z;
}

void IMUFusionMadgwick::reset(const Eigen::Quaterniond & quat)
{
  orientation_ = quat;
}

void IMUFusionMadgwick::integrate(
  geometry_msgs::msg::Vector3 const & angular_velocity, double interval,
  geometry_msgs::msg::Vector3 const & linear_acceleration,
  double maxGyroError)
{
  auto referenceDirMes = Eigen::Vector3d{
    linear_acceleration.x, linear_acceleration.y, linear_acceleration.z
  };

  auto angularRate = Eigen::Vector3d{
    angular_velocity.x, angular_velocity.y, angular_velocity.z
  };

  integrate(angularRate, interval, referenceDirMes, maxGyroError);
}

void IMUFusionMadgwick::integrate(const Eigen::Vector3d & angularRate, double interval)
{
  auto theta = angularRate * interval / 2;
  auto thetaMag = theta.norm();
  if (thetaMag == 0) {
    return;
  }

  auto deltaQuat = Eigen::Quaterniond{};
  deltaQuat.w() = cos(thetaMag);
  deltaQuat.vec() = sin(thetaMag) / thetaMag * theta;

  // T_A2^W = T_A1^W * T_A2^A1
  orientation_ = orientation_ * deltaQuat;
}

void IMUFusionMadgwick::integrate(
  Eigen::Vector3d const & angularRate, double interval,
  Eigen::Vector3d const & referenceDirLocalMeas,
  double maxGyroError,
  Eigen::Vector3d const & referenceDirGlobal)
{
  integrate(angularRate, interval);

  double constexpr sqrt34 = std::sqrt(0.75);
  auto beta = sqrt34 * maxGyroError;

  // The objective function is the difference between
  // expected and measured reference dir in local frame
  // The quaternion describes the orientation of the local frame in the global frame,
  // so a vector multiplied with it is transformed from local to global

  Eigen::Vector3d objectiveFunction =
    orientation_.conjugate() * referenceDirGlobal - referenceDirLocalMeas;

  auto q1 = orientation_.w();
  auto q2 = orientation_.x();
  auto q3 = orientation_.y();
  auto q4 = orientation_.z();

  auto jacobian =
    (Eigen::Matrix<double, 3, 4>() <<
    2 * q4, -2 * q1, 2 * q2, -2 * q3,
    2 * q1, 2 * q4, 2 * q3, 2 * q2,
    -4 * q2, -4 * q3, 0.0, 0.0   ).
    finished();

  Eigen::Vector4d normalizedObjectiveFunctionGradient =
    jacobian.transpose() * objectiveFunction;

  if (normalizedObjectiveFunctionGradient.squaredNorm() > 1e-16) {
    normalizedObjectiveFunctionGradient.normalize();
  }

  orientation_.coeffs() -= beta * normalizedObjectiveFunctionGradient * interval;
  orientation_.normalize();
}

}  // namespace imu_fusion_madgwick
