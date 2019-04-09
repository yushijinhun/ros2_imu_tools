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

#include <Eigen/Geometry>

#include "imu_fusion_madgwick/orientation.hpp"

namespace imu_fusion_madgwick
{

class IMUFusionMadgwick : public rclcpp::Node
{
public:
  IMUFusionMadgwick();

  virtual ~IMUFusionMadgwick();

private:
  using Imu = sensor_msgs::msg::Imu;
  using Quaternion = geometry_msgs::msg::Quaternion;

  bool use_fixed_dt;
  double dt;
  float gyroMeasError;
  rclcpp::Time lastUpdateTime_;

  rclcpp::Subscription<Imu>::SharedPtr imuRawSub_;
  rclcpp::Publisher<Imu>::SharedPtr imuPub_;

  Eigen::Quaterniond d_quaternion;

  Quaternion getQuaternion() const;

  void reset();
  void reset(const Quaternion & quat);
  void reset(Eigen::Quaternion<double> quat);

  void integrate(
    geometry_msgs::msg::Vector3 const & angular_velocity, double interval,
    geometry_msgs::msg::Vector3 const & linear_acceleration,
    double maxGyroError);

  /** Integrate measurement of angular rate
   *
   * @param angularRate Measured angular rate around axes, in rad/sec, e.g. from gyroscope
   * @param interval Time interval to integrate over, in secons
   */
  void integrate(Eigen::Matrix<double, 3, 1> const & angularRate, double interval);

  /** Integrate measurement of angular rate, corrected given a reference measurement
   *
   * It uses the technique from paper:
   * "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
   * Sebastian O.H. Madgwick, April 30, 2010
   *
   * @param angularRate Measured angular rate around axes, in rad/sec, e.g. from gyroscope
   * @param interval Time interval to integrate over, in seconds
   * @param referenceDirMeas Measurement of the reference directory in local reference frame
   * @param maxGyroError Maximum gyroscope measurement error, in rad/sec. Used to determine correction weight
   * @param referenceDir Reference direction that is measured in global reference frame. By default direction of gravity
   */
  void integrate(
    Eigen::Matrix<double, 3, 1> const & angularRate, double interval,
    Eigen::Matrix<double, 3, 1> const & referenceDirMeas,
    double maxGyroError,
    Eigen::Matrix<double, 3, 1> const & referenceDir = Eigen::Matrix<double, 3, 1>{0, 0, 1});
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace imu_fusion_madgwick

#endif  // IMU_FUSION_MADGWICK__IMU_FUSION_MADGWICK_HPP_
