#include "imu_filter_madgwick/imu_filter_madgwick.hpp"

namespace imu_filter_madgwick {

IMUFilterMadgwickPublisher::IMUFilterMadgwickPublisher()
  : rclcpp::Node{"imu_filter_madgwick_publisher"},
  lastUpdateTime_(now())
{
  imuPub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data");

  imuRawSub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data_raw",
    [ = ](sensor_msgs::msg::Imu::SharedPtr imuRawMsg) {

      float ax = static_cast<float>(imuRawMsg.get()->linear_acceleration.x);
      float ay = static_cast<float>(imuRawMsg.get()->linear_acceleration.y);
      float az = static_cast<float>(imuRawMsg.get()->linear_acceleration.z);

      float gx = static_cast<float>(imuRawMsg.get()->angular_velocity.x);
      float gy = static_cast<float>(imuRawMsg.get()->angular_velocity.y);
      float gz = static_cast<float>(imuRawMsg.get()->angular_velocity.z);

      // TODO: auto time = imuRawMsg.get()->header.stamp;
      float dt = static_cast<float>((now() - lastUpdateTime_).seconds());
      lastUpdateTime_ = now();

      filter.updateIMU(gx, gy, gz, ax, ay, az, dt);

      imuRawMsg->orientation.w = filter.q0;
      imuRawMsg->orientation.x = filter.q1;
      imuRawMsg->orientation.y = filter.q2;
      imuRawMsg->orientation.z = filter.q3;

      imuPub_->publish(imuRawMsg.get());
    });

  // something
}

IMUFilterMadgwickPublisher::~IMUFilterMadgwickPublisher()
{}

} // namespace
