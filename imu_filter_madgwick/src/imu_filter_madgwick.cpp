#include "imu_filter_madgwick/imu_filter_madgwick.hpp"

namespace imu_filter_madgwick {

IMUFilterMadgwickPublisher::IMUFilterMadgwickPublisher()
  : rclcpp::Node{"imu_filter_madgwick_publisher"}
{
  imuPub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data");

  imuRawSub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data_raw",
    [ = ](sensor_msgs::msg::Imu::SharedPtr imuRawMsg) {
      double ax = imuRawMsg.get()->linear_acceleration.x;
      double ay = imuRawMsg.get()->linear_acceleration.y;
      double az = imuRawMsg.get()->linear_acceleration.z;

      double gx = imuRawMsg.get()->angular_velocity.x;
      double gy = imuRawMsg.get()->angular_velocity.y;
      double gz = imuRawMsg.get()->angular_velocity.z;

      filter.updateIMU(gx, gy, gz, ax, ay, az);

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
