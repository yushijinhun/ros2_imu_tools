#include "imu_filter_madgwick/imu_filter_madgwick.hpp"

namespace imu_filter_madgwick {

IMUFilterMadgwickPublisher::IMUFilterMadgwickPublisher()
  : rclcpp::Node{"imu_filter_madgwick_publisher"}
{
  imuPub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data");

  imuRawSub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data_raw",
    [ = ](sensor_msgs::msg::Imu::SharedPtr imuRawMsg) {
      imuPub_->publish(imuRawMsg.get());
    });

  // something
}

IMUFilterMadgwickPublisher::~IMUFilterMadgwickPublisher()
{}


} // namespace
