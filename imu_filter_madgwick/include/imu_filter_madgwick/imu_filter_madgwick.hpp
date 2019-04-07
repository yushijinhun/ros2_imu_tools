#ifndef IMU_FILTER_MADGWICK__IMU_FILTER_MADGWICK_HPP_
#define IMU_FILTER_MADGWICK__IMU_FILTER_MADGWICK_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>

#include "imu_filter_madgwick/orientation.hpp"

namespace imu_filter_madgwick
{

class IMUFilterMadgwickPublisher : public rclcpp::Node
{
public:
  IMUFilterMadgwickPublisher();

  virtual ~IMUFilterMadgwickPublisher();

private:

  Orientation3d d_orientation;

  rclcpp::Time lastUpdateTime_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuRawSub_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub_;

};
}  // namespace imu_filter_madgwick

#endif  // IMU_FILTER_MADGWICK__IMU_FILTER_MADGWICK_HPP_
