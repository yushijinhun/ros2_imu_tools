#ifndef IMU_FILTER_MADGWICK__IMU_FILTER_MADGWICK_HPP_
#define IMU_FILTER_MADGWICK__IMU_FILTER_MADGWICK_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>

namespace imu_filter_madgwick
{

class IMUFilterMadgwickPublisher : public rclcpp::Node
{
public:
  IMUFilterMadgwickPublisher();

  virtual ~IMUFilterMadgwickPublisher();

private:


};

}  // namespace imu_filter_madgwick

#endif  // IMU_FILTER_MADGWICK__IMU_FILTER_MADGWICK_HPP_