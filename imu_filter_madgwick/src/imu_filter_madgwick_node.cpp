#include "imu_filter_madgwick/imu_filter_madgwick.hpp"
#include <cstdio>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<imu_filter_madgwick::IMUFilterMadgwickPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  node = nullptr;

  return 0;
}
