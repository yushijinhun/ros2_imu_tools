#include "rviz_marker/rviz_marker.hpp"
#include <cstdio>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rviz_marker::RvizMarkerPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  node = nullptr;

  return 0;
}
