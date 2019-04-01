#ifndef RVIZ_MARKER__RVIZ_MARKER_HPP_
#define RVIZ_MARKER__RVIZ_MARKER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>

#include "rviz_marker/rviz_marker.hpp"

namespace rviz_marker
{

class RvizMarkerPublisher : public rclcpp::Node
{
public:
  RvizMarkerPublisher();

  virtual ~RvizMarkerPublisher();

private:

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub_;

};
}  // namespace rviz_marker

#endif  // RVIZ_MARKER__RVIZ_MARKER_HPP_
