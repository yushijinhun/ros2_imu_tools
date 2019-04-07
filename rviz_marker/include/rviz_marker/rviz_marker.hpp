#ifndef RVIZ_MARKER__RVIZ_MARKER_HPP_
#define RVIZ_MARKER__RVIZ_MARKER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
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

  using ImuData = sensor_msgs::msg::Imu;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  rclcpp::Subscription<ImuData>::SharedPtr imuSub_;

  MarkerArray markerArrayMsg;
  rclcpp::Publisher<MarkerArray>::SharedPtr markerPub_;

  Marker baseAxisMarker(std::string axis_name);

};
}  // namespace rviz_marker

#endif  // RVIZ_MARKER__RVIZ_MARKER_HPP_
