#ifndef RVIZ_MARKER__RVIZ_MARKER_HPP_
#define RVIZ_MARKER__RVIZ_MARKER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker.hpp>
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

  visualization_msgs::msg::MarkerArray markerArrayMsg;

  geometry_msgs::msg::Point origin;
  geometry_msgs::msg::Point xTip;
  geometry_msgs::msg::Point yTip;
  geometry_msgs::msg::Point zTip;

  Marker xAxisMarker;
  Marker yAxisMarker;
  Marker zAxisMarker;

  Marker axisMarker(const geometry_msgs::msg::Point& tip, const double& r, const double& g, const double& b);


  rclcpp::Publisher<MarkerArray>::SharedPtr markerPub_;

};
}  // namespace rviz_marker

#endif  // RVIZ_MARKER__RVIZ_MARKER_HPP_
