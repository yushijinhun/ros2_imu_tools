#include "rviz_marker/rviz_marker.hpp"

namespace rviz_marker {

RvizMarkerPublisher::RvizMarkerPublisher()
  : rclcpp::Node{"rviz_marker_publisher"}
{

  markerPub_ = create_publisher<visualization_msgs::msg::Marker>("/visualization_msg/marker");

  imuSub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data",
    [ = ](sensor_msgs::msg::Imu::SharedPtr imuMsg) {
      // create package
    });

}

RvizMarkerPublisher::~RvizMarkerPublisher()
{}

} // namespace
