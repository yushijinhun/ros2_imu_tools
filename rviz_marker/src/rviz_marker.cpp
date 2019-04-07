#include "rviz_marker/rviz_marker.hpp"

namespace rviz_marker {

RvizMarkerPublisher::RvizMarkerPublisher()
  : rclcpp::Node{"rviz_marker_publisher"}
{

  markerPub_ = create_publisher<visualization_msgs::msg::Marker>("/visualization_msg/marker");

  // start point for Axes
  origin.x = 0;
  origin.y = 0;
  origin.z = 0;

  // end points for Axes
  xTip.x = 1;
  xTip.y = 0;
  xTip.z = 0;

  yTip.x = 0;
  yTip.y = 1;
  yTip.z = 0;

  zTip.x = 0;
  zTip.y = 0;
  zTip.z = 1;

  // create z-Axis
  zAxisMarker = axisMarker(zTip, 0, 0, 1);

  imuSub_ = create_subscription<ImuData>(
    "/imu/data",
    [ & ](ImuData::SharedPtr imuMsg) {

      zAxisMarker.header.stamp = now();
      zAxisMarker.action = Marker::MODIFY;

      zAxisMarker.pose.orientation.w = imuMsg.get()->orientation.w;
      zAxisMarker.pose.orientation.x = imuMsg.get()->orientation.x;
      zAxisMarker.pose.orientation.y = imuMsg.get()->orientation.y;
      zAxisMarker.pose.orientation.z = imuMsg.get()->orientation.z;

      markerPub_->publish(zAxisMarker);

    });

}

RvizMarkerPublisher::~RvizMarkerPublisher() {}

visualization_msgs::msg::Marker RvizMarkerPublisher::axisMarker(const geometry_msgs::msg::Point& tip, const double& r, const double& g, const double& b) {

  visualization_msgs::msg::Marker axisMarker;

  axisMarker.header.frame_id = "base_link";
  axisMarker.header.stamp = now();
  axisMarker.id = 0;
  axisMarker.ns = "imu_visualization";
  axisMarker.type = Marker::ARROW;
  axisMarker.action = Marker::ADD;

  axisMarker.points.push_back(origin);
  axisMarker.points.push_back(tip);

  axisMarker.scale.x = 0.1; // shaft diameter
  axisMarker.scale.y = 0.2; // head diameter
  // axisMarker.scale.z = 0.1; // head length

  // XYZ -> RGB
  axisMarker.color.r = r;
  axisMarker.color.g = g;
  axisMarker.color.b = b;
  axisMarker.color.a = 1;

  return axisMarker;
}

} // namespace
