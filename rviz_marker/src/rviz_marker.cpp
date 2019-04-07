#include "rviz_marker/rviz_marker.hpp"

namespace rviz_marker {

RvizMarkerPublisher::RvizMarkerPublisher()
  : rclcpp::Node{"rviz_marker_publisher"}
{

  markerPub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_msg/marker_array");

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
  xAxisMarker = axisMarker(xTip, 1, 0, 0);
  yAxisMarker = axisMarker(yTip, 0, 1, 0);
  //zAxisMarker = axisMarker(zTip, 0, 0, 1);

  markerArrayMsg.markers.push_back(xAxisMarker);
  markerArrayMsg.markers.push_back(yAxisMarker);
  markerArrayMsg.markers.push_back(zAxisMarker);

  imuSub_ = create_subscription<ImuData>(
    "/imu/data",
    [ & ](ImuData::SharedPtr imuMsg) {

      auto timeNow = now();

      for (auto& marker : markerArrayMsg.markers) {
        marker.header.stamp = timeNow;
        marker.action = Marker::MODIFY;
        marker.pose.orientation = imuMsg.get()->orientation;
      }

      markerPub_->publish(markerArrayMsg);

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
