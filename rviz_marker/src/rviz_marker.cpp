#include "rviz_marker/rviz_marker.hpp"

namespace rviz_marker {

RvizMarkerPublisher::RvizMarkerPublisher()
  : rclcpp::Node{"rviz_marker_publisher"}
{

  markerPub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_msg/marker_array");

  //
  RCLCPP_INFO(get_logger(), "Setup axes marker");

  std::vector<std::string> axes = {"X", "Y", "Z"};
  for(auto &name : axes) {
    Marker marker = baseAxisMarker(name);
    markerArrayMsg.markers.push_back(marker);
  }

  imuSub_ = create_subscription<ImuData>(
    "/imu/data",
    [ & ](ImuData::SharedPtr imuMsg) {

      auto timeNow = now();

      for (auto& marker : markerArrayMsg.markers) {
        marker.header.stamp = timeNow;
        marker.action = Marker::MODIFY;
        marker.pose.orientation = imuMsg.get()->orientation;
      }

      RCLCPP_DEBUG(get_logger(), "Publish new message of MarkerArray");
      markerPub_->publish(markerArrayMsg);

    });

}

RvizMarkerPublisher::~RvizMarkerPublisher() {

  RCLCPP_INFO(get_logger(), "Delete axes marker");

  for (auto& marker : markerArrayMsg.markers) {
    marker.action   = Marker::DELETE;
    //TODO set duration
  }

  markerPub_->publish(markerArrayMsg);

}

RvizMarkerPublisher::Marker RvizMarkerPublisher::baseAxisMarker(std::string axis_name) {

  geometry_msgs::msg::Point origin;
  // origin.x = 0;
  // origin.y = 0;
  // origin.z = 0;

  visualization_msgs::msg::Marker axisMarker;

  axisMarker.header.frame_id = "base_link";
  axisMarker.header.stamp = now();
  axisMarker.id = 0;
  axisMarker.ns = "imu_visualization/" + axis_name;
  axisMarker.type = Marker::ARROW;
  axisMarker.action = Marker::ADD;

  axisMarker.scale.x = 0.1; // shaft diameter
  axisMarker.scale.y = 0.2; // head diameter
  // axisMarker.scale.z = 0.1; // head length

  // set start/end point to origin
  axisMarker.points.push_back(origin);
  axisMarker.points.push_back(origin);

  axisMarker.color.a = 1;

  // change color: XYZ -> RGB
  // change endpoint
  if (axis_name == "X") {
    axisMarker.color.r = 1;
    axisMarker.points.at(1).x = 1;
  } else if (axis_name == "Y") {
    axisMarker.color.g = 1;
    axisMarker.points.at(1).y = 1;
  } else if (axis_name == "Z") {
    axisMarker.color.b = 1;
    axisMarker.points.at(1).z = 1;
  }

  return axisMarker;
}

} // namespace
