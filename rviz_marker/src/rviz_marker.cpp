#include "rviz_marker/rviz_marker.hpp"

namespace rviz_marker {

RvizMarkerPublisher::RvizMarkerPublisher()
  : rclcpp::Node{"rviz_marker_publisher"}
{

  markerPub_ = create_publisher<visualization_msgs::msg::Marker>("/visualization_msg/marker");

  imuSub_ = create_subscription<ImuData>(
    "/imu/data",
    [ & ](ImuData::SharedPtr imuMsg) {

      markerMsg.header.frame_id = "base_link";
      // markerMsg->header.stamp = ros_clock.now();
      markerMsg.ns = "imu_visualization";
      markerMsg.id = 0;
      markerMsg.type   = Marker::ARROW;
      markerMsg.action = Marker::ADD;

      markerMsg.pose.position.x = 0;
      markerMsg.pose.position.y = 0;
      markerMsg.pose.position.z = 0;

      markerMsg.pose.orientation.w = imuMsg.get()->orientation.w;
      markerMsg.pose.orientation.x = imuMsg.get()->orientation.x;
      markerMsg.pose.orientation.y = imuMsg.get()->orientation.y;
      markerMsg.pose.orientation.z = imuMsg.get()->orientation.z;

      markerMsg.scale.x = 1;
      markerMsg.scale.y = 0.1;
      markerMsg.scale.z = 0.1;
      markerMsg.color.a = 1.0;
      markerMsg.color.r = 0.9;
      markerMsg.color.g = 0.0;
      markerMsg.color.b = 0.1;

      markerPub_->publish(markerMsg);

    });

}

RvizMarkerPublisher::~RvizMarkerPublisher() {}

} // namespace
