#include "rviz_marker/rviz_marker.hpp"

namespace rviz_marker {

RvizMarkerPublisher::RvizMarkerPublisher()
  : rclcpp::Node{"rviz_marker_publisher"}
{

  markerPub_ = create_publisher<visualization_msgs::msg::Marker>("/visualization_msg/marker");

  imuSub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data",
    [ = ](sensor_msgs::msg::Imu::SharedPtr imuMsg) {

      visualization_msgs::msg::Marker::SharedPtr markerMsg;
      markerMsg->header.frame_id = "base_link";
      // markerMsg->header.stamp = ros_clock.now();
      markerMsg->ns = "imu_visualization";
      markerMsg->id = 0;
      markerMsg->type = visualization_msgs::msg::Marker::ARROW;
      markerMsg->action = visualization_msgs::msg::Marker::ADD;

      markerMsg->pose.position.x = 1;
      markerMsg->pose.position.y = 1;
      markerMsg->pose.position.z = 1;

      markerMsg->pose.orientation.w = imuMsg.get()->orientation.w;
      markerMsg->pose.orientation.x = imuMsg.get()->orientation.x;
      markerMsg->pose.orientation.y = imuMsg.get()->orientation.y;
      markerMsg->pose.orientation.z = imuMsg.get()->orientation.z;

      markerPub_->publish(markerMsg.get());

    });

}

RvizMarkerPublisher::~RvizMarkerPublisher()
{}

} // namespace
