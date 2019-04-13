// Copyright 2019 Bold Hearts
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <vector>

#include "imu_viz/imu_viz.hpp"

namespace imu_viz
{

ImuVizPublisher::ImuVizPublisher()
: rclcpp::Node{"imu_viz_publisher"}
{
  markerPub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/imu/viz");

  //
  RCLCPP_INFO(get_logger(), "Setup axes marker");

  std::vector<std::string> axes = {"X", "Y", "Z"};
  for (auto & name : axes) {
    Marker marker = baseAxisMarker(name);
    markerArrayMsg.markers.push_back(marker);
  }

  imuSub_ = create_subscription<ImuData>(
    "/imu/data",
    [&](ImuData::SharedPtr imuMsg) {
      auto timeNow = now();

      for (auto & marker : markerArrayMsg.markers) {
        marker.header.stamp = timeNow;
        marker.action = Marker::MODIFY;
        marker.pose.orientation = imuMsg.get()->orientation;
      }

      RCLCPP_DEBUG(get_logger(), "Publish new message of MarkerArray");
      markerPub_->publish(markerArrayMsg);
    });
}

ImuVizPublisher::~ImuVizPublisher()
{
  RCLCPP_INFO(get_logger(), "Delete axes marker");

  for (auto & marker : markerArrayMsg.markers) {
    marker.action = Marker::DELETE;
    // TODO(marcus) set duration
  }

  markerPub_->publish(markerArrayMsg);
}

ImuVizPublisher::Marker ImuVizPublisher::baseAxisMarker(std::string axis_name)
{
  geometry_msgs::msg::Point origin;
  // origin.x = 0;
  // origin.y = 0;
  // origin.z = 0;

  visualization_msgs::msg::Marker axisMarker;

  axisMarker.header.frame_id = "base_link";
  axisMarker.header.stamp = now();
  axisMarker.id = 0;
  axisMarker.ns = "imu_viz/" + axis_name;
  axisMarker.type = Marker::ARROW;
  axisMarker.action = Marker::ADD;

  axisMarker.scale.x = 0.1;  // shaft diameter
  axisMarker.scale.y = 0.2;  // head diameter
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

}  // namespace imu_viz
