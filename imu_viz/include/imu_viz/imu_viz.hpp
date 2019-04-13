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

#ifndef RVIZ_MARKER__RVIZ_MARKER_HPP_
#define RVIZ_MARKER__RVIZ_MARKER_HPP_

#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

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
