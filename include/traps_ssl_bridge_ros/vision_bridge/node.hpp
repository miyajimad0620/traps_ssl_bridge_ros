// Copyright 2024 TRAPS
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

#ifndef TRAPS_SSL_BRIDGE_ROS__VISION_BRIDGE__NODE_HPP_
#define TRAPS_SSL_BRIDGE_ROS__VISION_BRIDGE__NODE_HPP_

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "asio/io_service.hpp"
#include "asio/ip/address.hpp"
#include "asio/ip/udp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/node.hpp"
#include "traps_ssl_bridge_ros/visibility.hpp"
#include "traps_ssl_bridge_ros/vision_bridge/robot_sub_node.hpp"

namespace traps_ssl_bridge_ros::vision_bridge
{

class Node : public rclcpp::Node
{
public:
  static constexpr auto default_node_name() noexcept {return "vision_bridge";}

  TRAPS_SSL_BRIDGE_ROS_PUBLIC
  Node(
    const std::string & node_name, const std::string & node_namespace,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  TRAPS_SSL_BRIDGE_ROS_PUBLIC
  explicit inline Node(
    const std::string & node_name, const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node(node_name, "", node_options)
  {
  }

  TRAPS_SSL_BRIDGE_ROS_PUBLIC
  explicit inline Node(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node(this->default_node_name(), "", node_options)
  {
  }

private:
  void receive();

  std::vector<std::optional<RobotSubNode>> blue_robot_sub_nodes_, yellow_robot_sub_nodes_;
  rclcpp::TimerBase::SharedPtr receive_timer_;

  asio::io_service io_service_;
  asio::error_code ec_;
  asio::ip::udp::socket udp_socket_;
};

}  // namespace traps_ssl_bridge_ros::vision_bridge

#endif  // TRAPS_SSL_BRIDGE_ROS__VISION_BRIDGE__NODE_HPP_
