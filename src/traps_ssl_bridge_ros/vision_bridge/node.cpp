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

#include "traps_ssl_bridge_ros/vision_bridge/node.hpp"

#include <deque>

#include "asio/ip/multicast.hpp"
#include "fmt/core.h"
#include "fmt/ranges.h"
#include "./ssl_vision_wrapper.pb.h"
#include "traps_ssl_bridge_ros/create_robot_names.hpp"
#include "traps_ssl_bridge_ros/create_robot_subnodes.hpp"
#include "traps_ssl_bridge_ros/dynamic_qos.hpp"

namespace traps_ssl_bridge_ros::vision_bridge
{

Node::Node(
  const std::string & node_name, const std::string & node_namespace,
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_namespace, node_options),
  blue_robot_sub_nodes_(create_robot_subnodes<std::optional<RobotSubNode>>(
      this, this->declare_parameter(
        "blue_robots", create_robot_names(
          this->declare_parameter("blue_robots.prefix", "blue"),
          this->declare_parameter("blue_robots.count", 11))))),
  yellow_robot_sub_nodes_(create_robot_subnodes<std::optional<RobotSubNode>>(
      this, this->declare_parameter(
        "yellow_robots", create_robot_names(
          this->declare_parameter("yellow_robots.prefix", "yellow"),
          this->declare_parameter("yellow_robots.count", 11))))),
  receive_timer_(this->create_wall_timer(std::chrono::seconds(0), [this] {this->receive();})),
  io_service_(),
  udp_socket_(
    io_service_,
    asio::ip::udp::endpoint(asio::ip::udp::v4(), this->declare_parameter("udp.port", 10020)))
{
  asio::error_code ec;
  const auto multicast_address = asio::ip::address::from_string(
    this->declare_parameter("udp.multicast_address", "224.5.23.2"), ec);
  if (ec) {
    const auto error_str = fmt::format("asio error: {}", ec.value());
    RCLCPP_ERROR(this->get_logger(), error_str.c_str());
  }
  if (!multicast_address.is_multicast()) {
    const auto error_str =
      fmt::format("\"{}\" is not multicat address", multicast_address.to_string());
    RCLCPP_ERROR(this->get_logger(), error_str.c_str());
  }
  udp_socket_.set_option(asio::ip::multicast::join_group(multicast_address), ec);
  if (ec) {
    const auto error_str = fmt::format("asio error: {}", ec.value());
    RCLCPP_ERROR(this->get_logger(), error_str.c_str());
  }
}

void Node::receive()
{
  // 受信時刻設定
  const auto now = this->now();

  // 受信確認
  const auto receive_length = udp_socket_.available();
  if (receive_length <= 0) {
    return;
  }

  // 受信処理
  std::string buf;
  buf.resize(receive_length);
  asio::error_code ec;
  udp_socket_.receive(asio::buffer(buf), 0, ec);
  if (ec) {
    RCLCPP_ERROR(this->get_logger(), fmt::format("receiving error: {}", ec.message()).c_str());
    return;
  }

  // protobufへ変換
  SSL_WrapperPacket packet;
  if (!packet.MergeFromString(buf)) {
    RCLCPP_ERROR(this->get_logger(), "failed to purse buffer");
    return;
  }

  // detentionのpublish
  if (packet.has_detection()) {
    const auto & detection = packet.detection();

    // キャプチャ遅延処理
    const std::uint64_t delay_ns = (detection.t_sent() - detection.t_capture()) * 1e9;
    const auto stamp = now - std::chrono::nanoseconds(delay_ns);

    // publish
    const auto publish_robots = [&stamp](auto & subnodes, const auto & robots) {
        for (const auto & robot : robots) {
          if (robot.robot_id() >= subnodes.size()) {
            continue;
          }
          auto & subnode = subnodes[robot.robot_id()];
          if (!subnode) {
            continue;
          }

          subnode->bridge_pose(stamp, robot);
        }
      };
    publish_robots(yellow_robot_sub_nodes_, detection.robots_yellow());
    publish_robots(blue_robot_sub_nodes_, detection.robots_blue());
  }
}

}  // namespace traps_ssl_bridge_ros::vision_bridge

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(traps_ssl_bridge_ros::vision_bridge::Node)
