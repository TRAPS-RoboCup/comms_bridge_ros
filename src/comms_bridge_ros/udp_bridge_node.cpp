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

#include <string>
#include <vector>

#include "asio/io_context.hpp"
#include "asio/ip/multicast.hpp"
#include "asio/ip/udp.hpp"
#include "comms_bridge_ros/msg/byte_array.hpp"
#include "comms_bridge_ros/visibility.hpp"
#include "rclcpp/node.hpp"

namespace comms_bridge_ros
{

class UdpBridgeNode : public rclcpp::Node
{
  using ByteArrayMsg = comms_bridge_ros::msg::ByteArray;

public:
  static constexpr auto kDefaultNodeName = "traps_ai_cmd_bridge";

  COMMS_BRIDGE_ROS_PUBLIC
  UdpBridgeNode(
    const std::string & node_name, const std::string & node_namespace,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node(node_name, node_namespace, node_options),
    io_context_(),
    buffer_size_(this->declare_parameter("buffer_size", 8192)),
    buffer_(buffer_size_)
  {
    // Declare parameters
    const auto addresses = this->declare_parameter("addresses", std::vector<std::string>{});
    const auto qos = rclcpp::QoS(this->declare_parameter("history_depth", 1)).best_effort();

    // set bridger capacity
    sockets_.reserve(addresses.size());

    for (const auto & address : addresses) {
      // addressをip_address:port:aliasの形式で分解
      const auto colon1 = address.find(':');
      if (colon1 == std::string::npos) {
        RCLCPP_ERROR(this->get_logger(), "Invalid address: %s", address.c_str());
        continue;
      }
      std::string ip_address_str = address.substr(0, colon1);
      std::string port_str = address.substr(colon1 + 1);

      // Create socket
      try {
        auto port = std::stoi(port_str);
        sockets_.emplace_back(io_context_, asio::ip::udp::endpoint(asio::ip::udp::v4(), port));
        auto & socket = sockets_.back();
        auto ip_address =
          ip_address_str == "any" ? asio::ip::address() : asio::ip::make_address(ip_address_str);

        // join multicast group
        if (ip_address.is_multicast()) {
          socket.set_option(asio::ip::udp::socket::reuse_address(true));
          socket.set_option(asio::ip::multicast::join_group(ip_address));
        }

        // set callback
        if (ip_address.is_multicast() || ip_address.is_unspecified()) {
          // Publicher
          auto publisher = this->create_publisher<ByteArrayMsg>("udp" + port_str, 10);

          socket.async_receive(
            asio::buffer(buffer_),
            [&socket, this, publisher](const asio::error_code & ec, std::size_t receive_length) {
              this->bridge_and_reset_call_back(socket, ec, receive_length, publisher);
            });
        } else {
          // Publicher
          auto ip_address_topic_str = ip_address_str;
          for (auto & c : ip_address_topic_str) {
            if (c < '0' || '9' < c) {
              c = '_';
            }
          }
          auto publisher = this->create_publisher<ByteArrayMsg>(
            "udp" + port_str + "_" + ip_address_topic_str, qos);

          auto endpoint = asio::ip::udp::endpoint(ip_address, port);
          socket.async_receive_from(
            asio::buffer(buffer_), endpoint,
            [&socket, this, publisher, endpoint](
              const asio::error_code & ec, std::size_t receive_length) {
              this->bridge_and_reset_call_back(socket, ec, receive_length, publisher, endpoint);
            });
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set socket: %s", e.what());
        continue;
      }
    }

    // Start io_context_thread
    io_context_thread_ = std::thread([this] {io_context_.run();});
  }

  COMMS_BRIDGE_ROS_PUBLIC
  explicit inline UdpBridgeNode(
    const std::string & node_name, const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : UdpBridgeNode(node_name, "", node_options)
  {
  }

  COMMS_BRIDGE_ROS_PUBLIC
  explicit inline UdpBridgeNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : UdpBridgeNode(kDefaultNodeName, "", node_options)
  {
  }

  COMMS_BRIDGE_ROS_PUBLIC
  ~UdpBridgeNode()
  {
    io_context_.stop();
    if (io_context_thread_.joinable()) {
      io_context_thread_.join();
    }
  }

private:
  //
  inline void bridge_and_reset_call_back(
    asio::ip::udp::socket & socket, const asio::error_code & ec, std::size_t receive_length,
    rclcpp::Publisher<ByteArrayMsg>::SharedPtr publisher)
  {
    this->bridge(ec, receive_length, publisher);

    try {
      socket.async_receive(
        asio::buffer(buffer_),
        [&socket, this, publisher](const asio::error_code & ec, std::size_t receive_length) {
          this->bridge_and_reset_call_back(socket, ec, receive_length, publisher);
        });
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "Failed to resister_receive_handle: %s", e.what());
      throw;
    }
  }

  inline void bridge_and_reset_call_back(
    asio::ip::udp::socket & socket, const asio::error_code & ec, std::size_t receive_length,
    rclcpp::Publisher<ByteArrayMsg>::SharedPtr publisher, asio::ip::udp::endpoint endpoint)
  {
    this->bridge(ec, receive_length, publisher);

    try {
      socket.async_receive_from(
        asio::buffer(buffer_), endpoint,
        [&socket, this, publisher, endpoint](
          const asio::error_code & ec, std::size_t receive_length) {
          this->bridge_and_reset_call_back(socket, ec, receive_length, publisher, endpoint);
        });
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "Failed to resister_receive_handle: %s", e.what());
      throw;
    }
  }

  inline void bridge(
    const asio::error_code & ec, std::size_t receive_length,
    rclcpp::Publisher<ByteArrayMsg>::SharedPtr publisher)
  {
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Failed to async_receive: %s", ec.message().c_str());
      return;
    }

    thread_local auto byte_array_msg =
      std::make_unique<ByteArrayMsg>(rosidl_runtime_cpp::MessageInitialization::SKIP);
    buffer_.resize(receive_length);
    byte_array_msg->data = std::move(buffer_);
    publisher->publish(std::move(byte_array_msg));

    byte_array_msg =
      std::make_unique<ByteArrayMsg>(rosidl_runtime_cpp::MessageInitialization::SKIP);
    buffer_ = std::vector<std::uint8_t>(buffer_size_);
  }

  // ネットワーク
  asio::io_context io_context_;
  std::thread io_context_thread_;

  // バッファ
  std::size_t buffer_size_;
  std::vector<std::uint8_t> buffer_;

  std::vector<asio::ip::udp::socket> sockets_;
};

}  // namespace comms_bridge_ros

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(comms_bridge_ros::UdpBridgeNode)
