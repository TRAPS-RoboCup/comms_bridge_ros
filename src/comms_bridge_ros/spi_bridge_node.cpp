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

extern "C" {
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
}

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "comms_bridge_ros/msg/multi_byte_array.hpp"
#include "comms_bridge_ros/visibility.hpp"
#include "rclcpp/node.hpp"

namespace comms_bridge_ros
{

class SpiBridgeNode : public rclcpp::Node
{
  using ByteArrayMsg = comms_bridge_ros::msg::ByteArray;
  using MultiByteArrayMsg = comms_bridge_ros::msg::MultiByteArray;

public:
  static constexpr auto kDefaultNodeName = "spi_bridge";

  COMMS_BRIDGE_ROS_PUBLIC
  SpiBridgeNode(
    const std::string & node_name, const std::string & node_namespace,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node(node_name, node_namespace, node_options),
    ioc_transfer_base_({}),
    ioc_transfers_(),
    spi_miso_msg_(std::make_unique<MultiByteArrayMsg>())
  {
    // SPI device
    const auto device_raw = this->declare_parameter("device", "/dev/spidev0.0");
    const auto device_normalized = std::filesystem::path(device_raw).lexically_normal();
    const auto device = std::filesystem::absolute(device_normalized).string();

    fd_ = open(device.c_str(), O_RDWR);
    if (fd_ < 0) {
      const auto error_str = "Failed to open spi device(" + device + "): " + std::strerror(errno);
      RCLCPP_FATAL(this->get_logger(), "%s", error_str.c_str());
      throw std::runtime_error(error_str);
    }

    // SPI configuration
    ioc_transfer_base_.bits_per_word = this->declare_parameter("bits_per_word", 8);
    ioc_transfer_base_.speed_hz = this->declare_parameter("speed_hz", 1'000'000);
    ioc_transfer_base_.delay_usecs = this->declare_parameter("delay_usecs", 0);
    ioc_transfer_base_.cs_change = this->declare_parameter("cs_change", 0);
    ioc_transfer_base_.pad = this->declare_parameter("pad", 0);
    ioc_transfer_base_.rx_nbits = this->declare_parameter("rx_nbits", 0);
    ioc_transfer_base_.tx_nbits = this->declare_parameter("tx_nbits", 0);
    ioc_transfer_base_.word_delay_usecs = this->declare_parameter("word_delay_usecs", 0);

    std::uint32_t mode = this->declare_parameter("mode", 0);
    std::uint8_t lsb_first = this->declare_parameter("lsb_first", 0);
    ioctl(fd_, SPI_IOC_WR_MODE32, &mode);
    ioctl(fd_, SPI_IOC_WR_LSB_FIRST, &lsb_first);
    ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &ioc_transfer_base_.speed_hz);
    ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &ioc_transfer_base_.bits_per_word);

    // ROS
    auto device_replaced = device;
    for (auto & c : device_replaced) {
      if ((c < '0' || '9' < c) && (c < 'a' || 'z' < c) && (c < 'A' || 'Z' < c)) {
        c = '_';
      }
    }

    spi_miso_publisher_ = this->create_publisher<MultiByteArrayMsg>(
      "spi" + device_replaced + "_miso", rclcpp::QoS(1).best_effort());
    spi_mosi_subscriber_ = this->create_subscription<MultiByteArrayMsg>(
      "spi" + device_replaced + "_mosi", rclcpp::QoS(1).best_effort(),
      [this](MultiByteArrayMsg::UniquePtr msg) {this->bridge(std::move(msg));});
  }

  COMMS_BRIDGE_ROS_PUBLIC
  explicit inline SpiBridgeNode(
    const std::string & node_name, const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : SpiBridgeNode(node_name, "", node_options)
  {
  }

  COMMS_BRIDGE_ROS_PUBLIC
  explicit inline SpiBridgeNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : SpiBridgeNode(kDefaultNodeName, "", node_options)
  {
  }

  COMMS_BRIDGE_ROS_PUBLIC
  ~SpiBridgeNode()
  {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

private:
  /// @brief Bridge SPI messages
  /// @param spi_mosi_msg
  void bridge(MultiByteArrayMsg::UniquePtr spi_mosi_msg)
  {
    // check empty
    if (spi_mosi_msg->byte_arrays.empty()) {
      spi_miso_publisher_->publish(std::move(spi_mosi_msg));
      return;
    }

    // make rx_buffer
    spi_miso_msg_->byte_arrays.resize(spi_mosi_msg->byte_arrays.size());
    ioc_transfers_.resize(spi_mosi_msg->byte_arrays.size(), ioc_transfer_base_);

    // set ioc_transfers
    auto mosi = spi_mosi_msg->byte_arrays.begin();
    auto miso = spi_miso_msg_->byte_arrays.begin();
    auto ioc_transfer = ioc_transfers_.begin();
    for (; mosi != spi_mosi_msg->byte_arrays.end(); ++mosi, ++miso, ++ioc_transfer) {
      miso->data.resize(mosi->data.size());
      const auto tx_buf_ptr = mosi->data.data();
      std::memcpy(&ioc_transfer->tx_buf, &tx_buf_ptr, sizeof(ioc_transfer->tx_buf));
      const auto rx_buf_ptr = miso->data.data();
      std::memcpy(&ioc_transfer->rx_buf, &rx_buf_ptr, sizeof(ioc_transfer->rx_buf));
      ioc_transfer->len = mosi->data.size();
    }

    // write
    ioctl(fd_, SPI_IOC_MESSAGE(spi_mosi_msg->byte_arrays.size()), ioc_transfers_.data());

    // publish
    spi_miso_publisher_->publish(std::move(spi_miso_msg_));

    // reset rx_buffer
    spi_miso_msg_ = std::move(spi_mosi_msg);
  }

  // file descriptor
  int fd_;

  // transfer
  spi_ioc_transfer ioc_transfer_base_;
  std::vector<spi_ioc_transfer> ioc_transfers_;

  // buffer
  MultiByteArrayMsg::UniquePtr spi_miso_msg_;

  // ROS
  rclcpp::Publisher<MultiByteArrayMsg>::SharedPtr spi_miso_publisher_;
  rclcpp::Subscription<MultiByteArrayMsg>::SharedPtr spi_mosi_subscriber_;
};

}  // namespace comms_bridge_ros

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(comms_bridge_ros::SpiBridgeNode)
