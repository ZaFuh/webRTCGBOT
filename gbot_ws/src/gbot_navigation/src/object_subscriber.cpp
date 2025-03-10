#include "gbot_navigation/object_subscriber.hpp"
#include <iostream>
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>
#include <string>

// TODO: move all the functionality here since the main thread is
// blocked by the node spin call
namespace Navigation {
ObjectSubscriber::ObjectSubscriber(const std::string &topicName,
                                   const rclcpp::NodeOptions &options)
    : Node("navigation_object_subscriber", options) {
  try {
    if (!rclcpp::ok()) {
      throw std::runtime_error(
          "ROS 2 context is not initialized. Call rclcpp::init() first!");
    }

    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        topicName, rclcpp::QoS(10),
        std::bind(&ObjectSubscriber::topic_callback, this, _1));

    if (!subscription_) {
      throw std::runtime_error("Failed to create subscription");
    }

    RCLCPP_INFO(this->get_logger(), "ObjectSubscriber initialized on topic: %s",
                topicName.c_str());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Error initializing ObjectSubscriber: %s",
                 e.what());
    throw;
  }
}

void ObjectSubscriber::topic_callback(
    const std_msgs::msg::Float32MultiArray &msg) {

  if (msg.data.size() >= 2 && msg.data.size() % 2 == 0) {
    // Write to the non-reading buffer
    int write_idx = 1 - read_buffer_idx_.load();
    auto &write_buffer = buffers_[write_idx];

    const size_t num_coordinates = msg.data.size() / 2;
    write_buffer.clear();
    write_buffer.reserve(num_coordinates);

    // Process pairs of coordinates
    for (size_t i = 0; i < msg.data.size(); i += 2) {
      write_buffer.emplace_back(msg.data[i],    // x coordinate
                                msg.data[i + 1] // y coordinate
      );
    }

    // Quick atomic swap of the read buffer index
    {
      std::lock_guard<std::mutex> lock(swap_mutex_);
      read_buffer_idx_.store(write_idx);
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Invalid message size: %zu (must be even)",
                msg.data.size());
  }
}

} // namespace Navigation
