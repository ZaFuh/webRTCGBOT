#ifndef OBJECT_SUBCRIBER_HPP
#define OBJECT_SUBCRIBER_HPP

#include <functional>
#include <memory>
#include <rclcpp/executors.hpp>
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>
#include <string>

#include "gbot_navigation/coordinates.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <atomic>
#include <mutex>
#include <vector>

using std::placeholders::_1;

namespace Navigation {
class ObjectSubscriber : public rclcpp::Node {
public:
  explicit ObjectSubscriber(
      const std::string &topicName,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  virtual ~ObjectSubscriber() = default;
  // Get the latest coordinates - elements cannot be modified
  const std::vector<Coordinates> &get_positions() const {
    return buffers_[read_buffer_idx_.load()];
  }

private:
  void topic_callback(const std_msgs::msg::Float32MultiArray &msg);

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      subscription_;

  std::vector<Coordinates> buffers_[2]; // Double buffer
  std::atomic<int> read_buffer_idx_{0}; // Index of the readable buffer
  std::mutex swap_mutex_;               // Protects buffer swapping
};

} // namespace Navigation

#endif // !OBJECT_SUBCRIBER_HPP
