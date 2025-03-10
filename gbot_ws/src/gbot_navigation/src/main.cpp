#include "gbot_navigation/object_subscriber.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto subscriber =
      std::make_shared<Navigation::ObjectSubscriber>("object_detection");
  rclcpp::spin(subscriber);
  rclcpp::shutdown();
  return 0;
}
