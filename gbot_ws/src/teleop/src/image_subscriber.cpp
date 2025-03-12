#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "teleop/webrtc.hpp"

class ImageSubscriber : public rclcpp::Node {
public:
  ImageSubscriber(bool show_camera,
                  const std::string &ws_url = "ws://localhost:8080")
      : Node("image_subscriber"), show_camera_(show_camera), webrtc_(ws_url) {

    // Initialize WebRTC connection
    webrtc_.Initialize();

    // Set WebRTC connection state callback
    webrtc_.SetOnConnectionStateChange(
        [this](rtc::PeerConnection::State state) {
          RCLCPP_INFO(this->get_logger(), "WebRTC state changed to: %d",
                      static_cast<int>(state));
        });

    // Set data channel message callback
    RCLCPP_INFO(this->get_logger(),
                "Setting up data channel message callback...");
    webrtc_.SetOnDataChannelMessage(
        [this](const std::string &type, const nlohmann::json &data) {
          try {
            RCLCPP_INFO(this->get_logger(), "Data channel callback triggered!");
            RCLCPP_INFO(this->get_logger(), "Received message type: %s",
                        type.c_str());

            // Log the entire message content
            RCLCPP_INFO(this->get_logger(), "Message content: %s",
                        data.dump(2).c_str());

            // Handle specific message types
            if (type == "text") {
              RCLCPP_INFO(this->get_logger(), "Received text message: %s",
                          data.dump().c_str());
              if (data.contains("payload")) {
                RCLCPP_INFO(this->get_logger(), "Text message payload: %s",
                            data["payload"].get<std::string>().c_str());
              } else {
                RCLCPP_WARN(this->get_logger(),
                            "Text message missing payload field");
              }
            } else if (type == "command") {
              if (data.contains("command")) {
                RCLCPP_INFO(this->get_logger(), "Command received: %s",
                            data["command"].get<std::string>().c_str());
              } else {
                RCLCPP_WARN(this->get_logger(),
                            "Command message missing command field");
              }
            } else {
              RCLCPP_WARN(this->get_logger(), "Unknown message type: %s",
                          type.c_str());
            }
          } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing message: %s",
                         e.what());
          }
        });
    RCLCPP_INFO(this->get_logger(),
                "Data channel message callback registered successfully");
    RCLCPP_INFO(this->get_logger(), "Data channel message callback registered");
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 10,
        std::bind(&ImageSubscriber::image_callback, this,
                  std::placeholders::_1));

    if (show_camera_) {
      // Create a resizable window only if display is enabled
      cv::namedWindow("Camera Feed", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
      // Set initial window size to something reasonable (e.g., 640x480)
      cv::resizeWindow("Camera Feed", 640, 480);
      // Allow the window to be smaller than the image
      cv::setWindowProperty("Camera Feed", cv::WND_PROP_ASPECT_RATIO,
                            cv::WINDOW_KEEPRATIO);
    }

    RCLCPP_INFO(this->get_logger(),
                "Image subscriber node started (Display: %s)",
                show_camera_ ? "enabled" : "disabled");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    static rclcpp::Time last_reconnect_attempt = this->now();
    try {
      // Convert ROS Image message to OpenCV image
      cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

      // Send frame through WebRTC
      if (!webrtc_.IsConnected()) {
        auto now = this->now();
        // Attempt reconnection every 5 seconds
        if ((now - last_reconnect_attempt).seconds() >= 5.0) {
          RCLCPP_INFO(this->get_logger(),
                      "Connection lost, attempting to reconnect...");
          webrtc_.Reconnect();
          last_reconnect_attempt = now;
        }
      } else {
        webrtc_.SendVideoFrame(frame);
      }

      if (show_camera_) {
        // Display the image in the resizable window only if display is
        // enabled
        cv::imshow("Camera Feed", frame);
        cv::waitKey(1); // Required for image display
      }

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                           1000, // Throttle to log once per second
                           "Received image: %dx%d", msg->height, msg->width);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error processing image: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  bool show_camera_;
  teleop::WebRTCConnection webrtc_;
};

void signal_handler(int signum) {
  std::cout << "\nReceived signal " << signum << ". Shutting down..."
            << std::endl;
  rclcpp::shutdown();
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Setup signal handling for both SIGINT and SIGTERM
  struct sigaction sa;
  sa.sa_handler = signal_handler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGINT, &sa, NULL);
  sigaction(SIGTERM, &sa, NULL);

  // Parse command line arguments
  bool show_camera = false;
  std::string ws_url = "ws://localhost:8080"; // Default value

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--show-camera") {
      show_camera = true;
    } else if (arg == "--ws-url" && i + 1 < argc) {
      ws_url = argv[++i];
    }
  }

  // Print the configuration
  std::cout << "Starting with configuration:" << std::endl
            << "  WebSocket URL: " << ws_url << std::endl
            << "  Show camera: " << (show_camera ? "yes" : "no") << std::endl;

  auto node = std::make_shared<ImageSubscriber>(show_camera, ws_url);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
