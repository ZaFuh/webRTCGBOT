#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/core/core.hpp>
#include <rtc/rtc.hpp>
#include <string>
#include <thread>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>

namespace teleop {

class WebRTCConnection {
public:
  explicit WebRTCConnection(const std::string &signaling_url);
  ~WebRTCConnection();

  // Initialize WebRTC and connect to signaling server
  void Initialize();
  void Reconnect();
  bool IsConnected() const {
    return peer_connection_ && video_channel_ && !video_channel_->isClosed() &&
           text_channel_ && !text_channel_->isClosed();
  }

  // Video handling
  void SendVideoFrame(const cv::Mat &frame);

  // Send a text message through the data channel
  bool SendTextMessage(const std::string &type, const nlohmann::json &data) {
    if (!text_channel_ || text_channel_->isClosed()) {
      std::cerr << "Text channel not available" << std::endl;
      return false;
    }
    try {
      nlohmann::json message = data;
      message["type"] = type;
      text_channel_->send(message.dump());
      return true;
    } catch (const std::exception &e) {
      std::cerr << "Error sending text message: " << e.what() << std::endl;
      return false;
    }
  }

  // Connection state callback
  void SetOnConnectionStateChange(
      std::function<void(rtc::PeerConnection::State)> callback) {
    on_connection_state_change_ = std::move(callback);
  }

  // Add callback for data channel messages
  void SetOnDataChannelMessage(
      std::function<void(const std::string &, const nlohmann::json &)>
          callback) {
    on_data_channel_message_ = std::move(callback);
  }

private:
  void SetupVideoChannel();
  void HandleSignaling();
  void ConnectToSignalingServer();
  void SendSignalingMessage(const std::string &type,
                            const nlohmann::json &data);
  void ResetConnection();
  void InitiatePeerConnection();
  void SetupDataChannel(std::shared_ptr<rtc::DataChannel> channel);

  // WebRTC objects
  std::shared_ptr<rtc::PeerConnection> peer_connection_;
  std::shared_ptr<rtc::DataChannel> video_channel_;
  std::shared_ptr<rtc::DataChannel> text_channel_;

  // WebSocket client
  using WebSocketClient = websocketpp::client<websocketpp::config::asio_client>;
  std::unique_ptr<WebSocketClient> ws_client_;
  websocketpp::connection_hdl ws_connection_hdl_;
  std::thread ws_thread_;
  std::atomic<bool> ws_running_{false};

  // Callbacks
  std::function<void(rtc::PeerConnection::State)> on_connection_state_change_;
  std::function<void(const std::string &, const nlohmann::json &)>
      on_data_channel_message_;

  // Configuration
  std::string signaling_url_;
  bool initialized_ = false;
  std::atomic<bool> is_offering_{false};
  std::atomic<bool> stop_connection_check_{false};
  std::thread connection_check_thread_;
};
} // namespace teleop
