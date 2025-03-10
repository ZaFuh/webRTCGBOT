#include "teleop/webrtc.hpp"
#include <chrono>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rtc/rtc.hpp>
#include <thread>

using json = nlohmann::json;

namespace teleop {

namespace {
json create_json_message(const std::optional<rtc::Description> &description) {
  if (!description.has_value()) {
    throw std::runtime_error("No description available");
  }
  json j;
  j["sdp"] = description->generateSdp();
  return j;
}
} // namespace

WebRTCConnection::WebRTCConnection(const std::string &signaling_url)
    : signaling_url_(signaling_url) {
  rtc::Configuration config;
  config.iceServers.emplace_back("stun:stun.l.google.com:19302");

  peer_connection_ = std::make_shared<rtc::PeerConnection>(config);

  // Set up connection state callback
  peer_connection_->onStateChange([this](rtc::PeerConnection::State state) {
    std::cout << "Peer connection state changed to: " << static_cast<int>(state)
              << std::endl;
    if (on_connection_state_change_) {
      on_connection_state_change_(state);
    }
  });

  peer_connection_->onDataChannel([this](std::shared_ptr<rtc::DataChannel> dc) {
    std::cout << "Received data channel: " << dc->label() << std::endl;
    if (dc->label() == "text") {
      text_channel_ = dc;
      SetupDataChannel(text_channel_);
    } else if (dc->label() == "video") {
      video_channel_ = dc;
    }
  });

  // Initialize WebSocket client
  ws_client_ = std::make_unique<WebSocketClient>();
  ws_client_->clear_access_channels(websocketpp::log::alevel::all);
  ws_client_->clear_error_channels(websocketpp::log::elevel::all);
  ws_client_->init_asio();

  // Set up WebSocket callbacks
  ws_client_->set_open_handler([this](websocketpp::connection_hdl hdl) {
    ws_connection_hdl_ = hdl;
    if (!is_offering_) {
      is_offering_ = true;
      HandleSignaling();
    }
  });

  ws_client_->set_message_handler(
      [this](websocketpp::connection_hdl, WebSocketClient::message_ptr msg) {
        try {
          auto j = json::parse(msg->get_payload());
          if (j["type"] == "offer") {
            if (!peer_connection_) {
              InitiatePeerConnection();
            }
            peer_connection_->setRemoteDescription(j["sdp"].get<std::string>());
            peer_connection_->setLocalDescription();

            auto answer = peer_connection_->localDescription();
            SendSignalingMessage("answer", create_json_message(answer));
            is_offering_ = false;
          } else if (j["type"] == "answer" && is_offering_) {
            peer_connection_->setRemoteDescription(j["sdp"].get<std::string>());
          } else if (j["type"] == "candidate") {
            if (peer_connection_) {
              rtc::Candidate candidate(j["candidate"].get<std::string>(),
                                       j["mid"].get<std::string>());
              peer_connection_->addRemoteCandidate(candidate);
            }
          } else if (j["type"] == "reset") {
            ResetConnection();
            InitiatePeerConnection();
            if (is_offering_) {
              HandleSignaling();
            }
          }
        } catch (const std::exception &e) {
          std::cerr << "Error handling WebSocket message: " << e.what()
                    << std::endl;
        }
      });
}

void WebRTCConnection::Initialize() {
  if (initialized_)
    return;

  SetupVideoChannel();
  ConnectToSignalingServer();

  initialized_ = true;
}

void WebRTCConnection::ResetConnection() {
  if (video_channel_) {
    video_channel_->close();
    video_channel_ = nullptr;
  }
  if (peer_connection_) {
    peer_connection_->close();
    peer_connection_ = nullptr;
  }
  is_offering_ = false;
}

void WebRTCConnection::InitiatePeerConnection() {
  if (!peer_connection_) {
    rtc::Configuration config;
    peer_connection_ = std::make_shared<rtc::PeerConnection>(config);

    // Reset callbacks
    peer_connection_->onStateChange([this](rtc::PeerConnection::State state) {
      if (on_connection_state_change_) {
        on_connection_state_change_(state);
      }
    });

    peer_connection_->onLocalCandidate([this](rtc::Candidate candidate) {
      json j;
      j["candidate"] = candidate.candidate();
      j["mid"] = candidate.mid();
      SendSignalingMessage("candidate", j);
    });

    SetupVideoChannel();
  }
}

void WebRTCConnection::Reconnect() {
  try {
    ResetConnection();
    InitiatePeerConnection();

    if (ws_client_ && ws_connection_hdl_.lock()) {
      is_offering_ = true;
      HandleSignaling();
    } else {
      ConnectToSignalingServer();
    }
  } catch (const std::exception &e) {
    std::cerr << "Error during reconnection: " << e.what() << std::endl;
  }
}

void WebRTCConnection::ConnectToSignalingServer() {
  if (ws_thread_.joinable()) {
    ws_running_ = false;
    ws_thread_.join();
  }
  websocketpp::lib::error_code ec;
  auto connection = ws_client_->get_connection(signaling_url_, ec);
  if (ec) {
    throw std::runtime_error("Could not create WebSocket connection: " +
                             ec.message());
  }

  ws_client_->connect(connection);

  ws_running_ = true;
  ws_thread_ = std::thread([this]() {
    while (ws_running_) {
      try {
        ws_client_->run_one();
      } catch (const std::exception &e) {
        std::cerr << "WebSocket error: " << e.what() << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }
  });
}

void WebRTCConnection::SendSignalingMessage(const std::string &type,
                                            const nlohmann::json &data) {
  if (!ws_client_)
    return;

  try {
    nlohmann::json message = data;
    message["type"] = type;

    ws_client_->send(ws_connection_hdl_, message.dump(),
                     websocketpp::frame::opcode::text);
  } catch (const std::exception &e) {
    std::cerr << "Error sending signaling message: " << e.what() << std::endl;
  }
}

void WebRTCConnection::SetupVideoChannel() {
  if (!peer_connection_) {
    std::cerr << "Cannot setup channels: peer connection is null" << std::endl;
    return;
  }

  // Create video data channel if it doesn't exist or is closed
  if (!video_channel_ || video_channel_->isClosed()) {
    video_channel_ = peer_connection_->createDataChannel("video");
    std::cout << "Created video data channel" << std::endl;

    // Set up video channel callbacks
    video_channel_->onOpen(
        []() { std::cout << "Video channel opened" << std::endl; });

    video_channel_->onClosed(
        []() { std::cout << "Video channel closed" << std::endl; });

    video_channel_->onError([](std::string error) {
      std::cerr << "Video channel error: " << error << std::endl;
    });

    SetupDataChannel(video_channel_);
  }

  // Create text data channel if it doesn't exist or is closed
  if (!text_channel_ || text_channel_->isClosed()) {
    text_channel_ = peer_connection_->createDataChannel("text");
    std::cout << "Created text data channel" << std::endl;

    // Set up text channel callbacks
    text_channel_->onOpen(
        []() { std::cout << "Text channel opened" << std::endl; });

    text_channel_->onClosed(
        []() { std::cout << "Text channel closed" << std::endl; });

    text_channel_->onError([](std::string error) {
      std::cerr << "Text channel error: " << error << std::endl;
    });

    SetupDataChannel(text_channel_);
  }

  std::cout << "Data channels setup completed" << std::endl;

  // Handle incoming messages
  video_channel_->onMessage([this](
                                std::variant<rtc::binary, rtc::string> data) {
    try {
      if (auto text = std::get_if<rtc::string>(&data)) {
        std::cout << "Received data channel message: " << *text << std::endl;
        auto json_data = nlohmann::json::parse(*text);

        if (!json_data.contains("type")) {
          std::cerr << "Received message without type field" << std::endl;
          return;
        }

        std::string type = json_data["type"].get<std::string>();
        std::cout << "Message type: " << type << std::endl;

        if (on_data_channel_message_) {
          std::cout << "Calling data channel message callback" << std::endl;
          on_data_channel_message_(type, json_data);
        } else {
          std::cout << "No data channel message callback registered"
                    << std::endl;
        }
      } else {
        std::cout << "Received binary message of size: "
                  << std::get<rtc::binary>(data).size() << " bytes"
                  << std::endl;
      }
    } catch (const std::exception &e) {
      std::cerr << "Error handling data channel message: " << e.what()
                << std::endl;
    }
  });
}

void WebRTCConnection::SendVideoFrame(const cv::Mat &frame) {
  static auto last_reconnect_attempt = std::chrono::steady_clock::now();
  static bool waiting_for_client = true;

  // Check if we have an active connection
  if (!video_channel_ || video_channel_->isClosed() || !peer_connection_ ||
      peer_connection_->state() != rtc::PeerConnection::State::Connected) {

    if (waiting_for_client) {
      std::cout << "Waiting for client to connect..." << std::endl;
      waiting_for_client = false;
      return;
    }

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - last_reconnect_attempt);

    // Try to reconnect every 5 seconds
    if (elapsed.count() >= 5) {
      std::cout << "Connection lost. Attempting to reconnect..." << std::endl;
      Reconnect();
      last_reconnect_attempt = now;
    }
    return;
  }

  // Reset waiting flag when connected
  if (!waiting_for_client) {
    std::cout << "Client connected, starting video stream." << std::endl;
    waiting_for_client = true;
  }

  try {
    // Compress frame to JPEG
    std::vector<uchar> buffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
    cv::imencode(".jpg", frame, buffer, params);

    // Send frame over data channel
    if (video_channel_->isOpen()) {
      video_channel_->send(reinterpret_cast<const std::byte *>(buffer.data()),
                           buffer.size());
    }
  } catch (const std::exception &e) {
    std::cerr << "Error sending video frame: " << e.what() << std::endl;
  }
}

void WebRTCConnection::SetupDataChannel(
    std::shared_ptr<rtc::DataChannel> channel) {
  if (!channel)
    return;

  const std::string channel_label = channel->label();
  std::cout << "Setting up data channel: " << channel_label << std::endl;

  channel->onOpen([channel_label]() {
    std::cout << "Data channel '" << channel_label << "' opened" << std::endl;
  });

  channel->onClosed([channel_label]() {
    std::cout << "Data channel '" << channel_label << "' closed" << std::endl;
  });

  channel->onError([channel_label](const std::string &error) {
    std::cerr << "Data channel '" << channel_label << "' error: " << error
              << std::endl;
  });

  channel->onMessage(
      [this, channel_label](std::variant<rtc::binary, rtc::string> data) {
        try {
          if (auto text = std::get_if<rtc::string>(&data)) {
            std::cout << "Received message on " << channel_label
                      << " channel: " << *text << std::endl;
            auto json_data = nlohmann::json::parse(*text);

            if (!json_data.contains("type")) {
              std::cout << "Warning: Message missing type field: " << *text
                        << std::endl;
              return;
            }

            std::string type = json_data["type"].get<std::string>();
            if (on_data_channel_message_) {
              on_data_channel_message_(type, json_data);
            } else {
              std::cout << "No message callback registered for "
                        << channel_label << std::endl;
            }
          } else {
            const auto &binary = std::get<rtc::binary>(data);
            std::cout << "Received binary data on " << channel_label
                      << " channel, size: " << binary.size() << " bytes"
                      << std::endl;
          }
        } catch (const std::exception &e) {
          std::cerr << "Error handling message on " << channel_label
                    << " channel: " << e.what() << std::endl;
        }
      });
}

void WebRTCConnection::HandleSignaling() {
  try {
    if (!peer_connection_) {
      InitiatePeerConnection();
    }

    // Create and send offer
    peer_connection_->setLocalDescription();
    auto sdp = peer_connection_->localDescription();
    SendSignalingMessage("offer", create_json_message(sdp));

    // Handle ICE candidates
    peer_connection_->onLocalCandidate([this](rtc::Candidate candidate) {
      json j;
      j["candidate"] = candidate.candidate();
      j["mid"] = candidate.mid();
      SendSignalingMessage("candidate", j);
    });

  } catch (const std::exception &e) {
    std::cerr << "Error in signaling: " << e.what() << std::endl;
    // Schedule a reconnection attempt
    std::thread([this]() {
      std::this_thread::sleep_for(std::chrono::seconds(2));
      Reconnect();
    }).detach();
  }
}

WebRTCConnection::~WebRTCConnection() {
  try {
    // Stop the WebSocket first
    ws_running_ = false;
    if (ws_client_) {
      websocketpp::lib::error_code ec;
      ws_client_->close(ws_connection_hdl_, websocketpp::close::status::normal,
                        "", ec);
      ws_client_->stop();
    }

    // Wait for WebSocket thread to finish
    if (ws_thread_.joinable()) {
      ws_thread_.join();
    }

    // Close video channel
    if (video_channel_) {
      video_channel_->close();
      video_channel_ = nullptr;
    }

    // Close text channel
    if (text_channel_) {
      text_channel_->close();
      text_channel_ = nullptr;
    }

    // Close peer connection
    if (peer_connection_) {
      peer_connection_->close();
      peer_connection_ = nullptr;
    }

  } catch (const std::exception &e) {
    std::cerr << "Error during cleanup: " << e.what() << std::endl;
  }
}
} // namespace teleop
