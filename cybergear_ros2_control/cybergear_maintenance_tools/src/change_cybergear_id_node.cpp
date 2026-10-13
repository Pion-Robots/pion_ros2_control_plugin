// MIT License
//
// Copyright (c) 2024 Naoki Takahashi
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <array>
#include <atomic>
#include <chrono>
#include <cybergear_driver_core/cybergear_driver_core.hpp>
#include <cybergear_maintenance_tools/change_cybergear_id_node_parameters.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ros2_socketcan/socket_can_receiver.hpp>
#include <ros2_socketcan/socket_can_sender.hpp>
#include <thread>

namespace cybergear_maintenance_tools {
class ChangeCybergearIdNode : public rclcpp::Node {
   public:
    ChangeCybergearIdNode() = delete;
    explicit ChangeCybergearIdNode(const rclcpp::NodeOptions &options);
    ChangeCybergearIdNode(const ChangeCybergearIdNode &) = delete;
    ChangeCybergearIdNode(ChangeCybergearIdNode &&) = delete;
    ~ChangeCybergearIdNode() override;

    ChangeCybergearIdNode &operator=(const ChangeCybergearIdNode &) = delete;
    ChangeCybergearIdNode &operator=(ChangeCybergearIdNode &&) = delete;

   private:
    static constexpr std::chrono::nanoseconds kSendTimeout{std::chrono::milliseconds(10)};
    static constexpr std::chrono::nanoseconds kReceiveTimeout{std::chrono::milliseconds(10)};

    rclcpp::TimerBase::SharedPtr send_can_frame_timer_;

    std::unique_ptr<change_cybergear_id_node::ParamListener> param_listener_;
    std::unique_ptr<change_cybergear_id_node::Params> params_;

    std::unique_ptr<cybergear_driver_core::CybergearPacket> packet_;

    std::unique_ptr<drivers::socketcan::SocketCanSender> sender_;
    std::unique_ptr<drivers::socketcan::SocketCanReceiver> receiver_;

    std::thread receive_thread_;
    std::atomic<bool> receiving_;
    std::atomic<bool> id_changed_;

    unsigned int callback_counter_;

    void receiveLoop();
    void sendChangeIdFrame();
    void sendCanFrameTimerCallback();
};

ChangeCybergearIdNode::ChangeCybergearIdNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("change_cybergear_id", options),
      send_can_frame_timer_(nullptr),
      param_listener_(nullptr),
      params_(nullptr),
      packet_(nullptr),
      sender_(nullptr),
      receiver_(nullptr),
      receiving_(false),
      id_changed_(false),
      callback_counter_(0) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Start " << this->get_name());

    param_listener_ = std::make_unique<change_cybergear_id_node::ParamListener>(this->get_node_parameters_interface());
    params_ = std::make_unique<change_cybergear_id_node::Params>(param_listener_->get_params());

    // Log parameters
    RCLCPP_INFO_STREAM(this->get_logger(), "CAN bus: " << params_->can_bus);
    RCLCPP_INFO_STREAM(this->get_logger(), "Primary ID: " << params_->primary_id);
    RCLCPP_INFO_STREAM(this->get_logger(), "Device ID: " << params_->device_id);
    RCLCPP_INFO_STREAM(this->get_logger(), "Target ID: " << params_->target_id);

    cybergear_driver_core::CybergearPacketParam packet_param;
    packet_param.primary_id = static_cast<int>(params_->primary_id);
    packet_param.device_id = static_cast<int>(params_->device_id);
    packet_ = std::make_unique<cybergear_driver_core::CybergearPacket>(packet_param);

    try {
        sender_ = std::make_unique<drivers::socketcan::SocketCanSender>(params_->can_bus, false);
        receiver_ = std::make_unique<drivers::socketcan::SocketCanReceiver>(params_->can_bus, false);
    } catch (const std::exception &ex) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to open CAN interface " << params_->can_bus << ": " << ex.what());
        throw;
    }

    receiving_ = true;
    receive_thread_ = std::thread(&ChangeCybergearIdNode::receiveLoop, this);

    const auto send_duration_milliseconds = static_cast<unsigned int>(1e3 / params_->send_frequency);

    send_can_frame_timer_ = this->create_wall_timer(std::chrono::milliseconds(send_duration_milliseconds),
                                                    std::bind(&ChangeCybergearIdNode::sendCanFrameTimerCallback, this));
}

ChangeCybergearIdNode::~ChangeCybergearIdNode() {
    receiving_ = false;
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Finish " << this->get_name());
}

void ChangeCybergearIdNode::receiveLoop() {
    std::array<uint8_t, 8> data{};

    while (receiving_.load(std::memory_order_acquire)) {
        drivers::socketcan::CanId can_id;
        try {
            can_id = receiver_->receive(data.data(), kReceiveTimeout);
        } catch (const std::exception &) {
            continue;
        }
        const uint32_t id = can_id.identifier();
        if (!packet_->frameId().isInfo(id)) {
            continue;
        }
        const unsigned int device_id = packet_->frameId().getFrameId(id);
        if (device_id == static_cast<unsigned int>(params_->primary_id)) {
            continue;
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Found new CyberGear device: " << device_id << " on " << params_->can_bus);
        if (device_id == static_cast<unsigned int>(params_->target_id)) {
            id_changed_ = true;
        }
    }
}

void ChangeCybergearIdNode::sendChangeIdFrame() {
    using drivers::socketcan::CanId;
    using drivers::socketcan::ExtendedFrame;
    using drivers::socketcan::FrameType;

    constexpr std::array<uint8_t, 8> kEmptyData{};

    const CanId send_id(packet_->frameId().getChangeDeviceId(static_cast<uint8_t>(params_->target_id)), 0, FrameType::DATA, ExtendedFrame);
    try {
        sender_->send(kEmptyData.data(), kEmptyData.size(), send_id, kSendTimeout);
    } catch (const std::exception &ex) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Error sending CAN message on " << params_->can_bus << ": " << ex.what());
    }
}

void ChangeCybergearIdNode::sendCanFrameTimerCallback() {
    if (callback_counter_ == 0) {
        sendChangeIdFrame();
        callback_counter_++;
        return;
    }

    const double send_duration_seconds = 1 / params_->send_frequency;
    const double wait_seconds = callback_counter_ * send_duration_seconds;
    callback_counter_++;

    if (wait_seconds > params_->wait_recive_can_frame) {
        if (id_changed_.load(std::memory_order_acquire)) {
            RCLCPP_INFO_STREAM(this->get_logger(),
                               "Changed device id " << params_->device_id << " -> " << params_->target_id << " on " << params_->can_bus);
        } else {
            RCLCPP_WARN_STREAM(this->get_logger(),
                               "No reply from device " << params_->device_id << " on " << params_->can_bus << "; id may not have changed");
        }
        receiving_ = false;
        rclcpp::shutdown();
    }
}
}  // namespace cybergear_maintenance_tools

RCLCPP_COMPONENTS_REGISTER_NODE(cybergear_maintenance_tools::ChangeCybergearIdNode)
