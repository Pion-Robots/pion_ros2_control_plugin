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
#include <cybergear_maintenance_tools/search_cybergear_id_node_parameters.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ros2_socketcan/socket_can_receiver.hpp>
#include <ros2_socketcan/socket_can_sender.hpp>
#include <set>
#include <sstream>
#include <string>
#include <thread>

namespace cybergear_maintenance_tools {
class SearchCybergearIdNode : public rclcpp::Node {
   public:
    SearchCybergearIdNode() = delete;
    explicit SearchCybergearIdNode(const rclcpp::NodeOptions &options);
    SearchCybergearIdNode(const SearchCybergearIdNode &) = delete;
    SearchCybergearIdNode(SearchCybergearIdNode &&) = delete;
    ~SearchCybergearIdNode() override;

    SearchCybergearIdNode &operator=(const SearchCybergearIdNode &) = delete;
    SearchCybergearIdNode &operator=(SearchCybergearIdNode &&) = delete;

   private:
    static constexpr unsigned int kMaxId = 255;
    //!< Host id used as the sender of the scan frames; no device may use it
    static constexpr unsigned int kHostId = 0;
    static constexpr std::chrono::nanoseconds kSendTimeout{std::chrono::milliseconds(10)};
    static constexpr std::chrono::nanoseconds kReceiveTimeout{std::chrono::milliseconds(10)};

    rclcpp::TimerBase::SharedPtr send_can_frame_timer_;

    std::unique_ptr<search_cybergear_id_node::ParamListener> param_listener_;
    std::unique_ptr<search_cybergear_id_node::Params> params_;

    std::unique_ptr<cybergear_driver_core::CybergearPacket> packet_;

    std::unique_ptr<drivers::socketcan::SocketCanSender> sender_;
    std::unique_ptr<drivers::socketcan::SocketCanReceiver> receiver_;

    std::thread receive_thread_;
    std::atomic<bool> receiving_;

    std::mutex found_device_ids_mutex_;
    std::set<unsigned int> found_device_ids_;

    unsigned int scan_device_id_;
    unsigned int wait_counter_;

    void receiveLoop();
    void reportFoundDeviceIds();
    void sendCanFrameTimerCallback();
};

SearchCybergearIdNode::SearchCybergearIdNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("search_cybergear_id", options),
      send_can_frame_timer_(nullptr),
      param_listener_(nullptr),
      params_(nullptr),
      packet_(nullptr),
      sender_(nullptr),
      receiver_(nullptr),
      receiving_(false),
      scan_device_id_(0),
      wait_counter_(0) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Start " << this->get_name());

    param_listener_ = std::make_unique<search_cybergear_id_node::ParamListener>(this->get_node_parameters_interface());
    params_ = std::make_unique<search_cybergear_id_node::Params>(param_listener_->get_params());

    cybergear_driver_core::CybergearPacketParam packet_param;
    packet_param.primary_id = static_cast<int>(kHostId);
    packet_ = std::make_unique<cybergear_driver_core::CybergearPacket>(packet_param);

    try {
        sender_ = std::make_unique<drivers::socketcan::SocketCanSender>(params_->can_bus, false);
        receiver_ = std::make_unique<drivers::socketcan::SocketCanReceiver>(params_->can_bus, false);
    } catch (const std::exception &ex) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to open CAN interface " << params_->can_bus << ": " << ex.what());
        throw;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Scanning CyberGear devices on " << params_->can_bus);

    receiving_ = true;
    receive_thread_ = std::thread(&SearchCybergearIdNode::receiveLoop, this);

    const auto send_duration_milliseconds = static_cast<unsigned int>(1e3 / params_->send_frequency);

    send_can_frame_timer_ = this->create_wall_timer(std::chrono::milliseconds(send_duration_milliseconds),
                                                    std::bind(&SearchCybergearIdNode::sendCanFrameTimerCallback, this));
}

SearchCybergearIdNode::~SearchCybergearIdNode() {
    receiving_ = false;
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Finish " << this->get_name());
}

void SearchCybergearIdNode::receiveLoop() {
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
        if (device_id == kHostId) {
            continue;
        }
        bool is_new_device;
        {
            const std::lock_guard<std::mutex> lock(found_device_ids_mutex_);
            is_new_device = found_device_ids_.insert(device_id).second;
        }
        if (is_new_device) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Found CyberGear device: " << device_id << " on " << params_->can_bus);
        }
    }
}

void SearchCybergearIdNode::reportFoundDeviceIds() {
    const std::lock_guard<std::mutex> lock(found_device_ids_mutex_);

    if (found_device_ids_.empty()) {
        RCLCPP_WARN_STREAM(this->get_logger(), "No CyberGear device found on " << params_->can_bus);
        return;
    }
    std::stringstream id_list;
    for (auto itr = found_device_ids_.begin(); itr != found_device_ids_.end(); ++itr) {
        if (itr != found_device_ids_.begin()) {
            id_list << ", ";
        }
        id_list << *itr;
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Found " << found_device_ids_.size() << " CyberGear device(s) on " << params_->can_bus << ": " << id_list.str());
}

void SearchCybergearIdNode::sendCanFrameTimerCallback() {
    using drivers::socketcan::CanId;
    using drivers::socketcan::ExtendedFrame;
    using drivers::socketcan::FrameType;

    constexpr std::array<uint8_t, 8> kEmptyData{};

    if (scan_device_id_ <= kMaxId) {
        if (scan_device_id_ != kHostId) {
            const CanId send_id(packet_->frameId().getInfoId(static_cast<uint8_t>(scan_device_id_)), 0, FrameType::DATA, ExtendedFrame);
            try {
                sender_->send(kEmptyData.data(), kEmptyData.size(), send_id, kSendTimeout);
            } catch (const std::exception &ex) {
                RCLCPP_WARN_STREAM(this->get_logger(), "Error sending CAN message on " << params_->can_bus << ": " << ex.what());
            }
        }
        scan_device_id_++;
        return;
    }

    const double send_duration_seconds = 1 / params_->send_frequency;
    const double wait_seconds = wait_counter_ * send_duration_seconds;
    wait_counter_++;

    if (wait_seconds > params_->wait_recive_can_frame) {
        reportFoundDeviceIds();
        receiving_ = false;
        rclcpp::shutdown();
    }
}
}  // namespace cybergear_maintenance_tools

RCLCPP_COMPONENTS_REGISTER_NODE(cybergear_maintenance_tools::SearchCybergearIdNode)
