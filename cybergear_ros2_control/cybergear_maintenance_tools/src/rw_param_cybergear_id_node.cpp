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
#include <chrono>
#include <cstring>
#include <cybergear_driver_core/cybergear_driver_core.hpp>
#include <cybergear_maintenance_tools/rw_param_cybergear_id_node_parameters.hpp>
#include <iomanip>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ros2_socketcan/socket_can_receiver.hpp>
#include <ros2_socketcan/socket_can_sender.hpp>
#include <thread>

namespace cybergear_maintenance_tools {
class RwParamCybergearIdNode : public rclcpp::Node {
   public:
    RwParamCybergearIdNode() = delete;
    explicit RwParamCybergearIdNode(const rclcpp::NodeOptions &options);
    RwParamCybergearIdNode(const RwParamCybergearIdNode &) = delete;
    RwParamCybergearIdNode(RwParamCybergearIdNode &&) = delete;
    ~RwParamCybergearIdNode() override;

    RwParamCybergearIdNode &operator=(const RwParamCybergearIdNode &) = delete;
    RwParamCybergearIdNode &operator=(RwParamCybergearIdNode &&) = delete;

   private:
    static constexpr std::chrono::nanoseconds kSendTimeout{std::chrono::milliseconds(10)};
    static constexpr std::chrono::nanoseconds kReceiveTimeout{std::chrono::milliseconds(10)};

    std::unique_ptr<rw_param_cybergear_id_node::ParamListener> param_listener_;
    std::unique_ptr<rw_param_cybergear_id_node::Params> params_;

    std::unique_ptr<cybergear_driver_core::CybergearPacket> packet_;

    std::unique_ptr<drivers::socketcan::SocketCanSender> sender_;
    std::unique_ptr<drivers::socketcan::SocketCanReceiver> receiver_;

    std::thread worker_thread_;

    void run();
    bool send(const cybergear_driver_core::CanFrame &);
    bool readParameter(uint16_t, float &);
};

RwParamCybergearIdNode::RwParamCybergearIdNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("rw_param_cybergear_id", options),
      param_listener_(nullptr),
      params_(nullptr),
      packet_(nullptr),
      sender_(nullptr),
      receiver_(nullptr) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Start " << this->get_name());

    param_listener_ = std::make_unique<rw_param_cybergear_id_node::ParamListener>(this->get_node_parameters_interface());
    params_ = std::make_unique<rw_param_cybergear_id_node::Params>(param_listener_->get_params());

    RCLCPP_INFO_STREAM(this->get_logger(), "CAN bus: " << params_->can_bus);
    RCLCPP_INFO_STREAM(this->get_logger(), "Primary ID: " << params_->primary_id);
    RCLCPP_INFO_STREAM(this->get_logger(), "Device ID: " << params_->device_id);
    RCLCPP_INFO_STREAM(this->get_logger(), "Parameter index: 0x" << std::hex << params_->param_index << std::dec);

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

    worker_thread_ = std::thread(&RwParamCybergearIdNode::run, this);
}

RwParamCybergearIdNode::~RwParamCybergearIdNode() {
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Finish " << this->get_name());
}

void RwParamCybergearIdNode::run() {
    const auto index = static_cast<uint16_t>(params_->param_index);

    float value_before = 0.0F;
    if (!readParameter(index, value_before)) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "No reply from device " << params_->device_id << " on " << params_->can_bus << "; aborting");
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Value before: " << std::fixed << std::setprecision(6) << value_before);

    if (!params_->write) {
        rclcpp::shutdown();
        return;
    }

    const auto new_value = static_cast<float>(params_->value);
    std::array<uint8_t, 4> payload{};
    std::memcpy(payload.data(), &new_value, sizeof(new_value));

    if (!send(packet_->createWriteParameter(index, payload))) {
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Wrote " << new_value << " to 0x" << std::hex << index << std::dec << " on device " << params_->device_id);

    const auto settle_duration = std::chrono::duration<double>(params_->wait_after_write);
    std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(settle_duration));

    float value_after = 0.0F;
    if (!readParameter(index, value_after)) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "No reply after write; could not verify device " << params_->device_id);
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Value after: " << std::fixed << std::setprecision(6) << value_after);

    rclcpp::shutdown();
}

bool RwParamCybergearIdNode::send(const cybergear_driver_core::CanFrame &frame) {
    using drivers::socketcan::CanId;
    using drivers::socketcan::ExtendedFrame;
    using drivers::socketcan::FrameType;

    const CanId send_id(frame.id, 0, FrameType::DATA, ExtendedFrame);
    try {
        sender_->send(frame.data.data(), frame.data.size(), send_id, kSendTimeout);
    } catch (const std::exception &ex) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Error sending CAN message on " << params_->can_bus << ": " << ex.what());
        return false;
    }
    return true;
}

bool RwParamCybergearIdNode::readParameter(const uint16_t index, float &value) {
    cybergear_driver_core::CanFrame request;
    request.id = packet_->frameId().getReadParameterId();
    request.data.fill(0);
    request.data[0] = index & 0x00ff;
    request.data[1] = index >> 8;

    if (!send(request)) {
        return false;
    }

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(params_->wait_recive_can_frame));
    std::array<uint8_t, 8> data{};

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
        drivers::socketcan::CanId can_id;
        try {
            can_id = receiver_->receive(data.data(), kReceiveTimeout);
        } catch (const std::exception &) {
            continue;
        }
        const uint32_t id = can_id.identifier();
        if ((id >> 24) != cybergear_driver_core::commands::READ_PARAMETER) {
            continue;
        }
        if (packet_->frameId().getFrameId(id) != static_cast<uint8_t>(params_->device_id)) {
            continue;
        }
        const uint16_t reply_index = static_cast<uint16_t>(data[0] | (data[1] << 8));
        if (reply_index != index) {
            continue;
        }
        std::memcpy(&value, &data[4], sizeof(value));
        return true;
    }
    return false;
}
}  // namespace cybergear_maintenance_tools

RCLCPP_COMPONENTS_REGISTER_NODE(cybergear_maintenance_tools::RwParamCybergearIdNode)
