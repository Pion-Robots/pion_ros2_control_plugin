#ifndef SEALAND_CONTROL__IMU_HARDWARE_HPP_
#define SEALAND_CONTROL__IMU_HARDWARE_HPP_

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include "ch_serial.hpp"

#define GRA_ACC (9.8)
#define DEG_TO_RAD (0.01745329)
#define BUF_SIZE (1024)

#ifdef __cplusplus
}
#endif

namespace sealand_control {
class IMUHardware : public hardware_interface::SystemInterface {
   public:
    RCLCPP_SHARED_PTR_DEFINITIONS(IMUHardware)

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
        // IMU is read-only, no command interfaces
        return std::vector<hardware_interface::CommandInterface>();
    }

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

   private:
    // Hardware parameters
    std::string serial_device_;
    int baud_rate_;
    int serial_fd_ = -1;

    // IMU data storage
    std::vector<double> orientation_;          // quaternion [w, x, y, z]
    std::vector<double> angular_velocity_;     // rad/s [x, y, z]
    std::vector<double> linear_acceleration_;  // m/s² [x, y, z]

    // Serial communication
    uint8_t buf_[BUF_SIZE];
    raw_t raw_;

    // Helper functions
    int open_serial();
};

}  // namespace sealand_control

#endif  // SEALAND_CONTROL__IMU_HARDWARE_HPP_
