#include "sealand_control/imu_hardware_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace sealand_control {

static const std::string HW_NAME = "IMUHardware";

hardware_interface::CallbackReturn IMUHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams& params) {
    if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize serial port
    serial_device_ = info_.hardware_parameters["serial_device"];
    baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);

    RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Initializing IMU hardware interface with device: %s, baud: %d", serial_device_.c_str(), baud_rate_);

    // Initialize IMU data
    orientation_.resize(4, 0.0);  // quaternion [w, x, y, z]
    angular_velocity_.resize(3, 0.0);
    linear_acceleration_.resize(3, 0.0);

    // Initialize buffer
    memset(buf_, 0, sizeof(buf_));
    memset(&raw_, 0, sizeof(raw_));

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> IMUHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Orientation (quaternion)
    state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.w", &orientation_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.x", &orientation_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.y", &orientation_[2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.z", &orientation_[3]));

    // Angular velocity
    state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "angular_velocity.x", &angular_velocity_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "angular_velocity.y", &angular_velocity_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "angular_velocity.z", &angular_velocity_[2]));

    // Linear acceleration
    state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "linear_acceleration.x", &linear_acceleration_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "linear_acceleration.y", &linear_acceleration_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "linear_acceleration.z", &linear_acceleration_[2]));

    return state_interfaces;
}

hardware_interface::CallbackReturn IMUHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Activating IMU hardware interface");

    // Open serial port
    serial_fd_ = open_serial();
    if (serial_fd_ == -1) {
        RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Failed to open serial port");
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "IMU hardware interface activated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IMUHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Deactivating IMU hardware interface");

    if (serial_fd_ != -1) {
        close(serial_fd_);
        serial_fd_ = -1;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type IMUHardware::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    if (serial_fd_ == -1) {
        return hardware_interface::return_type::ERROR;
    }

    // Read data from serial port
    int n = ::read(serial_fd_, buf_, sizeof(buf_));

    if (n > 0) {
        for (int i = 0; i < n; i++) {
            int rev = ch_serial_input(&raw_, buf_[i]);

            if (raw_.item_code[raw_.nitem_code - 1] != KItemGWSOL) {
                if (rev) {
                    // Update orientation (quaternion)
                    orientation_[0] = raw_.imu[raw_.nimu - 1].quat[0];  // w
                    orientation_[1] = raw_.imu[raw_.nimu - 1].quat[1];  // x
                    orientation_[2] = raw_.imu[raw_.nimu - 1].quat[2];  // y
                    orientation_[3] = raw_.imu[raw_.nimu - 1].quat[3];  // z

                    // Update angular velocity (convert deg/s to rad/s)
                    angular_velocity_[0] = raw_.imu[raw_.nimu - 1].gyr[0] * DEG_TO_RAD;
                    angular_velocity_[1] = raw_.imu[raw_.nimu - 1].gyr[1] * DEG_TO_RAD;
                    angular_velocity_[2] = raw_.imu[raw_.nimu - 1].gyr[2] * DEG_TO_RAD;

                    // Update linear acceleration (convert g to m/s²)
                    linear_acceleration_[0] = raw_.imu[raw_.nimu - 1].acc[0] * GRA_ACC;
                    linear_acceleration_[1] = raw_.imu[raw_.nimu - 1].acc[1] * GRA_ACC;
                    linear_acceleration_[2] = raw_.imu[raw_.nimu - 1].acc[2] * GRA_ACC;
                }
            }
        }

        // Clear buffer
        memset(buf_, 0, sizeof(buf_));
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type IMUHardware::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    // IMU is read-only, no write operations needed
    return hardware_interface::return_type::OK;
}

int IMUHardware::open_serial() {
    struct termios options;

    int fd = open(serial_device_.c_str(), O_RDWR | O_NOCTTY);

    if (fd == -1) {
        RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Unable to open serial port: %s", serial_device_.c_str());
        return -1;
    }

    tcgetattr(fd, &options);

    if (fcntl(fd, F_SETFL, 0) < 0) {
        RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "fcntl failed");
    }

    memset(&options, 0, sizeof(options));

    // Set baud rate based on parameter
    speed_t baud;
    switch (baud_rate_) {
        case 921600:
            baud = B921600;
            break;
        case 460800:
            baud = B460800;
            break;
        case 230400:
            baud = B230400;
            break;
        case 115200:
            baud = B115200;
            break;
        case 57600:
            baud = B57600;
            break;
        case 38400:
            baud = B38400;
            break;
        case 19200:
            baud = B19200;
            break;
        case 9600:
            baud = B9600;
            break;
        default:
            RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "Unsupported baud rate %d, using 921600", baud_rate_);
            baud = B921600;
            break;
    }

    options.c_cflag = baud | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);

    RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Serial port opened successfully: %s at %d baud", serial_device_.c_str(), baud_rate_);

    return fd;
}

}  // namespace sealand_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sealand_control::IMUHardware, hardware_interface::SystemInterface)
