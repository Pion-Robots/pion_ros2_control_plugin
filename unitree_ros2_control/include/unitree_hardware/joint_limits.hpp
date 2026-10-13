/*
Copyright (c) 2024 Kazuya Oguma. All rights reserved.
Licensed under the MIT license. See LICENSE file in the project root for details.
*/

#include <urdf/model.h>

#include <joint_limits/joint_limits.hpp>
#include <rclcpp/macros.hpp>
#include <string>
#include <utility>
#include <vector>

namespace joint_limits_interface {

// Lightweight replacement for hardware_interface::ReadWriteHandle, which was removed after Humble.
class JointHandle {
   public:
    JointHandle(std::string prefix_name, std::string interface_name, double* value_ptr)
        : prefix_name_(std::move(prefix_name)), interface_name_(std::move(interface_name)), value_ptr_(value_ptr) {}

    explicit operator bool() const { return value_ptr_ != nullptr; }
    const std::string& get_prefix_name() const { return prefix_name_; }
    const std::string& get_interface_name() const { return interface_name_; }
    double get_value() const { return *value_ptr_; }
    void set_value(double value) { *value_ptr_ = value; }

   private:
    std::string prefix_name_;
    std::string interface_name_;
    double* value_ptr_;
};

class JointLimits {
   public:
    RCLCPP_SHARED_PTR_DEFINITIONS(JointLimits)

    bool load(const std::string& urdf_xml);
    bool configure(const std::vector<JointHandle>& joint_handles);
    void update();

    double get_min_position() const;
    double get_max_position() const;
    double get_max_velocity() const;
    double get_max_effort() const;

   private:
    JointHandle get_by_interface(const std::vector<JointHandle>& handles, const std::string& interface_name);
    bool are_names_identical(const std::vector<JointHandle>& handles);

    urdf::ModelSharedPtr model_;
    joint_limits::JointLimits joint_limits_;
    JointHandle joint_position_ = {"", "", nullptr};
    JointHandle joint_velocity_ = {"", "", nullptr};
    JointHandle joint_effort_ = {"", "", nullptr};
};

}  // namespace joint_limits_interface
