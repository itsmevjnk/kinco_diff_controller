#pragma once

#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "kinco_diff_controller/motor_interface.hpp"
#include "kinco_diff_controller/motor.hpp"

namespace kinco_diff_controller {
    class DrivetrainSystemHardware : public hardware_interface::SystemInterface {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(DrivetrainSystemHardware)

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        rclcpp::Logger get_logger() const { return *logger_; }
        rclcpp::Clock::SharedPtr get_clock() const { return clock_; }
    private:
        std::string port_;
        uint8_t left_id_, right_id_;
        double max_speed_;
        bool reverse_, reverse_right_;

        std::shared_ptr<rclcpp::Logger> logger_;
        rclcpp::Clock::SharedPtr clock_;

        MotorInterface* interface_ = nullptr;
        Motor* left_motor_         = nullptr;
        Motor* right_motor_        = nullptr;

        double left_position_  = std::numeric_limits<double>::quiet_NaN();
        double right_position_ = std::numeric_limits<double>::quiet_NaN();
        double left_velocity_  = std::numeric_limits<double>::quiet_NaN();
        double right_velocity_ = std::numeric_limits<double>::quiet_NaN();
        double left_command_   = std::numeric_limits<double>::quiet_NaN();
        double right_command_  = std::numeric_limits<double>::quiet_NaN();
    };
}