#include "kinco_diff_controller/drivetrain_system.hpp"

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"

#include <algorithm>

namespace kinco_diff_controller {
    hardware_interface::CallbackReturn DrivetrainSystemHardware::on_init(const hardware_interface::HardwareInfo& info) {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
            return hardware_interface::CallbackReturn::ERROR;

        logger_ = std::make_shared<rclcpp::Logger>(
            rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.Drivetrain")
        );
        clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

        port_ = info_.hardware_parameters["port"];
        left_id_ = hardware_interface::stod(info_.hardware_parameters["left_id"]);
        right_id_ = hardware_interface::stod(info_.hardware_parameters["right_id"]);
        max_speed_ = hardware_interface::stod(info_.hardware_parameters["max_speed"]);
        reverse_ = hardware_interface::stod(info_.hardware_parameters["reverse"]);
        reverse_right_ = hardware_interface::stod(info_.hardware_parameters["reverse_right"]);

        for (const hardware_interface::ComponentInfo& joint : info_.joints) {
            if (joint.command_interfaces.size() != 1) {
                RCLCPP_FATAL(
                    get_logger(), "joint '%s' has %zu command interfaces found (1 expected)",
                    joint.name.c_str(), joint.command_interfaces.size()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(
                    get_logger(), "joint '%s' has %s command interfaces found (%s expected)",
                    joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY           
                );
                return hardware_interface::CallbackReturn::ERROR;
            }
            
            if (joint.state_interfaces.size() != 2) {
                RCLCPP_FATAL(
                    get_logger(), "joint '%s' has %zu state interface(s) found (2 expected)",
                    joint.name.c_str(), joint.state_interfaces.size()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(
                    get_logger(), "joint '%s' state interface 0 is %s (%s expected)",
                    joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION           
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(
                    get_logger(), "joint '%s' state interface 1 is %s (%s expected)",
                    joint.name.c_str(), joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY           
                );
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> DrivetrainSystemHardware::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> interfaces;
        
        for (const hardware_interface::ComponentInfo& joint : info_.joints) {
            double* pos = nullptr;
            double* vel = nullptr;
            if (joint.name.find("left") != std::string::npos) {
                RCLCPP_DEBUG(get_logger(), "joint '%s' corresponds to left motor", joint.name.c_str());
                pos = &left_position_; vel = &left_velocity_;
            } else if (joint.name.find("right") != std::string::npos) {
                RCLCPP_DEBUG(get_logger(), "joint '%s' corresponds to right motor", joint.name.c_str());
                pos = &right_position_; vel = &right_velocity_;
            } else {
                RCLCPP_ERROR(get_logger(), "joint '%s' does not correspond to any drivetrain motor, skipping", joint.name.c_str());
                continue;
            }

            interfaces.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_POSITION, pos));
            interfaces.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_VELOCITY, vel));
        }

        return interfaces;
    }

    std::vector<hardware_interface::CommandInterface> DrivetrainSystemHardware::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> interfaces;
        
        for (const hardware_interface::ComponentInfo& joint : info_.joints) {
            double* cmd = nullptr;
            if (joint.name.find("left") != std::string::npos) {
                RCLCPP_DEBUG(get_logger(), "joint '%s' corresponds to left motor", joint.name.c_str());
                cmd = &left_command_;
            } else if (joint.name.find("right") != std::string::npos) {
                RCLCPP_DEBUG(get_logger(), "joint '%s' corresponds to right motor", joint.name.c_str());
                cmd = &right_command_;
            } else {
                RCLCPP_ERROR(get_logger(), "joint '%s' does not correspond to any drivetrain motor, skipping", joint.name.c_str());
                continue;
            }

            interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_VELOCITY, cmd));
        }

        return interfaces;
    }

    hardware_interface::CallbackReturn DrivetrainSystemHardware::on_configure(const rclcpp_lifecycle::State& /* previous_state*/) {
        RCLCPP_INFO(get_logger(), "starting motor interface on port %s", port_.c_str());
        interface_ = new MotorInterface(port_.c_str());
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DrivetrainSystemHardware::on_activate(const rclcpp_lifecycle::State& /* previous_state*/) {
        RCLCPP_INFO(get_logger(), "initialising left motor with ID %u", left_id_);
        left_motor_ = new Motor(*interface_, left_id_);
        left_position_ = left_motor_->GetPosition() * 2 * M_PI; // position in radians
        left_velocity_ = left_motor_->GetVelocity() * 2 * M_PI / 60; // velocity in radians per second
        left_command_ = 0.0;

        RCLCPP_INFO(get_logger(), "initialising right motor with ID %u", right_id_);
        right_motor_ = new Motor(*interface_, right_id_);
        right_position_ = right_motor_->GetPosition() * 2 * M_PI;
        right_velocity_ = right_motor_->GetVelocity() * 2 * M_PI / 60;
        right_command_ = 0.0;
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DrivetrainSystemHardware::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) {
        RCLCPP_INFO(get_logger(), "deinitialising left motor"); delete left_motor_; left_motor_ = nullptr;
        RCLCPP_INFO(get_logger(), "deinitialising right motor"); delete right_motor_; right_motor_ = nullptr;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type DrivetrainSystemHardware::read(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
        left_position_ = left_motor_->GetPosition() * 2 * M_PI; // position in radians
        left_velocity_ = left_motor_->GetVelocity() * 2 * M_PI / 60; // velocity in radians per second

        right_position_ = right_motor_->GetPosition() * 2 * M_PI;
        right_velocity_ = right_motor_->GetVelocity() * 2 * M_PI / 60;

        if (reverse_) {
            left_position_ = -left_position_;
            left_velocity_ = -left_velocity_;
            right_position_ = -right_position_;
            right_velocity_ = -right_velocity_;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type DrivetrainSystemHardware::write(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
        // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "velocity: left %llf, right %llf (rad/s)", left_command_, right_command_);
        double left_cmd = isnanl(left_command_) ? 0.0 : std::clamp((left_command_ * 60 / (2 * M_PI)), -max_speed_, max_speed_);
        if (reverse_) left_cmd = -left_cmd;
        if (!left_motor_->SetVelocity(left_cmd)) { // convert from radians per second to rpm
            RCLCPP_ERROR(get_logger(), "cannot set left motor velocity");
            return hardware_interface::return_type::ERROR;
        }
        
        double right_cmd = isnanl(right_command_) ? 0.0 : std::clamp((right_command_ * 60 / (2 * M_PI)), -max_speed_, max_speed_);
        if (reverse_) right_cmd = -right_cmd;
        if (reverse_right_) right_cmd = -right_cmd;
        if (!right_motor_->SetVelocity(right_cmd)) {
            RCLCPP_ERROR(get_logger(), "cannot set right motor velocity");
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(kinco_diff_controller::DrivetrainSystemHardware, hardware_interface::SystemInterface)