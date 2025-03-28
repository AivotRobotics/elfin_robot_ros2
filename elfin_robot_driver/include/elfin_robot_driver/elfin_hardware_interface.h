#pragma once
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>

#include <hardware_interface/system_interface.hpp>

namespace elfin_robot_driver {

    class ElfinHardwareInterface final :  public hardware_interface::SystemInterface {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(ElfinHardwareInterface)

        ElfinHardwareInterface();
        ~ElfinHardwareInterface() override = default;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>&, const std::vector<std::string>&) override;
        hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>&, const std::vector<std::string>&) override;

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        std::string get_hardware_param(const std::string & param_name, const std::string & default_value) const {
            return info_.hardware_parameters.find(param_name) != info_.hardware_parameters.end() ? info_.hardware_parameters.at(param_name) : default_value;
        }

        unsigned int get_hardware_param(const std::string & param_name, const unsigned int default_value) const {
            return std::stoi(get_hardware_param(param_name, std::to_string(default_value)));
        }

        double get_hardware_param(const std::string & param_name, const double default_value) const {
            return std::stod(get_hardware_param(param_name, std::to_string(default_value)));
        }

    private:
        static constexpr unsigned char NUM_JOINTS = 6;
        using array6d = std::array<double, NUM_JOINTS>;

        rclcpp::Logger logger_;

        unsigned int box_id_ = 0;
        unsigned int robot_id_ = 0;

        double servo_update_cycle_ = 0.02;
        double servo_lookahead_time_ = 0.0;

        array6d hw_cmd_position_;
        array6d hw_state_position_;
        array6d hw_state_velocity_;

        bool position_intf_claimed_ = false;
        bool position_ctrl_running_ = false;


        bool first_pass_ = true;
        bool initialized_ = false;
        bool servoing_ = false;
    };
}
