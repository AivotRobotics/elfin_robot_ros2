#include <cmath>
#include <iterator>
#include <thread>

#include "rclcpp/logging.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "elfin_sdk/HR_Pro.h"
#include "elfin_robot_driver/elfin_hardware_interface.h"

namespace elfin_robot_driver {

    constexpr auto deg2rad = [](const double d) { return d * M_PI / 180.0; };
    constexpr auto rad2deg = [](const double r) { return r * 180.0 / M_PI; };

    ElfinHardwareInterface::ElfinHardwareInterface()
        : logger_(rclcpp::get_logger(__func__))
    {
    }

    std::vector<hardware_interface::StateInterface> ElfinHardwareInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (auto i = 0u; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_state_position_[i]);
            state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocity_[i]);
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ElfinHardwareInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (auto i = 0u; i <  info_.joints.size(); i++) {
            command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_cmd_position_[i]);
        }

        return command_interfaces;
    }

    hardware_interface::return_type ElfinHardwareInterface::prepare_command_mode_switch(const std::vector<std::string>& start, const std::vector<std::string>& stop) {

        auto start_intfs = std::vector<std::string>();
        auto stop_intfs = std::vector<std::string>();

        const auto is_hw_if_pos = [] (const std::string & intf) {
            return intf.find(hardware_interface::HW_IF_POSITION) != std::string::npos;
        };

        const auto is_intf_own = [&] (const std::string & intf) {
            return std::find_if(info_.joints.begin(), info_.joints.end(), [&intf] (const auto & jnt_info) {
                            return intf == jnt_info.name;
                        }) != info_.joints.end();
        };

        std::copy_if(start.begin(), start.end(), std::back_inserter(start_intfs), is_intf_own);
        std::copy_if(stop.begin(), stop.end(), std::back_inserter(stop_intfs), is_intf_own);

        const auto num_stop_intfs = std::count_if(stop_intfs.begin(), stop_intfs.end(), is_hw_if_pos);
        if (num_stop_intfs > 0) {
            if (num_stop_intfs != NUM_JOINTS) {
                RCLCPP_FATAL(logger_, "Expected %d position interfaces to stop, but god %ld instead.", NUM_JOINTS, num_stop_intfs);
                return hardware_interface::return_type::ERROR;
            }

            position_intf_claimed_ = false;
        }

        const auto num_start_intfs = std::count_if(start_intfs.begin(), start_intfs.end(), is_hw_if_pos);
        if (num_start_intfs > 0) {
            if (num_start_intfs != NUM_JOINTS) {
                RCLCPP_FATAL(logger_, "Expected %d position interfaces to start, but god %ld instead.", NUM_JOINTS, num_stop_intfs);
                return hardware_interface::return_type::ERROR;
            }

            position_intf_claimed_ = true;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ElfinHardwareInterface::perform_command_mode_switch(const std::vector<std::string>& start, const std::vector<std::string>& stop) {

        if(position_intf_claimed_ &&  !position_ctrl_running_) {
            position_ctrl_running_ = true;
        } else if(!position_intf_claimed_ && position_ctrl_running_) {
            position_ctrl_running_ = false;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::CallbackReturn ElfinHardwareInterface::on_init(const hardware_interface::HardwareInfo & info) {
        if (const auto ret = SystemInterface::on_init(info); ret != hardware_interface::CallbackReturn::SUCCESS) {
            return ret;
        }

        logger_ = rclcpp::get_logger(get_name());

        if (const auto num_joints = info.joints.size(); num_joints != NUM_JOINTS) {
            RCLCPP_ERROR(logger_, "Invalid number of joints %lu, expected %u", num_joints, NUM_JOINTS);
            return hardware_interface::CallbackReturn::ERROR;
        }

        for (const hardware_interface::ComponentInfo& joint : info.joints) {
            if (joint.command_interfaces.size() != 1) {
                RCLCPP_FATAL(logger_, "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(), joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(logger_, "Joint '%s' has %s command interface. '%s' expected.",
                           joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2) {
                RCLCPP_FATAL(logger_, "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(), joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(logger_, "Joint '%s' has %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                           joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(logger_,
                           "Joint '%s' has %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
                           joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        box_id_ = get_hardware_param("box_id", box_id_);
        robot_id_ = get_hardware_param("robot_id", robot_id_);

        servo_update_cycle_ = get_hardware_param("servo_update_cycle", servo_update_cycle_);
        servo_lookahead_time_ = get_hardware_param("servo_lookahead_time", servo_lookahead_time_);

        hw_state_position_ = { { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 } };
        hw_state_velocity_ = { { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 } };
        hw_cmd_position_ = { { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 } };

        first_pass_ = true;
        initialized_ = false;

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ElfinHardwareInterface::on_configure(const rclcpp_lifecycle::State & previous_state) {
        const auto robot_ip = get_hardware_param("robot_ip", "");
        if (robot_ip.empty()) {
            RCLCPP_FATAL(logger_, "No robot IP specified.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        auto robot_port = 10003;
        if (const auto pos = robot_ip.find(':'); pos != std::string::npos) {
            robot_port = std::stoi(robot_ip.substr(pos + 1));
        }

        auto err = HRIF_SetLogDbgCB(box_id_, [](int logLevel, const string& strLog, void* arg) {
            RCLCPP_INFO(*(static_cast<rclcpp::Logger*>(arg)), "HRIF [%d] %s", logLevel, strLog.c_str());
        }, &logger_);
        RCLCPP_WARN_EXPRESSION(logger_, err, "Failed to set log callback, ERR: %d", err);

        err = HRIF_Connect(box_id_, robot_ip.c_str(), robot_port);
        if (err) {
            RCLCPP_ERROR(logger_, "Failed to connect to box %d at %s:%d, ERR: %d", box_id_, robot_ip.c_str(), robot_port, err);
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(logger_, "Connected to robot at %s:%s as box %d", robot_ip.c_str(), std::to_string(robot_port).c_str(), box_id_);

        std::string model, version;
        int cps_ver, codesys_ver, major_ver, minor_ver, min_ver, algo_ver, firmware_ver;
        HRIF_ReadRobotModel(box_id_, model);
        HRIF_ReadVersion(box_id_, robot_id_, version, cps_ver, codesys_ver, major_ver, minor_ver, min_ver, algo_ver, firmware_ver);
        RCLCPP_INFO(logger_, "Model: %s, Version: %s", model.c_str(), version.c_str());

        int started;
        err = HRIF_IsControllerStarted(box_id_, started);
        RCLCPP_WARN_EXPRESSION(logger_, !err && started == 0, "Controller is not started");

        int is_moving, is_enabled, is_in_error, error_code, error_axis, is_breaking, is_paused, is_in_emergency_stop, is_in_safety_guard, is_electrified, is_connected_to_box, is_blending_done, is_in_place;
        err = HRIF_ReadRobotState(box_id_,robot_id_, is_moving, is_enabled, is_in_error, error_code, error_axis, is_breaking,
            is_paused, is_in_emergency_stop, is_in_safety_guard, is_electrified, is_connected_to_box, is_blending_done, is_in_place);
        if (err) {
            RCLCPP_FATAL(logger_, "Failed to read robot state, ERR: %d", err);
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(logger_, "Enabled: %d, Break: %d, Paused: %d, Emergency: %d, Safety: %d, Electrified: %d, Connected: %d",
            is_enabled, is_breaking, is_paused, is_in_emergency_stop, is_in_safety_guard, is_electrified, is_connected_to_box);
        RCLCPP_WARN_EXPRESSION(logger_, is_in_error, "Robot is in error state: axis = %d, code = %d", error_axis, error_code);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ElfinHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ElfinHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ElfinHardwareInterface::on_shutdown(const rclcpp_lifecycle::State & previous_state) {
        if (const auto err = HRIF_DisConnect(box_id_)) {
            RCLCPP_ERROR(logger_, "Failed to disconnect from box %d, ERR: %d", box_id_, err);
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(logger_, "Disconnected from box %d", box_id_);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type ElfinHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period) {

        int is_moving, is_enabled, is_in_error, error_code, error_axis, is_breaking, is_paused, is_blending_done;
        auto err = HRIF_ReadRobotFlags(box_id_, robot_id_, is_moving, is_enabled, is_in_error, error_code, error_axis, is_breaking, is_paused, is_blending_done);
        if (err) {
            RCLCPP_ERROR(logger_, "Failed to read robot flags, ERR: %d", err);
            return hardware_interface::return_type::ERROR;
        }

        array6d joint_pos;
        err = HRIF_ReadActJointPos(box_id_, robot_id_, joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3], joint_pos[4], joint_pos[5]);
        if (err) {
            RCLCPP_ERROR(logger_, "Failed to read joint positions, ERR: %d", err);
            return hardware_interface::return_type::ERROR;
        }
        std::transform(joint_pos.begin(), joint_pos.end(), hw_state_position_.begin(), deg2rad);

        err = HRIF_ReadActJointVel(box_id_, robot_id_, hw_state_velocity_[0], hw_state_velocity_[1], hw_state_velocity_[2], hw_state_velocity_[3],hw_state_velocity_[4],hw_state_velocity_[5]);
        if (err) {
            RCLCPP_ERROR(logger_, "Failed to read joint velocities, ERR: %d", err);
            return hardware_interface::return_type::ERROR;
        }

        if (first_pass_ && !initialized_) {
            // initialize commands
            hw_cmd_position_ = hw_state_position_;
            initialized_ = true;
        }

        RCLCPP_DEBUG(logger_, "READ: [%s]", [&] {
            std::ostringstream joints;
            std::copy(joint_pos.begin(), joint_pos.end(), std::ostream_iterator<double>(joints, ", "));
            return joints.str();
        }().c_str());

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ElfinHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
        const auto current_update_cycle = period.seconds();
        if ((current_update_cycle - servo_update_cycle_) > 0.010) {
            RCLCPP_WARN(logger_, "Update period is higher (%.6f) than servo update cycle (%.6f)", current_update_cycle, servo_update_cycle_);
        }

        // start servoing right before the first write, HRIF is sensitive to delay between StartServo and PushServo
        if (!servoing_) {
            if (const auto err = HRIF_StartServo(box_id_, robot_id_, servo_update_cycle_, servo_lookahead_time_)) {
                RCLCPP_FATAL(logger_, "Failed to start servo, ERR: %d", err);
                return hardware_interface::return_type::ERROR;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            servoing_ = true;
        }

        array6d joint_pos;
        std::transform(hw_cmd_position_.begin(), hw_cmd_position_.end(), joint_pos.begin(),rad2deg);

        RCLCPP_DEBUG(logger_, "WRITE: [%s]", [&] {
            std::ostringstream joints;
            std::copy(joint_pos.begin(), joint_pos.end(), std::ostream_iterator<double>(joints, ", "));
            return joints.str();
        }().c_str());

        if (const auto err = HRIF_PushServoJ(box_id_, robot_id_, joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3], joint_pos[4], joint_pos[5])) {
            RCLCPP_FATAL(logger_, "Failed to push servo command, ERR: %d", err);
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(elfin_robot_driver::ElfinHardwareInterface, hardware_interface::SystemInterface)
