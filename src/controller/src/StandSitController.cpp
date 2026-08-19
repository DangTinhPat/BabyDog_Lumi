#include "controller/StandSitController.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace controller
{
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::InterfaceConfiguration StandSitController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : command_interface_types_)
            {
                if (!command_prefix_.empty())
                {
                    conf.names.push_back(command_prefix_ + "/" + joint_name + "/" += interface_type);
                }
                else
                {
                    conf.names.push_back(joint_name + "/" += interface_type);
                }
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration StandSitController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" += interface_type);
            }
        }

        return conf;
    }

    controller_interface::return_type StandSitController::
    update(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        // Refresh measured q/qd for Jacobian/Tff. Stand/Sit takes a coherent
        // feedback snapshot directly when entering and then runs IK from its
        // last valid command seed. Do not return early while the model is null:
        // Passive and ESTOP must still run; a requested pose waits safely at
        // kp=0 until /robot_description has produced a valid model.
        // std::atomic_load thay vi doc thang robot_model_ - shared_ptr KHONG an toan
        // doc/ghi dong thoi giua nhieu luong (luong RT nay vs luong executor chay
        // callback /robot_description ben duoi, co the chay lai bat ky luc nao neu
        // /robot_description duoc publish lai - transient_local co the redeliver).
        // Da xac nhan qua security review truoc khi ket noi phan cung that.
        if (const auto robot_model_snapshot = std::atomic_load(&robot_model_))
        {
            robot_model_snapshot->update();
        }

        if (mode_ == FSMMode::NORMAL)
        {
            current_state_->run(time, period);
            next_state_name_ = current_state_->checkChange();

            if (next_state_name_ != current_state_->state_name)
            {
                mode_ = FSMMode::CHANGE;
                next_state_ = getNextState(next_state_name_);
                RCLCPP_INFO(get_node()->get_logger(), "Switched from %s to %s",
                            current_state_->state_name_string.c_str(), next_state_->state_name_string.c_str());
            }
        }
        else if (mode_ == FSMMode::CHANGE)
        {
            // StateHoldPose giu Tff qua bien Stand<->Sit de ramp mem. Moi duong
            // sang Passive/ESTOP phai reset blend truoc khi state cu exit.
            if (next_state_name_ == FSMStateName::PASSIVE)
            {
                tau_ff_support_blend_ = 0.0;
            }
            current_state_->exit();
            current_state_ = next_state_;

            current_state_->enter();
            mode_ = FSMMode::NORMAL;
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn StandSitController::on_init()
    {
        try
        {
            joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
            command_interface_types_ =
                auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
            state_interface_types_ =
                auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
            command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);

            sit_foot_positions_ = auto_declare<std::vector<double>>(
                "sit_foot_positions", sit_foot_positions_);
            stand_foot_positions_ = auto_declare<std::vector<double>>(
                "stand_foot_positions", stand_foot_positions_);
            // Kiem tra kich thuoc TRUOC khi bat ky ai index [0..11] (StateHoldPose's
            // constructor khong tu kiem tra) - 1 mang YAML thieu/thua phan tu (loi
            // copy-paste) se gay doc ngoai vung std::vector (UB), dung loai bug da tung
            // gap that voi .value() truoc do. Fail som, ro rang, o day thay vi crash
            // mo ho sau trong StateHoldPose.
            if (stand_foot_positions_.size() != 12 || sit_foot_positions_.size() != 12)
            {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "stand_foot_positions (%zu) / sit_foot_positions (%zu) phai co "
                             "dung 12 gia tri (xyz x 4 chan)",
                             stand_foot_positions_.size(), sit_foot_positions_.size());
                return controller_interface::CallbackReturn::ERROR;
            }
            const auto valid_cartesian_target = [](const std::vector<double>& target) {
                return std::all_of(target.begin(), target.end(), [](const double value) {
                    return std::isfinite(value) && std::abs(value) <= 1.0;
                });
            };
            if (!valid_cartesian_target(stand_foot_positions_) ||
                !valid_cartesian_target(sit_foot_positions_))
            {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "Cartesian foot targets phai huu han va nam trong +/-1 m");
                return controller_interface::CallbackReturn::ERROR;
            }

            stand_kp_ = auto_declare<std::vector<double>>("stand_kp", stand_kp_);
            stand_kd_ = auto_declare<std::vector<double>>("stand_kd", stand_kd_);
            sit_kp_ = auto_declare<std::vector<double>>("sit_kp", sit_kp_);
            sit_kd_ = auto_declare<std::vector<double>>("sit_kd", sit_kd_);
            velocity_max_rad_s_ = auto_declare<std::vector<double>>(
                "velocity_max_rad_s", velocity_max_rad_s_);
            // Gain la hop dong 12-khop: sai kich thuoc phai fail som, khong
            // fallback ve khop[0] hay doc ngoai vector. Firmware con clamp lop
            // 2, nhung EC van phai tu choi NaN/inf/gain am truoc khi publish.
            const auto valid_gain_vector = [](const std::vector<double>& values, const bool strictly_positive) {
                return values.size() == 12 &&
                       std::all_of(values.begin(), values.end(), [strictly_positive](const double value) {
                           return std::isfinite(value) &&
                                  (strictly_positive ? value > 0.0 : value >= 0.0);
                       });
            };
            if (!valid_gain_vector(stand_kp_, true) || !valid_gain_vector(stand_kd_, false) ||
                !valid_gain_vector(sit_kp_, true) || !valid_gain_vector(sit_kd_, false))
            {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "stand/sit Kp/Kd phai la mang 12 gia tri huu han; Kp>0, Kd>=0");
                return controller_interface::CallbackReturn::ERROR;
            }
            if (velocity_max_rad_s_.size() != 12 ||
                !std::all_of(velocity_max_rad_s_.begin(), velocity_max_rad_s_.end(),
                             [](const double value) {
                                 // BabyAlpha2 wire range is +/-45 rad/s, but
                                 // /joint_cmd uses int16 mrad/s. Stay inside
                                 // both representations; shipped config is 5.
                                 return std::isfinite(value) && value > 0.0 && value <= 32.767;
                             }))
            {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "velocity_max_rad_s phai la mang 12 gia tri huu han trong (0,32.767]");
                return controller_interface::CallbackReturn::ERROR;
            }

            stand_duration_ = auto_declare<double>("stand_duration", stand_duration_);
            sit_duration_ = auto_declare<double>("sit_duration", sit_duration_);
            if (!std::isfinite(stand_duration_) || !std::isfinite(sit_duration_) ||
                stand_duration_ <= 0.0 || sit_duration_ <= 0.0)
            {
                RCLCPP_ERROR(get_node()->get_logger(), "stand_duration/sit_duration phai > 0");
                return controller_interface::CallbackReturn::ERROR;
            }

            base_name_ = auto_declare<std::string>("base_name", base_name_);
            feet_names_ = auto_declare<std::vector<std::string>>("feet_names", feet_names_);
            if (feet_names_.size() != 4)
            {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "feet_names phai co dung 4 phan tu (1 moi chan), hien co %zu - xem "
                             "QuadrupedRobot::QuadrupedRobot() (index [0..3] khong tu kiem tra)",
                             feet_names_.size());
                return controller_interface::CallbackReturn::ERROR;
            }
            tau_ff_scale_ = auto_declare<double>("tau_ff_scale", tau_ff_scale_);
            tau_ff_mass_kg_ = auto_declare<double>("tau_ff_mass_kg", tau_ff_mass_kg_);
            tau_ff_ramp_seconds_ = auto_declare<double>(
                "tau_ff_ramp_seconds", tau_ff_ramp_seconds_);
            tau_ff_load_share_ = auto_declare<std::vector<double>>(
                "tau_ff_load_share", tau_ff_load_share_);
            tau_ff_joint_scale_ = auto_declare<std::vector<double>>(
                "tau_ff_joint_scale", tau_ff_joint_scale_);
            tau_ff_diagnostics_enabled_ = auto_declare<bool>(
                "tau_ff_diagnostics_enabled", tau_ff_diagnostics_enabled_);
            tau_ff_diagnostics_start_seconds_ = auto_declare<double>(
                "tau_ff_diagnostics_start_seconds", tau_ff_diagnostics_start_seconds_);
            tau_ff_diagnostics_period_seconds_ = auto_declare<double>(
                "tau_ff_diagnostics_period_seconds", tau_ff_diagnostics_period_seconds_);
            stand_joint_trim_rad_ = auto_declare<std::vector<double>>(
                "stand_joint_trim_rad", stand_joint_trim_rad_);
            sit_joint_trim_rad_ = auto_declare<std::vector<double>>(
                "sit_joint_trim_rad", sit_joint_trim_rad_);
            tau_ff_max_nm_ = auto_declare<std::vector<double>>("tau_ff_max_nm", tau_ff_max_nm_);
            if (!std::isfinite(tau_ff_scale_) || tau_ff_scale_ < 0.0 || tau_ff_scale_ > 1.0)
            {
                RCLCPP_ERROR(get_node()->get_logger(), "tau_ff_scale phai huu han va nam trong [0,1]");
                return controller_interface::CallbackReturn::ERROR;
            }
            if (!std::isfinite(tau_ff_mass_kg_) || tau_ff_mass_kg_ < 0.0 ||
                tau_ff_mass_kg_ > 50.0)
            {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "tau_ff_mass_kg phai huu han va nam trong [0,50] kg; 0 = dung mass URDF");
                return controller_interface::CallbackReturn::ERROR;
            }
            if (!std::isfinite(tau_ff_ramp_seconds_) ||
                tau_ff_ramp_seconds_ < 0.1 || tau_ff_ramp_seconds_ > 10.0)
            {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "tau_ff_ramp_seconds phai huu han va nam trong [0.1,10] giay");
                return controller_interface::CallbackReturn::ERROR;
            }
            const double load_share_sum = std::accumulate(
                tau_ff_load_share_.begin(), tau_ff_load_share_.end(), 0.0);
            if (tau_ff_load_share_.size() != 4 ||
                !std::all_of(tau_ff_load_share_.begin(), tau_ff_load_share_.end(),
                             [](const double value) {
                                 return std::isfinite(value) && value >= 0.0 && value <= 1.0;
                             }) ||
                !std::isfinite(load_share_sum) || std::abs(load_share_sum - 1.0) > 1e-6)
            {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "tau_ff_load_share phai la 4 gia tri trong [0,1] co tong bang 1");
                return controller_interface::CallbackReturn::ERROR;
            }
            if (tau_ff_joint_scale_.size() != 12 ||
                !std::all_of(tau_ff_joint_scale_.begin(), tau_ff_joint_scale_.end(),
                             [](const double value) {
                                 return std::isfinite(value) && value >= 0.0 && value <= 2.0;
                             }))
            {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "tau_ff_joint_scale phai la mang 12 gia tri huu han trong [0,2]");
                return controller_interface::CallbackReturn::ERROR;
            }
            if (!std::isfinite(tau_ff_diagnostics_start_seconds_) ||
                tau_ff_diagnostics_start_seconds_ < 0.0 ||
                tau_ff_diagnostics_start_seconds_ > 60.0 ||
                !std::isfinite(tau_ff_diagnostics_period_seconds_) ||
                tau_ff_diagnostics_period_seconds_ <= 0.0 ||
                tau_ff_diagnostics_period_seconds_ > 60.0)
            {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "tau_ff_diagnostics_start_seconds phai trong [0,60], "
                             "tau_ff_diagnostics_period_seconds phai trong (0,60]");
                return controller_interface::CallbackReturn::ERROR;
            }
            const auto valid_trim_vector = [](const std::vector<double>& values) {
                return values.size() == 12 &&
                       std::all_of(values.begin(), values.end(), [](const double value) {
                           return std::isfinite(value) && std::abs(value) <= 0.35;
                       });
            };
            if (!valid_trim_vector(stand_joint_trim_rad_) || !valid_trim_vector(sit_joint_trim_rad_))
            {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "stand/sit_joint_trim_rad phai la mang 12 gia tri huu han trong [-0.35,0.35] rad");
                return controller_interface::CallbackReturn::ERROR;
            }
            if (tau_ff_max_nm_.size() != 12 ||
                !std::all_of(tau_ff_max_nm_.begin(), tau_ff_max_nm_.end(), [](const double value) {
                    // BabyAlpha2 ma hoa duoc +/-24 N.m, nhung gioi han phan
                    // cung du an dang tune la +/-10 N.m. EC khong cho phep
                    // config vuot tran nay; RealSystem va MCU con kep lai.
                    return std::isfinite(value) && value >= 0.0 && value <= 10.0;
                }))
            {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "tau_ff_max_nm phai la mang 12 gia tri huu han trong [0,10] N.m");
                return controller_interface::CallbackReturn::ERROR;
            }

            get_node()->get_parameter("update_rate", ctrl_interfaces_.frequency_);
            RCLCPP_INFO(get_node()->get_logger(), "Controller Manager Update Rate: %d Hz",
                        ctrl_interfaces_.frequency_);
        }
        catch (const std::exception& e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn StandSitController::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        control_input_subscription_ = get_node()->create_subscription<controller::msg::Inputs>(
            "/control_input", 10, [this](const controller::msg::Inputs::SharedPtr msg)
            {
                ctrl_interfaces_.control_inputs_.command = msg->command;
            });

        // FK/IK model - nap ngay khi co /robot_description (transient_local
        // nen se nhan duoc ban tin da publish truoc do neu robot_state_publisher da
        // chay roi). Neu con null luc vua khoi dong, Stand/Sit giu kp=0 va cho;
        // Passive/ESTOP van hoat dong binh thuong.
        robot_description_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                // QuadrupedRobot() co the nem std::runtime_error (base_name/feet_names sai
                // ten link trong URDF, xem QuadrupedRobot.cpp) - bat lai o day thay vi de
                // executor thread crash: log ro rang, robot_model_ giu nguyen (null hoac ban
                // cu con hoat dong duoc). Neu khong co model hop le, Stand/Sit se
                // khong cap lenh vi tri (kp=0), thay vi phat ket qua IK khong tin cay.
                try
                {
                    auto new_model =
                        std::make_shared<QuadrupedRobot>(ctrl_interfaces_, msg->data, feet_names_, base_name_);
                    RCLCPP_INFO(get_node()->get_logger(), "QuadrupedRobot loaded: mass_=%.3f kg (base=%s)",
                                new_model->mass_, base_name_.c_str());
                    // atomic_store - ghi con tro CHI SAU KHI QuadrupedRobot da xay dung
                    // xong hoan toan (new_model da la doi tuong hop le) - luong RT (update())
                    // va StateHoldPose::run() doc qua atomic_load se KHONG BAO GIO thay 1
                    // con tro dang xay dung do (torn read).
                    std::atomic_store(&robot_model_, new_model);
                }
                catch (const std::exception& e)
                {
                    RCLCPP_ERROR(get_node()->get_logger(),
                                 "Khong nap duoc QuadrupedRobot tu /robot_description: %s - "
                                 "Stand/Sit se cho an toan voi kp=0",
                                 e.what());
                }
            });

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    StandSitController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // clear out vectors in case of restart
        ctrl_interfaces_.clear();

        // assign command interfaces
        for (auto& interface : command_interfaces_)
        {
            std::string interface_name = interface.get_interface_name();
            if (const size_t pos = interface_name.find('/'); pos != std::string::npos)
            {
                command_interface_map_[interface_name.substr(pos + 1)]->push_back(interface);
            }
            else
            {
                command_interface_map_[interface_name]->push_back(interface);
            }
        }

        // assign state interfaces
        for (auto& interface : state_interfaces_)
        {
            state_interface_map_[interface.get_interface_name()]->push_back(interface);
        }

        RCLCPP_INFO(
            get_node()->get_logger(),
            "Command interfaces mapped: position=%zu velocity=%zu kp=%zu kd=%zu effort/Tff=%zu",
            ctrl_interfaces_.joint_position_command_interface_.size(),
            ctrl_interfaces_.joint_velocity_command_interface_.size(),
            ctrl_interfaces_.joint_kp_command_interface_.size(),
            ctrl_interfaces_.joint_kd_command_interface_.size(),
            ctrl_interfaces_.joint_torque_command_interface_.size());
        if (tau_ff_scale_ > 0.0 && ctrl_interfaces_.joint_torque_command_interface_.size() != 12)
        {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "tau_ff_scale>0 nhung effort/Tff command interface khong du 12");
            return CallbackReturn::ERROR;
        }

        // Create FSM List
        state_list_.passive = std::make_shared<StatePassive>(ctrl_interfaces_);
        tau_ff_support_blend_ = 0.0;
        state_list_.stand = std::make_shared<StateHoldPose>(
            ctrl_interfaces_, FSMStateName::STAND, "stand", stand_foot_positions_, stand_kp_, stand_kd_,
            velocity_max_rad_s_, stand_duration_,
            robot_model_, tau_ff_scale_, tau_ff_mass_kg_, tau_ff_ramp_seconds_, tau_ff_load_share_,
            tau_ff_joint_scale_, tau_ff_diagnostics_enabled_,
            tau_ff_diagnostics_start_seconds_, tau_ff_diagnostics_period_seconds_,
            stand_joint_trim_rad_, tau_ff_support_blend_, tau_ff_max_nm_);
        state_list_.sit = std::make_shared<StateHoldPose>(
            ctrl_interfaces_, FSMStateName::SIT, "sit", sit_foot_positions_, sit_kp_, sit_kd_,
            velocity_max_rad_s_, sit_duration_,
            robot_model_, tau_ff_scale_, tau_ff_mass_kg_, tau_ff_ramp_seconds_, tau_ff_load_share_,
            tau_ff_joint_scale_, tau_ff_diagnostics_enabled_,
            tau_ff_diagnostics_start_seconds_, tau_ff_diagnostics_period_seconds_,
            sit_joint_trim_rad_, tau_ff_support_blend_, tau_ff_max_nm_);

        // Initialize FSM
        current_state_ = state_list_.passive;
        current_state_->enter();
        next_state_ = current_state_;
        next_state_name_ = current_state_->state_name;
        mode_ = FSMMode::NORMAL;

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn StandSitController::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // Ghi mot lenh Passive cuoi cung TRUOC KHI nha loaned interfaces. Neu
        // chi release, cac backing value trong RealSystem co the van giu Kp/Tff
        // cua Stand/Sit va tiep tuc duoc publish trong cac chu ky hardware sau.
        // Reset support blend roi Passive::enter() xoa toan bo effort/Kp/Kd cho
        // ca 12 khop.
        if (current_state_)
        {
            tau_ff_support_blend_ = 0.0;
            current_state_->exit();
        }
        if (state_list_.passive)
        {
            state_list_.passive->enter();
        }
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    StandSitController::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    StandSitController::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    StandSitController::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    std::shared_ptr<FSMState> StandSitController::getNextState(const FSMStateName stateName) const
    {
        switch (stateName)
        {
        case FSMStateName::INVALID:
            return state_list_.invalid;
        case FSMStateName::PASSIVE:
            return state_list_.passive;
        case FSMStateName::STAND:
            return state_list_.stand;
        case FSMStateName::SIT:
            return state_list_.sit;
        default:
            return state_list_.invalid;
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(controller::StandSitController, controller_interface::ControllerInterface);
