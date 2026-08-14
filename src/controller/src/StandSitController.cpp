#include "controller/StandSitController.h"

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
        // Cap nhat cache vi tri/van toc cho FK/IK (Tff, xem StateHoldPose::run()) -
        // KHONG return som neu con null (chi tat Tff, KHONG duoc chan Passive/Stand/
        // Sit chay - khac superDog's UnitreeGuideController co
        // "if (robot_model_ == nullptr) return OK;" chan toan bo FSM, khong phu hop
        // o day vi core function (giu tu the) phai luon chay du /robot_description
        // co toi kip hay khong).
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

            sit_pos_ = auto_declare<std::vector<double>>("sit_pos", sit_pos_);
            stand_pos_ = auto_declare<std::vector<double>>("stand_pos", stand_pos_);
            // Kiem tra kich thuoc TRUOC khi bat ky ai index [0..11] (StateHoldPose's
            // constructor khong tu kiem tra) - 1 mang YAML thieu/thua phan tu (loi
            // copy-paste) se gay doc ngoai vung std::vector (UB), dung loai bug da tung
            // gap that voi .value() truoc do. Fail som, ro rang, o day thay vi crash
            // mo ho sau trong StateHoldPose.
            if (stand_pos_.size() != 12 || sit_pos_.size() != 12)
            {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "stand_pos (%zu phan tu) / sit_pos (%zu phan tu) phai co dung 12 "
                             "(1 moi khop) - kiem tra lai controllers.yaml/controllers_real.yaml",
                             stand_pos_.size(), sit_pos_.size());
                return controller_interface::CallbackReturn::ERROR;
            }

            stand_kp_ = auto_declare<double>("stand_kp", stand_kp_);
            stand_kd_ = auto_declare<double>("stand_kd", stand_kd_);
            sit_kp_ = auto_declare<double>("sit_kp", sit_kp_);
            sit_kd_ = auto_declare<double>("sit_kd", sit_kd_);
            // kp<=0 -> PD khong con luc phuc hoi ve target (hoac day nguoc huong neu
            // am) - kd<0 tuong tu khong hop ly ve vat ly cho khop dan dong PD cuc bo.
            // Chan som truoc khi gia tri nay toi tay dong co that.
            if (stand_kp_ <= 0.0 || stand_kd_ < 0.0 || sit_kp_ <= 0.0 || sit_kd_ < 0.0)
            {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "kp phai > 0, kd phai >= 0 (stand_kp=%.3f stand_kd=%.3f sit_kp=%.3f "
                             "sit_kd=%.3f) - gia tri am/khong se khien PD khong giu duoc tu the "
                             "hoac day khop sai huong",
                             stand_kp_, stand_kd_, sit_kp_, sit_kd_);
                return controller_interface::CallbackReturn::ERROR;
            }

            stand_duration_ = auto_declare<double>("stand_duration", stand_duration_);
            sit_duration_ = auto_declare<double>("sit_duration", sit_duration_);

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
            tau_ff_max_nm_ = auto_declare<double>("tau_ff_max_nm", tau_ff_max_nm_);

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

        // FK/IK (Tff) - nap robot_model_ ngay khi co /robot_description (transient_local
        // nen se nhan duoc ban tin da publish truoc do neu robot_state_publisher da
        // chay roi). CO THE con null 1 luc dau neu chua toi kip - KHONG sao, xem
        // update() (khong chan FSM, chi tat Tff tam thoi).
        robot_description_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                // QuadrupedRobot() co the nem std::runtime_error (base_name/feet_names sai
                // ten link trong URDF, xem QuadrupedRobot.cpp) - bat lai o day thay vi de
                // executor thread crash: log ro rang, robot_model_ giu nguyen (null hoac ban
                // cu con hoat dong duoc) - Tff tu dong tat/giu trang thai cu, KHONG anh huong
                // Passive/Stand/Sit (core function).
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
                                 "Khong nap duoc QuadrupedRobot tu /robot_description: %s - Tff se "
                                 "khong hoat dong (Passive/Stand/Sit khong bi anh huong)",
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

        // Create FSM List
        state_list_.passive = std::make_shared<StatePassive>(ctrl_interfaces_);
        state_list_.stand = std::make_shared<StateHoldPose>(
            ctrl_interfaces_, FSMStateName::STAND, "stand", stand_pos_, stand_kp_, stand_kd_, stand_duration_,
            robot_model_, tau_ff_scale_, tau_ff_max_nm_);
        state_list_.sit = std::make_shared<StateHoldPose>(
            ctrl_interfaces_, FSMStateName::SIT, "sit", sit_pos_, sit_kp_, sit_kd_, sit_duration_,
            robot_model_, tau_ff_scale_, tau_ff_max_nm_);

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
