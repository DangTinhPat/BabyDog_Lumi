#ifndef CONTROLLER_STANDSITCONTROLLER_H
#define CONTROLLER_STANDSITCONTROLLER_H

#include <controller_interface/controller_interface.hpp>
#include <controller/FSM/FSMState.h>
#include <controller/FSM/StatePassive.h>
#include <controller/FSM/StateHoldPose.h>
#include <controller/common/enumClass.h>
#include <controller/robot/QuadrupedRobot.h>
#include <std_msgs/msg/string.hpp>

namespace controller
{
    struct FSMStateList
    {
        std::shared_ptr<FSMState> invalid;
        std::shared_ptr<StatePassive> passive;
        std::shared_ptr<StateHoldPose> stand;
        std::shared_ptr<StateHoldPose> sit;
    };

    // Trimmed fork of superDog's UnitreeGuideController: only Passive/Stand/Sit
    // (no gait, no WaveGenerator/Estimator/BalanceCtrl QP). Filtered sensing is
    // available on /imu/data, but active IMU correction is intentionally not
    // part of this Stand/Sit controller yet.
    class StandSitController final : public controller_interface::ControllerInterface
    {
    public:
        StandSitController() = default;

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::return_type update(
            const rclcpp::Time& time, const rclcpp::Duration& period) override;

        controller_interface::CallbackReturn on_init() override;

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State& previous_state) override;

        CtrlInterfaces ctrl_interfaces_;

    protected:
        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;
        std::string command_prefix_;

        // Foot targets in body frame, ordered FR/FL/RR/RL and xyz per foot.
        // IK generates joint commands online; these are not joint angles.
        std::vector<double> stand_foot_positions_ = {
             0.176, -0.1295, -0.22,
             0.176,  0.1295, -0.22,
            -0.176, -0.1295, -0.22,
            -0.176,  0.1295, -0.22
        };
        std::vector<double> sit_foot_positions_ = {
             0.176, -0.1295, -0.10,
             0.176,  0.1295, -0.10,
            -0.176, -0.1295, -0.10,
            -0.176,  0.1295, -0.10
        };

        // Per-joint gains, same ordering as joint_names_ (FR/FL/RR/RL,
        // abad/hip/knee). Defaults preserve the previous common values while
        // allowing each actuator to be tuned independently.
        std::vector<double> stand_kp_ = std::vector<double>(12, 30.0);
        std::vector<double> stand_kd_ = std::vector<double>(12, 1.5);
        std::vector<double> sit_kp_ = std::vector<double>(12, 30.0);
        std::vector<double> sit_kd_ = std::vector<double>(12, 1.5);

        // Tran v_des tung khop. Gia tri thuc duoc StateHoldPose tinh tu hai
        // nghiem IK lien tiep / period; day chi la lop chan EC truoc khi gui
        // qua int16 mrad/s. Firmware con kep lai doc lap lan cuoi.
        std::vector<double> velocity_max_rad_s_ = std::vector<double>(12, 5.0);

        // Thoi gian noi suy tanh (giay) cua vi tri ban chan Cartesian -
        // rieng cho tung chieu vi sit thuong can cham hon stand de do soc co khi (xem
        // controllers.yaml/controllers_real.yaml). Mac dinh 1.2 khop gia tri cu (hardcode
        // truoc khi tach param nay).
        double stand_duration_ = 1.2;
        double sit_duration_ = 1.2;

        rclcpp::Subscription<controller::msg::Inputs>::SharedPtr control_input_subscription_;

        // FK/IK (KDL, port tu superDog - xem controller/robot/QuadrupedRobot.h). Nap
        // qua subscription /robot_description (transient_local, xem on_configure()) -
        // CO THE con null 1 thoi gian ngan luc moi activate neu /robot_description
        // chua kip toi. Passive/ESTOP van phai chay; StateHoldPose se giu kp=0 va
        // cho model hop le truoc khi bat dau FK/IK.
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;
        std::shared_ptr<QuadrupedRobot> robot_model_;
        std::string base_name_ = "body";
        std::vector<std::string> feet_names_ = {
            "front_right_foot", "front_left_foot", "hind_right_foot", "hind_left_foot"
        };

        // Tff = -J(q)^T * F_support tren EC (xem StateHoldPose::run()). Khong co
        // IMU/contact estimator trong Phase 1, nen day la bu tai TINH: Stand tang
        // dan support_blend, Sit giam ve 0; 4 load_share phai co tong bang 1.
        // tau_ff_scale_ la master gain; tau_ff_mass_kg_ > 0 override khoi luong
        // URDF chi cho tinh Tff (de tune robot that ma khong sua inertial model);
        // tau_ff_max_nm_ van kep rieng 12 khop sau khi nhan scale. Gia tri robot
        // that duoc tune trong controllers_real.yaml sau khi baseline
        // tau_ff_scale=0 xac nhan dau -J^T*F; sim dung 1.0.
        double tau_ff_scale_ = 0.0;
        double tau_ff_mass_kg_ = 0.0;
        double tau_ff_ramp_seconds_ = 1.0;
        std::vector<double> tau_ff_load_share_ = std::vector<double>(4, 0.25);
        std::vector<double> tau_ff_joint_scale_ = std::vector<double>(12, 1.0);
        // No fast error-based Tff correction in the control loop: real tests showed that
        // cutting/releasing feedforward from joint error can pump the body and
        // increase bouncing. Instead, keep Tff static and print settled
        // diagnostics for slow offline tuning of tau_ff_joint_scale/load_share.
        bool tau_ff_diagnostics_enabled_ = true;
        double tau_ff_diagnostics_start_seconds_ = 2.0;
        double tau_ff_diagnostics_period_seconds_ = 1.0;
        std::vector<double> stand_joint_trim_rad_ = std::vector<double>(12, 0.0);
        std::vector<double> sit_joint_trim_rad_ = std::vector<double>(12, 0.0);
        // Trang thai ramp dung chung giua State Stand va Sit: nhu vay Stand->Sit
        // ha Tff mem thay vi cat torque dot ngot o bien FSM. Passive/ESTOP reset 0.
        double tau_ff_support_blend_ = 0.0;
        std::vector<double> tau_ff_max_nm_ = std::vector<double>(12, 10.0);

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface> >*>
        command_interface_map_ = {
            {"effort", &ctrl_interfaces_.joint_torque_command_interface_},
            {"position", &ctrl_interfaces_.joint_position_command_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_command_interface_},
            {"kp", &ctrl_interfaces_.joint_kp_command_interface_},
            {"kd", &ctrl_interfaces_.joint_kd_command_interface_}
        };

        FSMMode mode_ = FSMMode::NORMAL;
        FSMStateName next_state_name_ = FSMStateName::INVALID;
        FSMStateList state_list_;
        std::shared_ptr<FSMState> current_state_;
        std::shared_ptr<FSMState> next_state_;

        std::shared_ptr<FSMState> getNextState(FSMStateName stateName) const;

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> >*>
        state_interface_map_ = {
            {"position", &ctrl_interfaces_.joint_position_state_interface_},
            {"effort", &ctrl_interfaces_.joint_effort_state_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_state_interface_}
        };
    };
}

#endif //CONTROLLER_STANDSITCONTROLLER_H
