#include "controller/FSM/StateHoldPose.h"
#include "controller/robot/QuadrupedRobot.h"

#include <algorithm>
#include <cmath>

StateHoldPose::StateHoldPose(CtrlInterfaces& ctrl_interfaces,
                              const FSMStateName state_name,
                              const std::string& state_name_string,
                              const std::vector<double>& target_foot_positions,
                              const double kp,
                              const double kd,
                              const double duration_seconds,
                              std::shared_ptr<QuadrupedRobot>& robot_model,
                              double& tau_ff_scale,
                              double& tau_ff_max_nm)
    : FSMState(state_name, state_name_string, ctrl_interfaces),
      kp_(kp), kd_(kd),
      robot_model_(robot_model), tau_ff_scale_(tau_ff_scale), tau_ff_max_nm_(tau_ff_max_nm)
{
    duration_ = ctrl_interfaces_.frequency_ * duration_seconds;
    target_foot_positions_.resize(4);
    for (int leg = 0; leg < 4; ++leg)
    {
        target_foot_positions_[leg] = KDL::Vector(
            target_foot_positions[leg * 3],
            target_foot_positions[leg * 3 + 1],
            target_foot_positions[leg * 3 + 2]);
    }
}

bool StateHoldPose::initializeCartesianMotion(const std::shared_ptr<QuadrupedRobot>& robot_model)
{
    if (!robot_model || ctrl_interfaces_.joint_position_state_interface_.size() != 12)
    {
        return false;
    }

    std::vector<KDL::JntArray> measured_q(4, KDL::JntArray(3));
    for (int leg = 0; leg < 4; ++leg)
    {
        for (int joint = 0; joint < 3; ++joint)
        {
            const auto value = ctrl_interfaces_.joint_position_state_interface_[leg * 3 + joint]
                                   .get().get_optional();
            if (!value || !std::isfinite(*value))
            {
                return false;
            }
            measured_q[leg](joint) = *value;
        }
    }

    // Use the exact same coherent encoder snapshot for FK and as the first IK
    // seed. This avoids a stale/model-cache read at a state transition.
    const auto measured_feet = robot_model->getFeet2BPositions(measured_q);
    if (measured_feet.size() != 4)
    {
        return false;
    }
    start_foot_positions_.resize(4);
    for (int leg = 0; leg < 4; ++leg)
    {
        start_foot_positions_[leg] = measured_feet[leg].p;
    }

    // Do not solve the final target in one jump here. The home configuration
    // is close to a straight-leg singularity, where a one-shot iterative IK
    // can choose the mirrored (out-of-limit) knee branch. run() follows small
    // Cartesian increments and validates convergence + limits before every
    // command, which reliably stays on the measured branch.
    ik_seed_ = std::move(measured_q);
    motion_ready_ = true;
    return true;
}

void StateHoldPose::enter()
{
    // position/kp/kd LA BAT BUOC tren CA 2 duong (sim va real, xem controllers.yaml/
    // controllers_real.yaml) - khac velocity/torque (co the vang mat, xem has_velocity/
    // has_torque duoi day). Neu thieu (config bien the sau nay lam sai), THOAT SOM va
    // BAO LOI RO RANG thay vi index [i] ngoai vung (UB, crash im lang) - an toan hon la
    // "boi qua tung phan tu" nhu has_velocity/has_torque, vi thieu position/kp/kd nghia
    // la controller nay khong the hoat dong dung nghia gi ca (khac velocity/torque von
    // la tinh nang tuy chon).
    if (ctrl_interfaces_.joint_position_command_interface_.size() != 12 ||
        ctrl_interfaces_.joint_kp_command_interface_.size() != 12 ||
        ctrl_interfaces_.joint_kd_command_interface_.size() != 12 ||
        ctrl_interfaces_.joint_position_state_interface_.size() != 12)
    {
        fprintf(stderr,
                "StateHoldPose::enter(): thieu position/kp/kd interface (can du 12 moi loai) - "
                "kiem tra command_interfaces/state_interfaces trong controllers.yaml. Bo qua "
                "enter() de tranh doc/ghi ngoai vung.\n");
        return;
    }

    ctrl_interfaces_.control_inputs_.command = 0;
    // controllers_real.yaml (robot that) chi khai command_interfaces: [position, kp, kd]
    // - khac sim (controllers.yaml, co them velocity, chuyen tiep qua leg_pd_controller
    // de tinh effort) - nen joint_velocity_command_interface_/joint_torque_command_
    // interface_ RONG (size=0) tren duong that. Truy cap [i] tren vector rong la
    // undefined behavior (KHONG nem exception de bat duoc, chi crash im lang) - day
    // chinh la nguyen nhan goc lam mode_ ket o CHANGE vinh vien, FSM dung hinh sau lan
    // chuyen trang thai dau tien (da xac nhan qua debug log: crash dung giua enter(),
    // truoc dong mode_ = FSMMode::NORMAL). Chi ghi vao interface THAT SU CO (size dung
    // 12, khop het joint) - an toan cho ca 2 duong (that: chi position/kp/kd; sim: co
    // ca velocity).
    const bool has_velocity = ctrl_interfaces_.joint_velocity_command_interface_.size() == 12;
    const bool has_torque = ctrl_interfaces_.joint_torque_command_interface_.size() == 12;
    bool all_feedback_valid = true;
    for (int i = 0; i < 12; i++)
    {
        const auto feedback = ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional();
        all_feedback_valid = all_feedback_valid && feedback && std::isfinite(*feedback);
        std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(
            feedback.value_or(0.0));
        if (has_velocity)
        {
            std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(0.0);
        }
        if (has_torque)
        {
            std::ignore = ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(0.0);
        }
        // Keep position actuation disabled until feedback and FK are valid.
        // Every IK command is validated separately in run().
        std::ignore = ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(0.0);
        std::ignore = ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kd_);
    }
    percent_ = 0;
    motion_ready_ = false;
    ik_error_reported_ = false;

    const auto robot_model_snapshot = std::atomic_load(&robot_model_);
    if (all_feedback_valid && initializeCartesianMotion(robot_model_snapshot))
    {
        for (int i = 0; i < 12; ++i)
        {
            std::ignore = ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kp_);
        }
    }
}

void StateHoldPose::run(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    const auto robot_model_snapshot = std::atomic_load(&robot_model_);
    if (!motion_ready_)
    {
        if (!initializeCartesianMotion(robot_model_snapshot))
        {
            return;
        }
        for (int i = 0; i < 12; ++i)
        {
            std::ignore = ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kp_);
        }
    }

    const double next_percent = percent_ + 1.0 / duration_;
    const double phase = std::tanh(next_percent);
    std::vector<KDL::Vector> desired_feet(4);
    for (int leg = 0; leg < 4; ++leg)
    {
        desired_feet[leg] = (1.0 - phase) * start_foot_positions_[leg] +
                            phase * target_foot_positions_[leg];
    }

    std::vector<KDL::JntArray> q_command;
    int failed_leg = -1;
    if (!robot_model_snapshot ||
        !robot_model_snapshot->solveFootIK(desired_feet, ik_seed_, q_command, &failed_leg))
    {
        if (!ik_error_reported_)
        {
            fprintf(stderr,
                    "StateHoldPose::run(): IK chan %d khong hoi tu/vi pham joint limit; "
                    "giu lenh hop le gan nhat, khong tang pha.\n", failed_leg);
            if (failed_leg >= 0 && failed_leg < 4)
            {
                fprintf(stderr,
                        "  seed q=[%.9f %.9f %.9f], start foot=[%.9f %.9f %.9f], "
                        "desired foot=[%.9f %.9f %.9f]\n",
                        ik_seed_[failed_leg](0), ik_seed_[failed_leg](1), ik_seed_[failed_leg](2),
                        start_foot_positions_[failed_leg].x(), start_foot_positions_[failed_leg].y(),
                        start_foot_positions_[failed_leg].z(), desired_feet[failed_leg].x(),
                        desired_feet[failed_leg].y(), desired_feet[failed_leg].z());
            }
            ik_error_reported_ = true;
        }
        return;
    }

    for (int leg = 0; leg < 4; ++leg)
    {
        for (int joint = 0; joint < 3; ++joint)
        {
            std::ignore = ctrl_interfaces_.joint_position_command_interface_[leg * 3 + joint]
                              .get().set_value(q_command[leg](joint));
        }
    }
    ik_seed_ = std::move(q_command);
    percent_ = next_percent;

    // Tff (bu trong luc tinh) - AN TOAN MAC DINH TAT (tau_ff_scale_=0.0, xem
    // StandSitController.h). Chi tinh/ghi khi:
    //   1. robot_model_ da nap xong (FK/IK san sang, xem StandSitController::
    //      on_configure()'s /robot_description subscription)
    //   2. da settled (percent_ >= 1.5, dung y het nguong dung trong checkChange())
    //      - luc dang noi suy, vi tri thay doi lien tuc, gia dinh tinh/can bang 4
    //      chan khong con dung, cong tau_ff luc nay co the cong don sai so dung luc
    //      dong co di chuyen nhanh nhat (rui ro nhat).
    //   3. interface effort THAT SU CO (size==12) - tu dong false tren
    //      controllers_real.yaml (khong khai "effort" trong command_interfaces,
    //      xem StateHoldPose::enter()) - Tff KHONG bao gio chay xuong robot that
    //      trong thiet ke nay, chi trong sim (controllers.yaml co "effort", chay
    //      qua leg_pd_controller: torque = effort_ff + kp*err + kd*err).
    const bool has_torque = ctrl_interfaces_.joint_torque_command_interface_.size() == 12;
    if (has_torque)
    {
        // std::atomic_load - xem StandSitController::update()/on_configure() giai thich
        // day du ly do (robot_model_ o day la THAM CHIEU toi cung 1 shared_ptr thanh
        // vien cua StandSitController, bi doc/ghi dong thoi tu 2 luong khac nhau).
        // Chup 1 ban snapshot on dinh 1 lan, dung xuyen suot khoi if nay - tranh truong
        // hop robot_model_ bi gan lai (con tro moi) giua chung cac lan doc rieng le.
        if (robot_model_snapshot && percent_ >= 1.5)
        {
            // Gia dinh tinh, can bang, chia deu 4 chan (chua co IMU nen khong biet
            // nghieng/lech tam) - F huong +Z trong frame goc KDL chain (tai
            // base_name_="body") = phan luc mat dat can co de nang 1/4 trong luong
            // co the.
            //
            // kTffSign: RobotLeg::calcTorque() = J(q)^T * F dung quy uoc "F la luc
            // chan tac dung LEN MAT DAT" (chuan balance-control cua unitree-guide) -
            // NGUOC voi truc giac "F la luc can de nang robot len". Da kiem chung
            // thuc nghiem (log throttled tau_ff truoc khi nhan scale, so sanh dau voi
            // /joint_states.effort cua PD dang giu Stand voi tau_ff_scale=0.0): dau
            // tho RAW nguoc hoan toan voi dau PD tren ca 12 khop (vd front_left luc
            // scale=0: abad PD=-3.66 nhung tau_ff_raw abad=+1.77) - can dao dau de Tff
            // THAT SU giam tai PD thay vi cong don. KHONG sua RobotLeg::calcTorque
            // (giu ham do generic, dung quy uoc goc cua no) - chi dao dau o day.
            constexpr double kTffSign = -1.0;
            const double f_z = (robot_model_snapshot->mass_ * 9.81) / 4.0;
            for (int leg = 0; leg < 4; leg++)
            {
                const KDL::JntArray tau_leg = robot_model_snapshot->getTorque(Vec3(0.0, 0.0, f_z), leg);
                for (int j = 0; j < 3; j++)
                {
                    double tau = kTffSign * tau_leg(j) * tau_ff_scale_;
                    tau = std::clamp(tau, -tau_ff_max_nm_, tau_ff_max_nm_);
                    std::ignore = ctrl_interfaces_.joint_torque_command_interface_[leg * 3 + j].get().set_value(tau);
                }
            }
        }
        else
        {
            for (int i = 0; i < 12; i++)
            {
                std::ignore = ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(0.0);
            }
        }
    }
}

void StateHoldPose::exit()
{
    percent_ = 0;
    motion_ready_ = false;
}

FSMStateName StateHoldPose::checkChange()
{
    // ESTOP always takes effect immediately, even mid-interpolation.
    if (ctrl_interfaces_.control_inputs_.command == 9)
    {
        return FSMStateName::PASSIVE;
    }
    // Otherwise, let the current motion settle before honoring a new target -
    // avoids reversing direction mid-swing.
    if (percent_ < 1.5)
    {
        return state_name;
    }
    switch (ctrl_interfaces_.control_inputs_.command)
    {
    case 1:
        return FSMStateName::STAND;
    case 2:
        return FSMStateName::SIT;
    default:
        return state_name;
    }
}
