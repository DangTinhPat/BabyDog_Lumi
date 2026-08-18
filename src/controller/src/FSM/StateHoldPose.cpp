#include "controller/FSM/StateHoldPose.h"
#include "controller/robot/QuadrupedRobot.h"

#include <algorithm>
#include <cmath>

StateHoldPose::StateHoldPose(CtrlInterfaces& ctrl_interfaces,
                              const FSMStateName state_name,
                              const std::string& state_name_string,
                              const std::vector<double>& target_foot_positions,
                              const std::vector<double>& kp,
                              const std::vector<double>& kd,
                              const std::vector<double>& velocity_max_rad_s,
                              const double duration_seconds,
                              std::shared_ptr<QuadrupedRobot>& robot_model,
                              double& tau_ff_scale,
                              double& tau_ff_mass_kg,
                              double& tau_ff_ramp_seconds,
                              std::vector<double>& tau_ff_load_share,
                              std::vector<double>& tau_ff_joint_scale,
                              bool& tau_ff_diagnostics_enabled,
                              double& tau_ff_diagnostics_start_seconds,
                              double& tau_ff_diagnostics_period_seconds,
                              std::vector<double>& joint_trim_rad,
                              double& tau_ff_support_blend,
                              std::vector<double>& tau_ff_max_nm)
    : FSMState(state_name, state_name_string, ctrl_interfaces),
      kp_(kp), kd_(kd), velocity_max_rad_s_(velocity_max_rad_s),
      robot_model_(robot_model), tau_ff_scale_(tau_ff_scale),
      tau_ff_mass_kg_(tau_ff_mass_kg), tau_ff_ramp_seconds_(tau_ff_ramp_seconds),
      tau_ff_load_share_(tau_ff_load_share), tau_ff_joint_scale_(tau_ff_joint_scale),
      tau_ff_diagnostics_enabled_(tau_ff_diagnostics_enabled),
      tau_ff_diagnostics_start_seconds_(tau_ff_diagnostics_start_seconds),
      tau_ff_diagnostics_period_seconds_(tau_ff_diagnostics_period_seconds),
      joint_trim_rad_(joint_trim_rad), tau_ff_support_blend_(tau_ff_support_blend),
      tau_ff_max_nm_(tau_ff_max_nm)
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

void StateHoldPose::zeroOptionalVelocityCommand()
{
    if (ctrl_interfaces_.joint_velocity_command_interface_.size() != 12)
    {
        return;
    }
    for (int i = 0; i < 12; ++i)
    {
        std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(0.0);
    }
}

void StateHoldPose::zeroOptionalTorqueCommand()
{
    if (ctrl_interfaces_.joint_torque_command_interface_.size() != 12)
    {
        return;
    }
    for (int i = 0; i < 12; ++i)
    {
        std::ignore = ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(0.0);
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
    // Bake the trim into the seed now (not into start_foot_positions_/FK above,
    // which must stay the true physical pose) so run()'s very first velocity
    // sample - (q_command including trim) minus ik_seed_ - doesn't see a one-
    // cycle trim-sized jump. From here on both sides of that subtraction carry
    // the same constant trim every cycle, so it always cancels out cleanly.
    for (int leg = 0; leg < 4; ++leg)
    {
        for (int joint = 0; joint < 3; ++joint)
        {
            ik_seed_[leg](joint) += joint_trim_rad_[leg * 3 + joint];
        }
    }
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
        tau_ff_support_blend_ = 0.0;
        zeroOptionalVelocityCommand();
        zeroOptionalTorqueCommand();
        fprintf(stderr,
                "StateHoldPose::enter(): thieu position/kp/kd interface (can du 12 moi loai) - "
                "kiem tra command_interfaces/state_interfaces trong controllers.yaml. Bo qua "
                "enter() de tranh doc/ghi ngoai vung.\n");
        return;
    }

    ctrl_interfaces_.control_inputs_.command = 0;
    // Config chuan sim va real deu co velocity/effort. Vi cac bien the cau
    // hinh sau nay van co the bo interface tuy chon, truy cap [i] tren vector rong la
    // undefined behavior (KHONG nem exception de bat duoc, chi crash im lang) - day
    // chinh la nguyen nhan goc lam mode_ ket o CHANGE vinh vien, FSM dung hinh sau lan
    // chuyen trang thai dau tien (da xac nhan qua debug log: crash dung giua enter(),
    // truoc dong mode_ = FSMMode::NORMAL). Chi ghi vao interface THAT SU CO (size dung
    // 12, khop het joint) - an toan cho ca 2 duong.
    const bool has_velocity = ctrl_interfaces_.joint_velocity_command_interface_.size() == 12;
    bool all_feedback_valid = true;
    for (int i = 0; i < 12; i++)
    {
        const auto feedback = ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional();
        const bool feedback_valid = feedback && std::isfinite(*feedback);
        all_feedback_valid = all_feedback_valid && feedback_valid;
        std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(
            feedback_valid ? *feedback : 0.0);
        if (has_velocity)
        {
            std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(0.0);
        }
        // Keep position actuation disabled until feedback and FK are valid.
        // Every IK command is validated separately in run().
        std::ignore = ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(0.0);
        std::ignore = ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kd_[i]);
    }
    percent_ = 0;
    motion_ready_ = false;
    ik_error_reported_ = false;
    tff_ramp_reported_ = false;
    tff_active_reported_ = false;
    trim_limit_clamped_reported_ = false;
    tff_settled_seconds_ = 0.0;
    tff_diagnostics_elapsed_ = 0.0;

    const auto robot_model_snapshot = std::atomic_load(&robot_model_);
    if (all_feedback_valid && initializeCartesianMotion(robot_model_snapshot))
    {
        for (int i = 0; i < 12; ++i)
        {
            std::ignore = ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kp_[i]);
        }
    }
    else
    {
        // Khong ke thua Tff tu state truoc neu snapshot feedback/FK moi khong
        // hop le. Tff khong tu suy giam theo position error nen phai clear.
        tau_ff_support_blend_ = 0.0;
        zeroOptionalTorqueCommand();
    }
}

void StateHoldPose::run(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
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
            std::ignore = ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kp_[i]);
        }
    }

    std::vector<KDL::JntArray> q_command;
    const bool was_settled = percent_ >= 1.0;
    const double next_percent = was_settled
                                    ? 1.0
                                    : std::min(percent_ + 1.0 / duration_, 1.0);
    if (was_settled && ik_seed_.size() == 4)
    {
        // Sau khi da toi target, giu nguyen nghiem IK cuoi va ep vel_des=0.
        // Tranh goi IK lai mai tren cung target (co the tao vi-sai so so hoc nho)
        // va tranh tanh tiem can mai sinh "băm" target/velocity duoi tai.
        q_command = ik_seed_;
    }
    else
    {
        // Smootherstep (bac 5) 0..1 toi dung target trong duration_seconds, khac
        // tanh cu tiem can mai va van tao buoc nho sau khi robot da gan dung yen.
        // Bac 5 (thay vi smoothstep bac 3) trieu tieu ca gia toc lan van toc o 2
        // dau doan [0,1] - giam giat co khi dung luc bat dau/vua settled, khong
        // chi rieng van toc=0 nhu bac 3.
        const double p = next_percent;
        const double phase = p >= 1.0
                                 ? 1.0
                                 : p * p * p * (p * (p * 6.0 - 15.0) + 10.0);
        std::vector<KDL::Vector> desired_feet(4);
        for (int leg = 0; leg < 4; ++leg)
        {
            desired_feet[leg] = (1.0 - phase) * start_foot_positions_[leg] +
                                phase * target_foot_positions_[leg];
        }

        int failed_leg = -1;
        if (!robot_model_snapshot ||
            !robot_model_snapshot->solveFootIK(desired_feet, ik_seed_, q_command, &failed_leg))
        {
            // Khong giu lai Tff cua chu ky truoc neu model/IK mat hop le. Position
            // command gan nhat van duoc giu boi PD, nhung feedforward phai fail-safe
            // ve 0 vi no khong phu thuoc sai so/feedback de tu giam.
            zeroOptionalVelocityCommand();
            zeroOptionalTorqueCommand();
            tau_ff_support_blend_ = 0.0;
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
    }

    for (int leg = 0; leg < 4; ++leg)
    {
        for (int joint = 0; joint < 3; ++joint)
        {
            const int index = leg * 3 + joint;
            q_command[leg](joint) += joint_trim_rad_[index];
        }
        // solveFootIK() only validated its own raw output against joint limits
        // - trim is added afterward, so a large enough trim could push the
        // final command out of range even though the pre-trim IK solution was
        // valid. Clamp back into range and warn once if it ever actually
        // triggers (harmless no-op while every trim is 0.0).
        if (robot_model_snapshot && robot_model_snapshot->clampLegToLimits(leg, q_command[leg]) &&
            !trim_limit_clamped_reported_)
        {
            fprintf(stderr,
                    "StateHoldPose::run(): leg %d joint_trim_rad pushed a command past its URDF "
                    "limit; clamped back into range. Reduce that trim value.\n", leg);
            trim_limit_clamped_reported_ = true;
        }
    }

    const bool has_velocity = ctrl_interfaces_.joint_velocity_command_interface_.size() == 12;
    const double dt = period.seconds();
    const bool valid_dt = std::isfinite(dt) && dt > 0.0;
    for (int leg = 0; leg < 4; ++leg)
    {
        for (int joint = 0; joint < 3; ++joint)
        {
            const int index = leg * 3 + joint;
            std::ignore = ctrl_interfaces_.joint_position_command_interface_[index]
                              .get().set_value(q_command[leg](joint));
            if (has_velocity)
            {
                // v_des la dao ham cua CHINH quy dao joint do IK tao ra, khong
                // phai van toc encoder. Dung hai nghiem IK hop le lien tiep de
                // dam bao q_des va qd_des mo ta cung mot quy dao. period khong
                // hop le hoac ket qua bat thuong phai fail-safe ve 0; clamp tung
                // khop chan mot buoc IK/scheduler loi tao lenh toc do lon.
                double velocity = (valid_dt && !was_settled)
                                      ? (q_command[leg](joint) - ik_seed_[leg](joint)) / dt
                                      : 0.0;
                velocity = std::isfinite(velocity)
                               ? std::clamp(velocity, -velocity_max_rad_s_[index],
                                            velocity_max_rad_s_[index])
                               : 0.0;
                std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[index]
                                  .get().set_value(velocity);
            }
        }
    }
    ik_seed_ = std::move(q_command);
    percent_ = next_percent;

    // Tff bu tai trong tinh theo mau dieu khien quadruped pho bien:
    //   tau_ff = -J(q_measured)^T * [0, 0, mass*g*load_share].
    // MIT Cheetah, Unitree Guide va D1 deu ket hop J^T*foot-force/WBC torque
    // song song voi joint PD. babyDog Phase 1 chua co IMU/contact estimator nen
    // KHONG gia lap WBC dong: chi gia dinh 4 chan tiep dat khi STAND. Blend tang
    // dan luc Stand va giam dan khi Sit de khong tao buoc nhay torque o bien FSM.
    // Passive/ESTOP, IK loi, model/period loi deu dua Tff ve 0.
    const bool has_torque = ctrl_interfaces_.joint_torque_command_interface_.size() == 12;
    if (has_torque)
    {
        // std::atomic_load - xem StandSitController::update()/on_configure() giai thich
        // day du ly do (robot_model_ o day la THAM CHIEU toi cung 1 shared_ptr thanh
        // vien cua StandSitController, bi doc/ghi dong thoi tu 2 luong khac nhau).
        // Chup 1 ban snapshot on dinh 1 lan, dung xuyen suot khoi if nay - tranh truong
        // hop robot_model_ bi gan lai (con tro moi) giua chung cac lan doc rieng le.
        if (!robot_model_snapshot || !valid_dt || !std::isfinite(tau_ff_support_blend_))
        {
            tau_ff_support_blend_ = 0.0;
            tff_settled_seconds_ = 0.0;
            tff_diagnostics_elapsed_ = 0.0;
            zeroOptionalTorqueCommand();
        }
        else
        {
            const double blend_target =
                state_name == FSMStateName::STAND && tau_ff_scale_ > 0.0 ? 1.0 : 0.0;
            const double max_blend_step = dt / tau_ff_ramp_seconds_;
            const double blend_error = blend_target - tau_ff_support_blend_;
            tau_ff_support_blend_ += std::clamp(
                blend_error, -max_blend_step, max_blend_step);
            tau_ff_support_blend_ = std::clamp(tau_ff_support_blend_, 0.0, 1.0);
            if (!tff_ramp_reported_ && blend_target > 0.0)
            {
                fprintf(stderr,
                        "Tff Stand ramp: period=%.9f s ramp=%.3f s scale=%.3f\n",
                        dt, tau_ff_ramp_seconds_, tau_ff_scale_);
                tff_ramp_reported_ = true;
            }

            // Gia dinh tinh, can bang, chua co IMU nen khong biet nghieng/lech
            // tam. Load share mac dinh deu 0.25 nhung cho phep hieu chinh
            // front/rear trong YAML; tong luon duoc validate bang 1. F huong +Z
            // trong frame goc KDL chain (base_name_="body"). tau_ff_mass_kg_ > 0
            // chi override khoi luong dung cho Tff; FK/IK va inertial URDF khong
            // doi theo gia tri tuning nay.
            //
            // RobotLeg::calcTorque() tra +J(q)^T*F. Baseline robot that voi
            // tau_ff_scale=0 (cung target va Kp/Kd) cho thay moment PD giu dang
            // cung dau voi -J^T*F tren 11/12 khop; dot(PD,-J^T*F)=+28.74. Ngoai le
            // FR hip co |J^T*F| chi 0.020 N.m (gan diem doi dau, khong du y nghia
            // de chon quy uoc). Khong duoc so dau Tff voi P khi Tff dang bat: P se
            // dich chuyen de bu lai chinh feedforward va co the nguoc dau du Tff
            // dung. Firmware doi LOGIC->RAW cho position va torque bang cung
            // MOTOR_JOINT_SIGN, nen khong dao dau them o MCU.
            constexpr double kTffSign = -1.0;
            const double support_mass_kg =
                tau_ff_mass_kg_ > 0.0 ? tau_ff_mass_kg_ : robot_model_snapshot->mass_;
            double max_abs_tau = 0.0;
            double error_rad[12] = {};
            double p_tau_nm[12] = {};
            double tau_ff_nm[12] = {};
            bool error_valid[12] = {};
            for (int leg = 0; leg < 4; leg++)
            {
                const double f_z =
                    support_mass_kg * 9.81 * tau_ff_load_share_[leg];
                const KDL::JntArray tau_leg = robot_model_snapshot->getTorque(Vec3(0.0, 0.0, f_z), leg);
                for (int j = 0; j < 3; j++)
                {
                    const int index = leg * 3 + j;
                    double tau = kTffSign * tau_leg(j) * tau_ff_scale_ *
                                 tau_ff_joint_scale_[index] * tau_ff_support_blend_;
                    const auto measured_q_optional =
                        ctrl_interfaces_.joint_position_state_interface_[index].get().get_optional();
                    if (measured_q_optional && std::isfinite(*measured_q_optional))
                    {
                        const double error = ik_seed_[leg](j) - *measured_q_optional;
                        if (std::isfinite(error))
                        {
                            error_rad[index] = error;
                            p_tau_nm[index] = kp_[index] * error;
                            error_valid[index] = true;
                        }
                    }
                    // std::clamp(NaN, ...) van tra NaN. Tren robot that lop
                    // serialize se doi NaN ve 0, nhung sim co the nhan NaN
                    // truc tiep vao effort; fail-safe ngay tai nguon cho ca hai.
                    tau = std::isfinite(tau)
                              ? std::clamp(tau, -tau_ff_max_nm_[index], tau_ff_max_nm_[index])
                              : 0.0;
                    tau_ff_nm[index] = tau;
                    max_abs_tau = std::max(max_abs_tau, std::abs(tau));
                    std::ignore = ctrl_interfaces_.joint_torque_command_interface_[index].get().set_value(tau);
                }
            }
            if (!tff_active_reported_ && blend_target > 0.0 &&
                tau_ff_support_blend_ >= 1.0 - 1e-6)
            {
                fprintf(stderr,
                        "Tff Stand active: mass=%.3f kg (urdf=%.3f kg) scale=%.3f max_abs=%.3f N.m\n",
                        support_mass_kg, robot_model_snapshot->mass_, tau_ff_scale_, max_abs_tau);
                tff_active_reported_ = true;
            }
            const bool stand_tff_settled =
                state_name == FSMStateName::STAND && percent_ >= 1.0 &&
                tau_ff_support_blend_ >= 1.0 - 1e-6;
            if (stand_tff_settled)
            {
                tff_settled_seconds_ += dt;
                tff_diagnostics_elapsed_ += dt;
            }
            else
            {
                tff_settled_seconds_ = 0.0;
                tff_diagnostics_elapsed_ = 0.0;
            }
            if (tau_ff_diagnostics_enabled_ && stand_tff_settled &&
                tff_settled_seconds_ >= tau_ff_diagnostics_start_seconds_ &&
                tff_diagnostics_elapsed_ >= tau_ff_diagnostics_period_seconds_)
            {
                tff_diagnostics_elapsed_ = 0.0;
                fprintf(stderr,
                        "Tff settled diag (order FR/FL/RR/RL abad/hip/knee): "
                        "scale=%.3f load=[%.3f %.3f %.3f %.3f]\n",
                        tau_ff_scale_, tau_ff_load_share_[0], tau_ff_load_share_[1],
                        tau_ff_load_share_[2], tau_ff_load_share_[3]);
                fprintf(stderr, "  err_rad=[");
                for (int i = 0; i < 12; ++i)
                {
                    fprintf(stderr, "%s%.4f", i == 0 ? "" : ",", error_valid[i] ? error_rad[i] : 0.0);
                }
                fprintf(stderr, "]\n  p_tau_nm=[");
                for (int i = 0; i < 12; ++i)
                {
                    fprintf(stderr, "%s%.3f", i == 0 ? "" : ",", error_valid[i] ? p_tau_nm[i] : 0.0);
                }
                fprintf(stderr, "]\n  tau_ff_nm=[");
                for (int i = 0; i < 12; ++i)
                {
                    fprintf(stderr, "%s%.3f", i == 0 ? "" : ",", tau_ff_nm[i]);
                }
                fprintf(stderr, "]\n");
            }
        }
    }
}

void StateHoldPose::exit()
{
    zeroOptionalVelocityCommand();
    // Khong clear Tff o day: Stand va Sit dung chung tau_ff_support_blend_, de
    // Stand->Sit ramp xuong mem. Neu state ke tiep la Passive/ESTOP, controller
    // reset blend va StatePassive::enter() ghi effort=0 trong cung chu ky CHANGE.
    percent_ = 0;
    motion_ready_ = false;
    tff_settled_seconds_ = 0.0;
    tff_diagnostics_elapsed_ = 0.0;
}

FSMStateName StateHoldPose::checkChange()
{
    // ESTOP always takes effect immediately, even mid-interpolation.
    if (ctrl_interfaces_.control_inputs_.command == 9)
    {
        return FSMStateName::PASSIVE;
    }
    // Otherwise, let the finite smoothstep motion reach its exact target before
    // honoring a new target - avoids reversing direction mid-swing.
    if (percent_ < 1.0)
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
