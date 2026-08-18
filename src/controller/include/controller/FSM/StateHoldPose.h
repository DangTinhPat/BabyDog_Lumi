#ifndef CONTROLLER_STATEHOLDPOSE_H
#define CONTROLLER_STATEHOLDPOSE_H

#include "FSMState.h"
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <memory>
#include <vector>

class QuadrupedRobot;

// Cartesian stand/sit motion: FK captures the four measured foot positions on
// entry, the controller interpolates those positions in the body frame, and
// position-only IK produces 12 joint commands every update. Joint-space PD is
// still the low-level actuator control law.
//
// ESTOP can interrupt mid-interpolation (checked every cycle, immediately);
// switching between STAND and SIT only takes effect once the current
// interpolation has settled (percent_ >= 1.0), so a direction is never
// reversed mid-motion.
class StateHoldPose final : public FSMState
{
public:
    // robot_model/cac tham so Tff nhan qua THAM CHIEU (khong phai gia
    // tri) toi cac thanh vien tuong ung cua StandSitController - vi robot_model
    // (con tro shared_ptr) co the con null luc StateHoldPose duoc tao (on_activate())
    // roi duoc gan sau, khi callback /robot_description chay (xem
    // StandSitController::on_configure()); giu tham chieu de StateHoldPose luon
    // thay gia tri MOI NHAT thay vi ban sao dong bang luc constructor chay.
    StateHoldPose(CtrlInterfaces& ctrl_interfaces,
                  FSMStateName state_name,
                  const std::string& state_name_string,
                  const std::vector<double>& target_foot_positions,
                  const std::vector<double>& kp,
                  const std::vector<double>& kd,
                  const std::vector<double>& velocity_max_rad_s,
                  double duration_seconds,
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
                  std::vector<double>& tau_ff_max_nm);

    void enter() override;

    void run(const rclcpp::Time& time,
             const rclcpp::Duration& period) override;

    void exit() override;

    FSMStateName checkChange() override;

private:
    bool initializeCartesianMotion(const std::shared_ptr<QuadrupedRobot>& robot_model);
    void zeroOptionalVelocityCommand();
    void zeroOptionalTorqueCommand();

    std::vector<KDL::Vector> target_foot_positions_;
    std::vector<KDL::Vector> start_foot_positions_;
    std::vector<KDL::JntArray> ik_seed_;
    bool motion_ready_ = false;
    bool ik_error_reported_ = false;
    bool tff_ramp_reported_ = false;
    bool tff_active_reported_ = false;
    bool trim_limit_clamped_reported_ = false;
    std::vector<double> kp_, kd_, velocity_max_rad_s_;

    double duration_ = 600; // steps
    double percent_ = 0; // 0..1 finite smoothstep progress; 1.0 = target latched
    double tff_settled_seconds_ = 0.0;
    double tff_diagnostics_elapsed_ = 0.0;

    std::shared_ptr<QuadrupedRobot>& robot_model_;
    double& tau_ff_scale_;
    double& tau_ff_mass_kg_;
    double& tau_ff_ramp_seconds_;
    std::vector<double>& tau_ff_load_share_;
    std::vector<double>& tau_ff_joint_scale_;
    bool& tau_ff_diagnostics_enabled_;
    double& tau_ff_diagnostics_start_seconds_;
    double& tau_ff_diagnostics_period_seconds_;
    std::vector<double>& joint_trim_rad_;
    double& tau_ff_support_blend_;
    std::vector<double>& tau_ff_max_nm_;
};

#endif //CONTROLLER_STATEHOLDPOSE_H
