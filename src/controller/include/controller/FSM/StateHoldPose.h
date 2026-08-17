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
// interpolation has settled (percent_ >= 1.5), so a direction is never
// reversed mid-motion.
class StateHoldPose final : public FSMState
{
public:
    // robot_model/tau_ff_scale/tau_ff_max_nm nhan qua THAM CHIEU (khong phai gia
    // tri) toi cac thanh vien tuong ung cua StandSitController - vi robot_model
    // (con tro shared_ptr) co the con null luc StateHoldPose duoc tao (on_activate())
    // roi duoc gan sau, khi callback /robot_description chay (xem
    // StandSitController::on_configure()); giu tham chieu de StateHoldPose luon
    // thay gia tri MOI NHAT thay vi ban sao dong bang luc constructor chay.
    StateHoldPose(CtrlInterfaces& ctrl_interfaces,
                  FSMStateName state_name,
                  const std::string& state_name_string,
                  const std::vector<double>& target_foot_positions,
                  double kp,
                  double kd,
                  double duration_seconds,
                  std::shared_ptr<QuadrupedRobot>& robot_model,
                  double& tau_ff_scale,
                  double& tau_ff_max_nm);

    void enter() override;

    void run(const rclcpp::Time& time,
             const rclcpp::Duration& period) override;

    void exit() override;

    FSMStateName checkChange() override;

private:
    bool initializeCartesianMotion(const std::shared_ptr<QuadrupedRobot>& robot_model);

    std::vector<KDL::Vector> target_foot_positions_;
    std::vector<KDL::Vector> start_foot_positions_;
    std::vector<KDL::JntArray> ik_seed_;
    bool motion_ready_ = false;
    bool ik_error_reported_ = false;

    double kp_, kd_;

    double duration_ = 600; // steps
    double percent_ = 0; // 0..1 (well past 1 via tanh saturation, see run())

    std::shared_ptr<QuadrupedRobot>& robot_model_;
    double& tau_ff_scale_;
    double& tau_ff_max_nm_;
};

#endif //CONTROLLER_STATEHOLDPOSE_H
