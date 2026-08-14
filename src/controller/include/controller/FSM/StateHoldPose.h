#ifndef CONTROLLER_STATEHOLDPOSE_H
#define CONTROLLER_STATEHOLDPOSE_H

#include "FSMState.h"
#include <memory>

class QuadrupedRobot;

// Smoothly interpolates every joint (tanh ramp, same shape as superDog's
// BaseFixedStand/StateFixedDown) from wherever it currently is to a fixed
// target pose, then holds it there with ordinary joint-space PD. Used for
// both STAND (target_pos = stand_pos) and SIT (target_pos = sit_pos) - no
// separate base/derived pair needed since neither pose does the active
// IMU-driven balance superDog's BaseFixedStand does once settled (we have no
// IMU yet, see main_bot/description/babydog.xacro).
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
                  const std::vector<double>& target_pos,
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
    double target_pos_[12] = {};
    double start_pos_[12] = {};

    double kp_, kd_;

    double duration_ = 600; // steps
    double percent_ = 0; // 0..1 (well past 1 via tanh saturation, see run())

    std::shared_ptr<QuadrupedRobot>& robot_model_;
    double& tau_ff_scale_;
    double& tau_ff_max_nm_;
};

#endif //CONTROLLER_STATEHOLDPOSE_H
