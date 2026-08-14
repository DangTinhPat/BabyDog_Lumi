#ifndef CONTROLLER_STATEPASSIVE_H
#define CONTROLLER_STATEPASSIVE_H
#include "FSMState.h"

// Boot / e-stop state: all joints torque-free (kp=0, kd=1 just to damp
// jitter), so the robot goes limp. Entered on startup and whenever an ESTOP
// command arrives from any other state.
class StatePassive final : public FSMState
{
public:
    explicit StatePassive(CtrlInterfaces& ctrl_interfaces);

    void enter() override;

    void run(const rclcpp::Time& time,
             const rclcpp::Duration& period) override;

    void exit() override;

    FSMStateName checkChange() override;
};

#endif //CONTROLLER_STATEPASSIVE_H
