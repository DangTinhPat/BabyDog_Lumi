// Port tu superDog (/home/dvt/superDog/src/unitree_guide_controller/include/
// unitree_guide_controller/robot/QuadrupedRobot.h) - doi include path
// (unitree_guide_controller/ -> controller/). Xem QuadrupedRobot.cpp cho phan
// sua bug an toan (update()).

#ifndef QUADRUPEDROBOT_H
#define QUADRUPEDROBOT_H
#include <string>
#include <kdl_parser/kdl_parser.hpp>
#include <controller/common/mathTypes.h>

#include "RobotLeg.h"


struct CtrlInterfaces;

class QuadrupedRobot {
public:
    explicit QuadrupedRobot(CtrlInterfaces &ctrl_interfaces, const std::string &robot_description,
                            const std::vector<std::string> &feet_names, const std::string &base_name);

    ~QuadrupedRobot() = default;

    /**
     * Calculate the joint positions based on the foot end position
     * @param pEe_list vector of foot-end position
     * @return
     */
    [[nodiscard]] std::vector<KDL::JntArray> getQ(const std::vector<KDL::Frame> &pEe_list) const;

    [[nodiscard]] Vec12 getQ(const Vec34 &vecP) const;

    /** Solve position-only IK for all four feet using one seed per leg. */
    [[nodiscard]] bool solveFootIK(const std::vector<KDL::Vector> &foot_positions,
                                   const std::vector<KDL::JntArray> &q_seed,
                                   std::vector<KDL::JntArray> &q_out,
                                   int *failed_leg = nullptr) const;

    Vec12 getQd(const std::vector<KDL::Frame> &pos, const Vec34 &vel);

    /**
     * Calculate the foot end position based on joint positions
     * @return vector of foot-end position
     */
    [[nodiscard]] std::vector<KDL::Frame> getFeet2BPositions() const;

    /** FK from an explicit, coherent four-leg joint snapshot. */
    [[nodiscard]] std::vector<KDL::Frame> getFeet2BPositions(
        const std::vector<KDL::JntArray> &joint_positions) const;

    /**
     * Calculate the foot end position based on joint positions
     * @param index leg index
     * @return foot-end position
     */
    [[nodiscard]] KDL::Frame getFeet2BPositions(int index) const;

    /**
     * Calculate the Jacobian matrix based on joint positions
     * @param index leg index
     * @return Jacobian matrix
     */
    [[nodiscard]] KDL::Jacobian getJacobian(int index) const;

    /**
     * Calculate the torque based on joint positions, joint velocities and external force
     * @param force external force
     * @param index leg index
     * @return torque
     */
    [[nodiscard]] KDL::JntArray getTorque(
        const Vec3 &force, int index) const;

    /**
    * Calculate the torque based on joint positions, joint velocities and external force
    * @param force external force
    * @param index leg index
    * @return torque
    */
    [[nodiscard]] KDL::JntArray getTorque(
        const KDL::Vector &force, int index) const;

    /**
     * Calculate the foot end velocity
     * @param index leg index
     * @return velocity vector
     */
    [[nodiscard]] KDL::Vector getFeet2BVelocities(int index) const;

    /**
     * Calculate all foot end velocity
     * @return list of foot end velocity
     */
    [[nodiscard]] std::vector<KDL::Vector> getFeet2BVelocities() const;

    double mass_ = 0;
    Vec34 feet_pos_normal_stand_;
    std::vector<KDL::JntArray> current_joint_pos_;
    std::vector<KDL::JntArray> current_joint_vel_;

    void update();

private:
    CtrlInterfaces &ctrl_interfaces_;
    std::vector<std::shared_ptr<RobotLeg> > robot_legs_;

    KDL::Chain fr_chain_;
    KDL::Chain fl_chain_;
    KDL::Chain rr_chain_;
    KDL::Chain rl_chain_;
};


#endif //QUADRUPEDROBOT_H
