// Port tu superDog (/home/dvt/superDog/src/unitree_guide_controller/include/
// unitree_guide_controller/robot/RobotLeg.h), extended with checked,
// position-only IK and URDF joint-limit validation for babyDog.

#ifndef ROBOTLEG_H
#define ROBOTLEG_H

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <controller/common/mathTypes.h>

class RobotLeg {
public:
    RobotLeg(const KDL::Chain &chain,
             const KDL::JntArray &q_min,
             const KDL::JntArray &q_max);

    ~RobotLeg() = default;

    /**
     * Use forward kinematic to calculate the Pose of End effector to Body frame.
     * @param joint_positions Leg joint positions
     * @return Pose of End effector to Body frame
     */
    [[nodiscard]] KDL::Frame calcPEe2B(const KDL::JntArray &joint_positions) const;

    /** Checked FK variant used by safety-sensitive state transitions. */
    [[nodiscard]] bool calcPEe2B(const KDL::JntArray &joint_positions,
                                 KDL::Frame &pose_out) const;

    /**
     * Use inverse kinematic to calculate the joint positions.
     * @param pEe target position of end effector
     * @param q_init current joint positions
     * @return target joint positions
     */
    [[nodiscard]] KDL::JntArray calcQ(const KDL::Frame &pEe, const KDL::JntArray &q_init) const;

    /**
     * Solve position-only IK and reject non-converged/non-finite/out-of-limit
     * solutions. A quadruped leg has only three joints, so asking the solver
     * to also match foot orientation would over-constrain the problem.
     */
    [[nodiscard]] bool calcQPosition(const KDL::Vector &target_position,
                                     const KDL::JntArray &q_init,
                                     KDL::JntArray &q_out) const;

    /**
     * Calculate the current jacobian matrix.
     * @param joint_positions Leg joint positions
     * @return jacobian matrix
     */
    [[nodiscard]] KDL::Jacobian calcJaco(const KDL::JntArray &joint_positions) const;

    /**
     * Calculate the torque based on joint positions and end force
     * @param joint_positions current joint positions
     * @param force foot end force
     * @return joint torque
     */
    [[nodiscard]] KDL::JntArray calcTorque(const KDL::JntArray &joint_positions, const Vec3 &force) const;

protected:
    KDL::Chain chain_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_pose_solver_;
    std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
    std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_pose_solver_;
    KDL::JntArray q_min_;
    KDL::JntArray q_max_;
};


#endif //ROBOTLEG_H
