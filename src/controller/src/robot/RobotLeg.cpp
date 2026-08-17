// Port tu superDog (/home/dvt/superDog/src/unitree_guide_controller/src/robot/
// RobotLeg.cpp) - extended with checked position-only IK for babyDog.

#include <memory>
#include <algorithm>
#include <cmath>
#include "controller/robot/RobotLeg.h"

#include <controller/common/mathTypes.h>

RobotLeg::RobotLeg(const KDL::Chain &chain,
                   const KDL::JntArray &q_min,
                   const KDL::JntArray &q_max)
    : q_min_(q_min), q_max_(q_max) {
    chain_ = chain;

    // Solvers keep a reference to their Chain. Always bind them to the owned
    // member, never to the constructor argument (which may be a temporary or a
    // short-lived element in a caller-side container).
    fk_pose_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
    // Three translational constraints for three leg joints. Orientation has
    // zero weight because a 3-DOF leg cannot independently control all six
    // components of a foot pose.
    Eigen::Matrix<double, 6, 1> task_weights;
    task_weights << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
    ik_pose_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(
        chain_, task_weights, 1e-6, 100, 1e-12);
    jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(chain_);
}

KDL::Frame RobotLeg::calcPEe2B(const KDL::JntArray &joint_positions) const {
    KDL::Frame pEe;
    (void)calcPEe2B(joint_positions, pEe);
    return pEe;
}

bool RobotLeg::calcPEe2B(const KDL::JntArray &joint_positions,
                         KDL::Frame &pose_out) const {
    if (joint_positions.rows() != chain_.getNrOfJoints()) {
        return false;
    }
    return fk_pose_solver_->JntToCart(joint_positions, pose_out) >= 0;
}

KDL::JntArray RobotLeg::calcQ(const KDL::Frame &pEe, const KDL::JntArray &q_init) const {
    KDL::JntArray q(chain_.getNrOfJoints());
    ik_pose_solver_->CartToJnt(q_init, pEe, q);
    return q;
}

bool RobotLeg::calcQPosition(const KDL::Vector &target_position,
                             const KDL::JntArray &q_init,
                             KDL::JntArray &q_out) const {
    if (q_init.rows() != chain_.getNrOfJoints()) {
        return false;
    }

    KDL::Frame target_frame(KDL::Rotation::Identity(), target_position);
    KDL::JntArray candidate(chain_.getNrOfJoints());
    // The return code alone is not sufficient: LMA can stop because the
    // increment is tiny even when its Cartesian residual is already valid.
    // Validate the actual result below instead.
    (void)ik_pose_solver_->CartToJnt(q_init, target_frame, candidate);

    constexpr double kLimitTolerance = 1e-6;
    for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i) {
        if (!std::isfinite(candidate(i)) ||
            candidate(i) < q_min_(i) - kLimitTolerance ||
            candidate(i) > q_max_(i) + kLimitTolerance) {
            return false;
        }
        candidate(i) = std::clamp(candidate(i), q_min_(i), q_max_(i));
    }

    KDL::Frame reached;
    if (!calcPEe2B(candidate, reached)) {
        return false;
    }
    if ((reached.p - target_position).Norm() > 1e-4) {
        return false;
    }

    q_out = candidate;
    return true;
}

KDL::Jacobian RobotLeg::calcJaco(const KDL::JntArray &joint_positions) const {
    KDL::Jacobian jacobian(chain_.getNrOfJoints());
    jac_solver_->JntToJac(joint_positions, jacobian);
    return jacobian;
}

KDL::JntArray RobotLeg::calcTorque(const KDL::JntArray &joint_positions, const Vec3 &force) const {
    const Eigen::Matrix<double, 3, Eigen::Dynamic> jacobian = calcJaco(joint_positions).data.topRows(3);
    Eigen::VectorXd torque_eigen = jacobian.transpose() * force;
    KDL::JntArray torque(chain_.getNrOfJoints());
    for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i) {
        torque(i) = torque_eigen(i);
    }
    return torque;
}
