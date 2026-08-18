// Port tu superDog (/home/dvt/superDog/src/unitree_guide_controller/src/robot/
// QuadrupedRobot.cpp) - DA SUA 1 bug an toan thuc su trong update() (xem duoi).

#include <iostream>
#include <stdexcept>
#include <urdf/model.h>
#include "controller/CtrlInterfaces.h"
#include "controller/robot/QuadrupedRobot.h"

QuadrupedRobot::QuadrupedRobot(CtrlInterfaces &ctrl_interfaces, const std::string &robot_description,
                               const std::vector<std::string> &feet_names,
                               const std::string &base_name) : ctrl_interfaces_(ctrl_interfaces) {
    // feet_names.size()==4 da duoc StandSitController::on_init() kiem tra truoc khi
    // callback /robot_description (noi constructor nay duoc goi) co the chay - nhung
    // van kiem tra lai o day (bao ve khi class nay duoc dung o noi khac sau nay).
    if (feet_names.size() != 4) {
        throw std::runtime_error("QuadrupedRobot: feet_names phai co dung 4 phan tu, hien co " +
                                  std::to_string(feet_names.size()));
    }

    KDL::Tree robot_tree;
    if (!kdl_parser::treeFromString(robot_description, robot_tree)) {
        throw std::runtime_error("QuadrupedRobot: khong parse duoc /robot_description thanh KDL::Tree");
    }

    // getChain() tra ve false (chain rong, KHONG nem exception) neu base_name/foot link
    // khong ton tai trong URDF (vd typo trong controllers.yaml's base_name/feet_names) -
    // bo qua gia tri tra ve nay se de RobotLeg xay tren 1 chain rong, khien FK/IK/Jacobian
    // (va Tff dung getTorque()) tinh ra gia tri sai IM LANG thay vi bao loi ro rang. Kiem
    // tra tuong minh + nem exception - StandSitController::on_configure()'s callback bat
    // exception nay (try/catch), log ro rang, robot_model_ giu nguyen null (Tff tu dong
    // tat, KHONG crash node).
    if (!robot_tree.getChain(base_name, feet_names[0], fr_chain_) ||
        !robot_tree.getChain(base_name, feet_names[1], fl_chain_) ||
        !robot_tree.getChain(base_name, feet_names[2], rr_chain_) ||
        !robot_tree.getChain(base_name, feet_names[3], rl_chain_)) {
        throw std::runtime_error(
            "QuadrupedRobot: getChain() that bai - kiem tra base_name (\"" + base_name +
            "\") va feet_names co dung ten link trong URDF khong");
    }


    urdf::Model urdf_model;
    if (!urdf_model.initString(robot_description)) {
        throw std::runtime_error("QuadrupedRobot: khong parse duoc URDF de doc joint limits");
    }

    const std::vector<KDL::Chain> chains = {fr_chain_, fl_chain_, rr_chain_, rl_chain_};
    robot_legs_.resize(4);
    for (int leg = 0; leg < 4; ++leg) {
        const auto joint_count = chains[leg].getNrOfJoints();
        KDL::JntArray q_min(joint_count);
        KDL::JntArray q_max(joint_count);
        unsigned int joint_index = 0;
        for (unsigned int segment = 0; segment < chains[leg].getNrOfSegments(); ++segment) {
            const KDL::Joint &kdl_joint = chains[leg].getSegment(segment).getJoint();
            if (kdl_joint.getType() == KDL::Joint::None) {
                continue;
            }
            const auto urdf_joint = urdf_model.getJoint(kdl_joint.getName());
            if (!urdf_joint || !urdf_joint->limits || joint_index >= joint_count) {
                throw std::runtime_error("QuadrupedRobot: thieu limit cho joint \"" +
                                         kdl_joint.getName() + "\"");
            }
            q_min(joint_index) = urdf_joint->limits->lower;
            q_max(joint_index) = urdf_joint->limits->upper;
            ++joint_index;
        }
        if (joint_index != joint_count) {
            throw std::runtime_error("QuadrupedRobot: so joint limit khong khop KDL chain");
        }
        robot_legs_[leg] = std::make_shared<RobotLeg>(chains[leg], q_min, q_max);
    }

    current_joint_pos_.resize(4);
    current_joint_vel_.resize(4);

    std::cout << "robot_legs_.size(): " << robot_legs_.size() << std::endl;

    // calculate total mass from urdf
    mass_ = 0;
    for (const auto &[fst, snd]: robot_tree.getSegments()) {
        mass_ += snd.segment.getInertia().getMass();
    }
    // Gia tri nay tu superDog (hinh hoc unitree go2/a1 goc) - KHONG khop hinh hoc
    // babyDog, va KHONG duoc dung boi tinh nang Tff tinh (chi dung mass_ +
    // getTorque()/calcJaco() qua current_joint_pos_ thuc te doc tu feedback,
    // khong dung "vi tri dung chuan" hardcode nay). Giu lai cho du cau truc
    // giong ban goc (phong khi sau nay can cho gait/swing), nhung KHONG duoc
    // tin dung gia tri o day cho babyDog neu chua tu tinh lai.
    feet_pos_normal_stand_ << 0.1881, 0.1881, -0.1881, -0.1881, -0.1300, 0.1300,
            -0.1300, 0.1300, -0.3200, -0.3200, -0.3200, -0.3200;
}

std::vector<KDL::JntArray> QuadrupedRobot::getQ(const std::vector<KDL::Frame> &pEe_list) const {
    std::vector<KDL::JntArray> result;
    result.resize(4);
    for (int i(0); i < 4; ++i) {
        result[i] = robot_legs_[i]->calcQ(pEe_list[i], current_joint_pos_[i]);
    }
    return result;
}

Vec12 QuadrupedRobot::getQ(const Vec34 &vecP) const {
    Vec12 q;
    for (int i(0); i < 4; ++i) {
        KDL::Frame frame;
        frame.p = KDL::Vector(vecP.col(i)[0], vecP.col(i)[1], vecP.col(i)[2]);
        frame.M = KDL::Rotation::Identity();
        q.segment(3 * i, 3) = robot_legs_[i]->calcQ(frame, current_joint_pos_[i]).data;
    }
    return q;
}

bool QuadrupedRobot::solveFootIK(const std::vector<KDL::Vector> &foot_positions,
                                 const std::vector<KDL::JntArray> &q_seed,
                                 std::vector<KDL::JntArray> &q_out,
                                 int *failed_leg) const {
    if (foot_positions.size() != 4 || q_seed.size() != 4) {
        if (failed_leg) *failed_leg = -1;
        return false;
    }
    std::vector<KDL::JntArray> candidate(4);
    for (int leg = 0; leg < 4; ++leg) {
        if (!robot_legs_[leg]->calcQPosition(foot_positions[leg], q_seed[leg], candidate[leg])) {
            if (failed_leg) *failed_leg = leg;
            return false;
        }
    }
    q_out = std::move(candidate);
    return true;
}

Vec12 QuadrupedRobot::getQd(const std::vector<KDL::Frame> &pos, const Vec34 &vel) {
    Vec12 qd;
    const std::vector<KDL::JntArray> q = getQ(pos);
    for (int i(0); i < 4; ++i) {
        Mat3 jacobian = robot_legs_[i]->calcJaco(q[i]).data.topRows(3);
        qd.segment(3 * i, 3) = jacobian.inverse() * vel.col(i);
    }
    return qd;
}

std::vector<KDL::Frame> QuadrupedRobot::getFeet2BPositions() const {
    return getFeet2BPositions(current_joint_pos_);
}

std::vector<KDL::Frame> QuadrupedRobot::getFeet2BPositions(
    const std::vector<KDL::JntArray> &joint_positions) const {
    std::vector<KDL::Frame> result;
    if (joint_positions.size() != 4) {
        return result;
    }
    result.resize(4);
    for (int i = 0; i < 4; i++) {
        if (!robot_legs_[i]->calcPEe2B(joint_positions[i], result[i])) {
            return {};
        }
        result[i].M = KDL::Rotation::Identity();
    }
    return result;
}

KDL::Frame QuadrupedRobot::getFeet2BPositions(const int index) const {
    return robot_legs_[index]->calcPEe2B(current_joint_pos_[index]);
}

KDL::Jacobian QuadrupedRobot::getJacobian(const int index) const {
    return robot_legs_[index]->calcJaco(current_joint_pos_[index]);
}

KDL::JntArray QuadrupedRobot::getTorque(
    const Vec3 &force, const int index) const {
    return robot_legs_[index]->calcTorque(current_joint_pos_[index], force);
}

KDL::JntArray QuadrupedRobot::getTorque(const KDL::Vector &force, int index) const {
    return robot_legs_[index]->calcTorque(current_joint_pos_[index], Vec3(force.data));
}

KDL::Vector QuadrupedRobot::getFeet2BVelocities(const int index) const {
    const Mat3 jacobian = getJacobian(index).data.topRows(3);
    Vec3 foot_velocity = jacobian * current_joint_vel_[index].data;
    return {foot_velocity(0), foot_velocity(1), foot_velocity(2)};
}

std::vector<KDL::Vector> QuadrupedRobot::getFeet2BVelocities() const {
    std::vector<KDL::Vector> result;
    result.resize(4);
    for (int i = 0; i < 4; i++) {
        result[i] = getFeet2BVelocities(i);
    }
    return result;
}

bool QuadrupedRobot::clampLegToLimits(const int index, KDL::JntArray &q) const {
    return robot_legs_[index]->clampToLimits(q);
}

void QuadrupedRobot::update() {
    if (mass_ == 0) return;
    for (int i = 0; i < 4; i++) {
        // SUA BUG AN TOAN THUC SU (khac ban goc superDog): ban goc dung thang
        // .get_optional().value() - nem std::bad_optional_access (undefined
        // behavior, khong bat duoc) neu CHUA TUNG co feedback that nao toi (dung
        // tinh huong hien tai cua babyDog khi chua noi dong co that). Da tu gap +
        // tu sua dung loai bug nay 1 lan that trong StateHoldPose::enter() dem nay
        // (xem controller/src/FSM/StateHoldPose.cpp) - FSM bi ket vinh vien vi
        // exception xay ra giua ham, khien dong lenh sau .value() (vd
        // mode_ = FSMMode::NORMAL o StandSitController::update()) khong bao gio
        // chay toi. .value_or(0.0) o day AN TOAN vi StateHoldPose chi bat Kp/Tff
        // sau khi doc du feedback huu han va FK/IK khoi tao thanh cong; fallback
        // nay khong bao gio tu no tro thanh lenh gui xuong motor.
        KDL::JntArray pos_array(3);
        pos_array(0) = ctrl_interfaces_.joint_position_state_interface_[i * 3].get().get_optional().value_or(0.0);
        pos_array(1) = ctrl_interfaces_.joint_position_state_interface_[i * 3 + 1].get().get_optional().value_or(0.0);
        pos_array(2) = ctrl_interfaces_.joint_position_state_interface_[i * 3 + 2].get().get_optional().value_or(0.0);
        current_joint_pos_[i] = pos_array;

        KDL::JntArray vel_array(3);
        vel_array(0) = ctrl_interfaces_.joint_velocity_state_interface_[i * 3].get().get_optional().value_or(0.0);
        vel_array(1) = ctrl_interfaces_.joint_velocity_state_interface_[i * 3 + 1].get().get_optional().value_or(0.0);
        vel_array(2) = ctrl_interfaces_.joint_velocity_state_interface_[i * 3 + 2].get().get_optional().value_or(0.0);
        current_joint_vel_[i] = vel_array;
    }
}
