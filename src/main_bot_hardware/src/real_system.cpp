#include "main_bot_hardware/real_system.hpp"

#include <algorithm>
#include <cmath>

#include <rclcpp/logging.hpp>

namespace main_bot_hardware
{

namespace
{
rclcpp::Logger Logger() { return rclcpp::get_logger("main_bot_hardware"); }

// Kep + kiem tra NaN/inf TRUOC khi ep kieu sang int16_t/uint16_t - ep kieu 1 double
// vuot pham vi bieu dien (hoac NaN) sang kieu nguyen la undefined behavior trong
// C++ (KHONG phai wrap an toan), khong phai loi ly thuyet: 1 loi YAML (vd nhap do
// thay vi radian cho stand_pos/sit_pos) hoac 1 buoc tinh IK/Tff sau nay tra ve gia
// tri bat thuong se toi thang day truoc khi gui xuong dong co that. Day la lop
// phong ve DAU TIEN (truoc ca lop kep KP/KD tren firmware, xem motor_calib.h's
// MOTOR_KP_ABS_LIMIT/MOTOR_KD_ABS_LIMIT va Actuator_SetTarget()'s kep vi tri).
int16_t ToInt16Safe(const double value)
{
  if (!std::isfinite(value)) { return 0; }
  return static_cast<int16_t>(std::clamp(value, -32767.0, 32767.0));
}

uint16_t ToUint16Safe(const double value)
{
  if (!std::isfinite(value)) { return 0; }
  return static_cast<uint16_t>(std::clamp(value, 0.0, 65535.0));
}
}  // namespace

hardware_interface::CallbackReturn RealSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kJointCount)
  {
    RCLCPP_ERROR(
      Logger(), "RealSystem can dung dung %u khop, xacro khai bao %zu", kJointCount,
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RealSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(kJointCount * 3U);
  for (uint32_t i = 0; i < kJointCount; i++)
  {
    interfaces.emplace_back(info_.joints[i].name, "position", &position_state_[i]);
    interfaces.emplace_back(info_.joints[i].name, "velocity", &velocity_state_[i]);
    interfaces.emplace_back(info_.joints[i].name, "effort", &effort_state_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> RealSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(kJointCount * 3U);
  for (uint32_t i = 0; i < kJointCount; i++)
  {
    interfaces.emplace_back(info_.joints[i].name, "position", &position_command_[i]);
    interfaces.emplace_back(info_.joints[i].name, "kp", &kp_command_[i]);
    interfaces.emplace_back(info_.joints[i].name, "kd", &kd_command_[i]);
  }
  return interfaces;
}

void RealSystem::JointFbCallback(const main_bot_hardware_msgs::msg::JointFb & msg)
{
  std::lock_guard<std::mutex> lock(joint_fb_mutex_);
  latest_joint_fb_ = msg;
  has_joint_fb_ = true;
}

hardware_interface::CallbackReturn RealSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  position_state_.fill(0.0);
  velocity_state_.fill(0.0);
  effort_state_.fill(0.0);
  position_command_.fill(0.0);
  kp_command_.fill(0.0);
  kd_command_.fill(0.0);

  // get_node(): rclcpp::Node do chinh hardware_interface framework tao + spin cho moi
  // hardware component (xem real_system.hpp) - khong tu quan ly executor/thread rieng.
  joint_cmd_pub_ = get_node()->create_publisher<main_bot_hardware_msgs::msg::JointCmd>(
    "/joint_cmd", rclcpp::SensorDataQoS());
  joint_fb_sub_ = get_node()->create_subscription<main_bot_hardware_msgs::msg::JointFb>(
    "/joint_fb", rclcpp::SensorDataQoS(),
    [this](const main_bot_hardware_msgs::msg::JointFb & msg) { JointFbCallback(msg); });

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RealSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  main_bot_hardware_msgs::msg::JointFb fb;
  bool got_fb = false;
  {
    std::lock_guard<std::mutex> lock(joint_fb_mutex_);
    if (has_joint_fb_)
    {
      fb = latest_joint_fb_;
      got_fb = true;
    }
  }

  if (got_fb)
  {
    for (uint32_t i = 0; i < kJointCount; i++)
    {
      position_state_[i] = static_cast<double>(fb.measured_angle_mrad[i]) / 1000.0;
      velocity_state_[i] = static_cast<double>(fb.measured_velocity_mrad_s[i]) / 1000.0;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RealSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  main_bot_hardware_msgs::msg::JointCmd cmd;
  for (uint32_t i = 0; i < kJointCount; i++)
  {
    cmd.target_angle_mrad[i] = ToInt16Safe(position_command_[i] * 1000.0);
  }
  // kp/kd dung chung cho ca 12 khop - stand_sit_controller hien luon ghi cung 1 gia tri
  // stand_kp/sit_kp vao moi khop, lay khop[0] lam dai dien.
  cmd.kp_x100 = ToUint16Safe(kp_command_[0] * 100.0);
  cmd.kd_x100 = ToUint16Safe(kd_command_[0] * 100.0);
  cmd.seq = seq_++;

  joint_cmd_pub_->publish(cmd);

  return hardware_interface::return_type::OK;
}

}  // namespace main_bot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(main_bot_hardware::RealSystem, hardware_interface::SystemInterface)
