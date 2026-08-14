# RealSystem::write() -> firmware/stm32h7 (subscriber "/joint_cmd", node stm32_joint_node)
# qua micro-ROS/UART, thay cho JointCmdFrame_t cu tren CAN - cung field/don vi, chi doi
# transport. mrad/x100 de giu message nho + tranh serialize double khong can thiet tren MCU
# (xem ly do tuong tu trong testSTM/README.md ve sensor_msgs/Imu qua nang).
int16[12] target_angle_mrad
uint16 kp_x100
uint16 kd_x100
uint8 seq
