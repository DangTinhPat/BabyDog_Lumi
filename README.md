# babyDog

Robot chó 4 chân thật (RDK X5 + STM32H7), giai đoạn cơ bản nhất: **chỉ đứng lên / ngồi xuống**, điều khiển bằng joystick. MPU6050 đã có đường dữ liệu 100 Hz và Kalman trên EC, nhưng chưa tác động vào motor để cân bằng chủ động. Repo gồm cả mô phỏng ROS 2 Jazzy + Gazebo Harmonic và stack phần cứng thật.

Đây là bản rút gọn có chủ đích từ [`superDog`](../superDog) (dự án mô phỏng đi bộ đầy đủ) - tái sử dụng khung vật lý (devq) + `leg_pd_controller`, nhưng chưa mang sang gait/Estimator/QP cân bằng đầy đủ.

## Kiến trúc

```
                     ┌───────────────────┐
                     │  Joystick / bàn    │
                     │  phím (joystick_   │
                     │  bridge)            │
                     └─────────┬─────────┘
                               │ topic /control_input
                               │ (stand_sit_controller/msg/Inputs: command)
                               │ 0=None 1=Stand 2=Sit 9=Estop
                 ┌─────────────┴─────────────┐
                 │                           │
                 ▼                           ▼
     ┌─────────────────────┐     ┌─────────────────────────┐
     │   MÔ PHỎNG (dev PC)  │     │   THỰC TẾ (RDK X5/laptop)│
     │                       │     │                          │
     │  stand_sit_controller │     │  stand_sit_controller    │
     │  (FSM Passive/Stand/  │     │  (command_prefix="",     │
     │   Sit, ros2_control)  │     │   claim thẳng hardware)  │
     │         │             │     │         │                │
     │         ▼             │     │         ▼                │
     │  leg_pd_controller    │     │  main_bot_hardware/       │
     │  (tau = kp*Δq+kd*Δq̇)  │     │  RealSystem (publish/     │
     │         │             │     │  subscribe /joint_cmd,    │
     │         ▼             │     │  /joint_fb qua micro-ROS) │
     │  gz_ros2_control      │     └────────────┬─────────────┘
     │         │             │                  │ micro_ros_agent (serial,
     │         ▼             │                  │ UART1 <-> DDS, ~921600 baud)
     │  Gazebo - 12 khớp     │                  ▼
     │  babydog vật lý       │     ┌─────────────────────────┐
     └───────────────────────┘     │   STM32H7 (firmware/     │
                                    │   stm32h7/, board FK743  │
                                    │   M5-XIH6, không HAL)    │
                                    │  - microros_bridge.c:    │
                                    │    node stm32_joint_node │
                                    │  - relay lệnh khớp +     │
                                    │    watchdog ngắt lực     │
                                    │  - actuator_if.c: lệnh   │
                                    │  vị trí/vận tốc/kp/kd qua│
                                    │    CAN-FD cho 12    │
                                    │    board driver khớp     │
                                    │    (mỗi khớp tự PD +     │
                                    │    PWM + encoder 6 dây)  │
                                    └─────────────────────────┘
```

Cả 2 nhánh (mô phỏng/thực tế) dùng FSM trong `stand_sit_controller`. Trên robot thật, controller chạy FK/IK rồi gửi lệnh khớp xuống STM32; firmware không tự chạy FSM hay chọn tư thế.

Đường IMU độc lập với actuation: MPU6050 thật đi `I2C1 -> STM32 -> /imu/raw`, Gazebo đi
`/imu/sim_raw`; cả hai vào `imu_kalman_filter` trên EC và xuất chuẩn `/imu/data` + `/diagnostics`.

## Cấu trúc thư mục

```
src/
├── main_bot/              Mô tả robot (kế thừa devq từ superDog) + world + launch
├── leg_pd_controller/     PD cấp thấp (copy nguyên từ superDog, không đổi)
├── stand_sit_controller/  FSM Passive/Stand/Sit (bản rút gọn của unitree_guide_controller
│                          - KHÔNG có gait/WaveGenerator/Estimator/BalanceCtrl QP)
├── imu_kalman_filter/    Hiệu chuẩn gyro + Kalman roll/pitch/accel/gyro trên EC
├── joystick_bridge/       Node Python: joystick thật (ros2 `joy`) + bàn phím -> /control_input
├── main_bot_hardware/     hardware_interface::SystemInterface cho robot thật - relay
│                    position/velocity/kp/kd/Tff riêng cho 12 khớp tới firmware qua micro-ROS/UART
├── main_bot_hardware_msgs/ JointCmd/JointFb/ImuRaw - message giữa EC và firmware
└── gui/                   GUI sim/real/FSM với hàng IMU-only + monitor raw/filtered read-only

firmware/stm32h7/          Firmware STM32H743, không HAL/CMSIS (đăng ký thanh ghi trực tiếp,
                           kế thừa từ ~/OUT_SAVE/babyDog_fwSTM/). Xem firmware/stm32h7/README.md.
```

## Cách chạy nhanh nhất

Xem [GUIDE.md](GUIDE.md) - có `make sim`, `make stand`, `make sit`, v.v. để khỏi phải mở nhiều terminal/nhớ lệnh dài.

## Đã build & test

- Workspace ROS 2 build sạch; `imu_kalman_filter` có kiểm thử tổng hợp bias/tilt/nhiễu/delta-time.
- Đã chạy thật `sim.launch.py`, kiểm tra qua CLI: `joint_state_broadcaster`/`leg_pd_controller`/`controller` đều activate thành công; gửi lệnh Stand (1) làm 12 khớp hội tụ về đúng tư thế đứng (~[0, -0.748, 1.495] rad/khớp); gửi Sit (2) hội tụ về tư thế ngồi (~[0, -1.231, 2.462]); gửi Estop (9) không làm controller crash.
- Firmware `firmware/stm32h7` dùng thư viện thanh ghi thật của bạn (`~/OUT_SAVE/babyDog_fwSTM/lib/`, board FK743M5-XIH6) - `lib/can.c` đã test loopback CAN thật trên board thật (7 test case Classic/FD, xem `firmware/stm32h7/README.md`); code ứng dụng (`app/`, `main.c`) build sạch với `arm-none-eabi-gcc 13.2.1` nhưng CHƯA nạp/chạy trên board ở mức toàn hệ thống. Đã sửa 1 lỗi Makefile (thiếu include path `newlib/`) khiến cả project firmware gốc `~/OUT_SAVE/babyDog_fwSTM/` không build được với toolchain cài thủ công của bạn.
- `joystick_bridge` (Python) import sạch.

## Giới hạn đã biết / việc cần làm tiếp

- **Chưa cân bằng chủ động bằng IMU** - `/imu/data` đã được hiệu chuẩn/lọc nhưng controller Stand/Sit chưa dùng nó để đổi lệnh khớp. `axis_map`/`axis_sign` vẫn phải xác nhận trên robot cố định trước; bật feedback với dấu sai có thể gây positive feedback.
- **12 động cơ, mỗi khớp có 1 board driver CAN-FD riêng**: `actuator_if.c` nhận mảng vị trí/vận tốc/Kp/Kd/Tff riêng cho 12 khớp, clamp độc lập rồi đóng gói BabyAlpha2 PD frame. `v_des` được tính từ quỹ đạo IK; Tff được EC tính từ `-J(q)^T F_support`, ramp khi Stand/Sit và luôn về 0 ở Passive/ESTOP. Giá trị scale robot thật được tune trong `controllers_real.yaml`; sim dùng `1.0`.
- **Bit-timing CAN 1Mbit/s + việc dùng chung bus `CAN_INSTANCE_1` (RDK-link CAN FD + 6 động cơ chân trước CAN-FD) chưa đo trên phần cứng thật** - `lib/can.c` mới test loopback nội bộ (không qua đường truyền vật lý), chưa qua bus CAN thật - xem mục "CHƯA kiểm tra" trong `firmware/stm32h7/README.md` trước khi nối vào bus thật.
- **Monitor IMU Tkinter** (`make imu-test`) là đường test cảm biến độc lập, chỉ subscribe
  `/imu/raw`, `/imu/data`, `/diagnostics`; nó không khởi động controller và không gửi lệnh motor.
