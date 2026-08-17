# babyDog

Robot chó 4 chân thật (RDK X5 + STM32H7), giai đoạn cơ bản nhất: **chỉ đứng lên / ngồi xuống**, điều khiển bằng joystick, chưa có IMU. Bao gồm cả mô phỏng (ROS 2 Jazzy + Gazebo Harmonic, để trực quan trước khi có phần cứng/chờ ráp xong) và khung code cho phần cứng thật.

Đây là bản rút gọn có chủ đích từ [`superDog`](../superDog) (dự án mô phỏng đi bộ đầy đủ) - tái sử dụng khung vật lý (devq) + `leg_pd_controller`, nhưng bỏ hết phần đi bộ/gait/cân bằng chủ động/IMU vì giai đoạn này chưa cần và chưa có phần cứng IMU thật.

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
                                    │    vị trí/kp/kd qua      │
                                    │    CAN-FD cho 12    │
                                    │    board driver khớp     │
                                    │    (mỗi khớp tự PD +     │
                                    │    PWM + encoder 6 dây)  │
                                    └─────────────────────────┘
```

Cả 2 nhánh (mô phỏng/thực tế) dùng FSM trong `stand_sit_controller`. Trên robot thật, controller chạy FK/IK rồi gửi lệnh khớp xuống STM32; firmware không tự chạy FSM hay chọn tư thế.

## Cấu trúc thư mục

```
src/
├── main_bot/              Mô tả robot (kế thừa devq từ superDog) + world + launch
├── leg_pd_controller/     PD cấp thấp (copy nguyên từ superDog, không đổi)
├── stand_sit_controller/  FSM Passive/Stand/Sit (bản rút gọn của unitree_guide_controller
│                          - KHÔNG có gait/WaveGenerator/Estimator/BalanceCtrl QP, vì
│                          chưa đi bộ và chưa có IMU để cân bằng chủ động)
├── joystick_bridge/       Node Python: joystick thật (ros2 `joy`) + bàn phím -> /control_input
├── main_bot_hardware/     hardware_interface::SystemInterface cho robot thật - relay
│                          position/kp/kd tới firmware/stm32h7 qua micro-ROS/UART
├── main_bot_hardware_msgs/ JointCmd/JointFb - message giữa main_bot_hardware và firmware
└── gui/                   Bảng điều khiển Tkinter cho mô phỏng (`make gui`) - port từ superDog

firmware/stm32h7/          Firmware STM32H743, không HAL/CMSIS (đăng ký thanh ghi trực tiếp,
                           kế thừa từ ~/OUT_SAVE/babyDog_fwSTM/). Xem firmware/stm32h7/README.md.
```

## Cách chạy nhanh nhất

Xem [GUIDE.md](GUIDE.md) - có `make sim`, `make stand`, `make sit`, v.v. để khỏi phải mở nhiều terminal/nhớ lệnh dài.

## Đã build & test

- `colcon build` sạch (không lỗi) cho cả 5 package ROS2.
- Đã chạy thật `sim.launch.py`, kiểm tra qua CLI: `joint_state_broadcaster`/`leg_pd_controller`/`controller` đều activate thành công; gửi lệnh Stand (1) làm 12 khớp hội tụ về đúng tư thế đứng (~[0, -0.748, 1.495] rad/khớp); gửi Sit (2) hội tụ về tư thế ngồi (~[0, -1.231, 2.462]); gửi Estop (9) không làm controller crash.
- Firmware `firmware/stm32h7` dùng thư viện thanh ghi thật của bạn (`~/OUT_SAVE/babyDog_fwSTM/lib/`, board FK743M5-XIH6) - `lib/can.c` đã test loopback CAN thật trên board thật (7 test case Classic/FD, xem `firmware/stm32h7/README.md`); code ứng dụng (`app/`, `main.c`) build sạch với `arm-none-eabi-gcc 13.2.1` nhưng CHƯA nạp/chạy trên board ở mức toàn hệ thống. Đã sửa 1 lỗi Makefile (thiếu include path `newlib/`) khiến cả project firmware gốc `~/OUT_SAVE/babyDog_fwSTM/` không build được với toolchain cài thủ công của bạn.
- `joystick_bridge` (Python) import sạch.

## Giới hạn đã biết / việc cần làm tiếp

- **Chưa có IMU** (như đề bài) - `controller` trên RDK dùng FK/IK để tạo quỹ đạo bàn chân Cartesian rồi giữ kết quả bằng PD khớp, nhưng KHÔNG cân bằng chủ động khi bị đẩy (khác với `BaseFixedStand` bên superDog, vốn dùng Estimator+QP). Firmware STM32 chỉ nhận các góc do IK sinh ra qua `/joint_cmd`; nếu mất lệnh, watchdog ngắt lực thay vì fallback về góc đặt sẵn.
- **12 động cơ, mỗi khớp có 1 board driver CAN-FD riêng** (tự làm PWM + đọc encoder 6 dây + thuật toán PD cục bộ tại chỗ) - `actuator_if.c` đã gửi lệnh vị trí/kp/kd + nhận feedback qua CAN cho cả 12 khớp (6 trên `CAN_INSTANCE_1` cùng bus RDK-link CAN FD, 6 trên `CAN_INSTANCE_2`), nhưng **giao thức CAN với board driver (`motor_topology.h`) là TỰ ĐỊNH NGHĨA** - chưa có board driver thật để xác nhận/test, và mapping khớp->connector (P1-P12) trong `Motor_BusForJoint()` là giả định theo schematic, cần sửa lại theo cách bạn đấu dây thật.
- **Bit-timing CAN 1Mbit/s + việc dùng chung bus `CAN_INSTANCE_1` (RDK-link CAN FD + 6 động cơ chân trước CAN-FD) chưa đo trên phần cứng thật** - `lib/can.c` mới test loopback nội bộ (không qua đường truyền vật lý), chưa qua bus CAN thật - xem mục "CHƯA kiểm tra" trong `firmware/stm32h7/README.md` trước khi nối vào bus thật.
- **GUI Tkinter** (`src/gui/`) - port nguyên xi từ `superDog/src/gui` (cùng cơ chế start/stop process, log panel, kill/shutdown), chỉ đổi hàng nút FSM: bỏ joystick di chuyển/nút Trot/biểu đồ cân bằng IMU của superDog (không áp dụng được - `stand_sit_controller/msg/Inputs` chỉ có field `command`, không có `lx/ly/rx/ry`, và babydog.xacro chưa bridge `/imu`), thay bằng 3 nút Đứng lên/Ngồi xuống/Estop khớp đúng `/control_input` của babyDog. Chạy `make gui`.
