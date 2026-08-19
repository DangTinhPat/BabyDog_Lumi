# Guide thao tác nhanh

Tất cả lệnh chạy từ thư mục gốc `babyDog/`. Lần đầu tiên phải `make build` trước.

## 1. Build

```bash
make build
```

## 2. Mô phỏng (trên máy dev, cần ROS2 Jazzy + Gazebo Harmonic đã cài)

Mở 2 terminal:

**Terminal 1** - Gazebo + RViz + bộ điều khiển:
```bash
make sim              # có RViz
make sim-no-rviz       # không mở RViz, đỡ tốn tài nguyên
```

**Terminal 2** - điều khiển đứng/ngồi, chọn 1 trong các cách:

```bash
make keyboard          # bàn phím: '1'=đứng, '2'=ngồi, '0'=estop (ngắt lực)
make joystick          # joystick/gamepad thật cắm USB - xem "Đổi nút joystick" bên dưới
make stand             # gửi lệnh đứng 1 lần qua CLI, không cần joystick (test nhanh)
make sit               # gửi lệnh ngồi 1 lần
make estop             # ngắt lực khẩn cấp
```

Hoặc bỏ qua cả 2 terminal trên, dùng 1 cửa sổ duy nhất:

```bash
make gui                # bảng điều khiển Tkinter: nút Start sim/RViz, Đứng lên/Ngồi xuống/
                         # Estop và hàng IMU-only, xem log, Kill traces, Tắt hết & Thoát
```

Kiểm tra controller đã chạy chưa: `make controllers` (phải thấy 3 dòng "active").

Muốn mở/đóng riêng cửa sổ RViz mà không restart cả sim: `make sim-no-rviz` rồi
`make rz-sim` (terminal khác) - chỉ Grid+TF, bấm TF sẽ hiện đúng trạng thái sim
đang chạy. Tương tự với robot thật: `make real-no-rviz` + `make rz-real`.

Tắt hết Gazebo còn sót (nếu Ctrl+C không sạch): `make kill`.

### Đổi nút joystick

3 nút: `home_button` (luôn gửi Ngồi/về home, bất kể đang ở trạng thái nào),
`toggle_button` (bấm 1 lần = đứng lên, bấm lại = ngồi xuống, tự nhớ trạng thái),
`estop_button` (luôn tách riêng, bấm là ngắt lực ngay). Mặc định: A=home (index 0),
B=toggle (index 1), Back/Select=estop (index 6) - theo layout tay cầm kiểu Xbox qua
`joy_node`. Nếu tay cầm khác layout:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 topic echo /joy    # bấm từng nút, xem index nào đổi trong mảng "buttons"
ros2 launch joystick_bridge joystick.launch.py home_button:=2 toggle_button:=3 estop_button:=7
```

## 3. Phần cứng thật (RDK X5/laptop)

Điều khiển khớp qua `ros2_control` (`main_bot_hardware/RealSystem`), relay tới
`firmware/stm32h7` qua micro-ROS/UART - xem `src/main_bot/launch/real_ros2_control.launch.py`
để biết chi tiết (yêu cầu firmware đã nạp, UART1 nối `micro_ros_agent`, workspace có
sẵn `micro_ros_agent` đã source):

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch main_bot real_ros2_control.launch.py serial_dev:=/dev/ttyUSB0
make keyboard           # hoặc joystick/stand/sit - HỆT như mô phỏng, cùng /control_input
```

## 4. Firmware STM32H7

```bash
make microros-lib      # bắt buộc sau khi đổi/thêm message dùng bởi firmware
make firmware-test     # unit test codec CAN trên máy, không chạm phần cứng
make firmware          # build (cần arm-none-eabi-gcc trong PATH - đã có sẵn qua ~/tools/arm-none-eabi-toolchain, xem .bashrc)
make firmware-flash     # build + nạp qua ST-Link (cần stlink-tools + ST-Link thật)
make firmware-clean
```

Đọc `firmware/stm32h7/README.md` **trước khi nối vào phần cứng thật** - có ghi rõ phần nào đã đối chiếu tài liệu ST, phần nào (bit-timing thật, giao thức động cơ) chưa đo/chưa làm.

## 5. Kiểm tra MPU6050 và trục IMU thật

Kalman chạy trên EC, không chạy trong STM32. Firmware chỉ đọc MPU6050 qua I2C1
(`PB8=SCL`, `PB7=SDA`, địa chỉ `0x68`) và gửi bản tin fixed-size `/imu/raw`.
Robot phải nằm/cố định hoàn toàn trong khoảng 2 giây đầu để thu đủ 200 mẫu bias gyro.

Sau khi build, regenerate micro-ROS và flash firmware mới, chạy pipeline IMU độc lập:

```bash
make build
make microros-lib       # bắt buộc sau khi thêm ImuRaw lần đầu
make firmware-flash    # có --reset, để MCU thật chạy firmware IMU mới
make imu-test
```

`make imu-test` chỉ mở `micro_ros_agent`, `imu_kalman_filter` và cửa sổ
`imu_monitor`; nó không mở `ros2_control`, không tạo `/joint_cmd` hay `/control_input`, vì vậy
không điều khiển động cơ. Trước khi chạy, script tự kiểm tra `/dev/ttyUSB0` chưa bị giữ và từ chối
khởi động nếu phát hiện agent/stack robot thật khác. Đóng cửa sổ sẽ dừng cả agent lẫn filter.

Nếu STM32 ở cổng khác:

```bash
make imu-test IMU_SERIAL_DEV=/dev/ttyUSB1 IMU_SERIAL_BAUD=921600
```

Màn hình hiển thị song song:

- `/imu/raw`: số nguyên trên wire, giá trị SI, status, tần số và độ trễ;
- `/imu/data`: accel/gyro đã Kalman, quaternion, roll/pitch/yaw, covariance, tần số và độ trễ;
- `/diagnostics`: tiến độ calibration, gyro bias, số mẫu nhận/publish/reject;
- log ghép RAW + FILTERED ở 5 Hz để quan sát mà không làm nghẽn GUI.

Nếu pipeline đã được mở bằng cách khác, `make imu-monitor` chỉ mở thêm màn hình read-only.
Kiểm tra bằng CLI vẫn có thể dùng trong terminal khác (nhớ `export ROS_DOMAIN_ID=0`):

```bash
ros2 topic hz /imu/raw          # kỳ vọng xấp xỉ 100 Hz
ros2 topic hz /imu/data         # xuất hiện sau khi stationary calibration xong
ros2 topic echo /diagnostics    # babyDog/imu_kalman phải chuyển sang OK
```

Có thể làm toàn bộ từ `make gui`: nhập `Serial`/`Baud` ở hàng Real rồi bấm
**Start IMU-only**. GUI tự khóa chéo các nút Start Sim/Real/Joystick trong lúc test IMU;
đóng màn hình IMU hoặc bấm **Stop IMU-only** sẽ dừng agent/filter và mở lại các nút đó.

Xác nhận mapping vật lý khi robot vẫn được treo/cố định:

- đặt thân cân bằng: acceleration x/y gần 0, z gần `+9.81 m/s²`, roll/pitch gần 0;
- nâng phía trái thân: roll phải tăng dương;
- nâng mũi robot: pitch phải tăng dương;
- quay ngược chiều kim đồng hồ khi nhìn từ trên: `angular_velocity.z` phải dương.

Nếu sai, chỉ sửa `axis_map`/`axis_sign` trong `src/main_bot/config/imu_filter.yaml`, rồi
`make build` và kiểm tra lại. Không sửa `MOTOR_JOINT_SIGN`, không đổi dấu trong firmware IMU.
IMU hiện chưa tác động lệnh motor; giữ nguyên như vậy cho tới khi cả ba phép thử trục/dấu đều qua.

Trong mô phỏng, `make sim` tự bridge `/imu/sim_raw` và chạy đúng cùng node Kalman ra `/imu/data`.

## 6. Quy ước lệnh /control_input (dùng chung mô phỏng + thực tế)

| command | Ý nghĩa |
|---------|---------|
| 0       | Không làm gì (giữ nguyên trạng thái) |
| 1       | Đứng lên |
| 2       | Ngồi xuống |
| 9       | Estop - ngắt lực khẩn cấp (motor về passive/mềm) |

Gửi tay 1 lệnh bất kỳ (ngoài `make stand/sit/estop`):
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 topic pub -1 /control_input stand_sit_controller/msg/Inputs "{command: 1}"
```
