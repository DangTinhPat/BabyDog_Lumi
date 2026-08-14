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
                         # Estop, xem log, Kill traces, Tắt hết & Thoát - không cần nhớ lệnh
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
make firmware          # build (cần arm-none-eabi-gcc trong PATH - đã có sẵn qua ~/tools/arm-none-eabi-toolchain, xem .bashrc)
make firmware-flash     # build + nạp qua ST-Link (cần stlink-tools + ST-Link thật)
make firmware-clean
```

Đọc `firmware/stm32h7/README.md` **trước khi nối vào phần cứng thật** - có ghi rõ phần nào đã đối chiếu tài liệu ST, phần nào (bit-timing thật, giao thức động cơ) chưa đo/chưa làm.

## 5. Quy ước lệnh /control_input (dùng chung mô phỏng + thực tế)

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
