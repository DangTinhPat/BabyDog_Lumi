# joystick_bridge

Lớp input người dùng, tách biệt hoàn toàn khỏi việc điều khiển robot là mô phỏng hay
thật - chỉ publish `/control_input` (`controller/msg/Inputs`, field `command`:
`0`=None `1`=Stand `2`=Sit `9`=Estop), không biết gì về Gazebo/CAN/micro-ROS bên dưới.

## 2 node

- **`joystick_input.py`** - subscribe `sensor_msgs/Joy` (do `joy_node` chuẩn của ROS2
  publish, đọc gamepad USB/bluetooth thật qua `/dev/input/js*`). Edge-trigger theo nút
  bấm (không gửi lặp lại mỗi frame `/joy` ~50Hz trong lúc giữ nút). 3 nút:
  - `home_button` - luôn gửi Sit (về "home", nội suy mượt như Sit bình thường, KHÔNG
    phải Estop/Passive buông thả) bất kể đang đứng hay ngồi.
  - `toggle_button` - bấm 1 lần gửi Stand, bấm lại (khi đã đứng) gửi Sit, tự nhớ trạng
    thái nội bộ trong node (không đọc feedback thật từ FSM).
  - `estop_button` - luôn tách riêng khỏi `home_button`/`toggle_button`, bấm là gửi
    Estop ngay bất kể trạng thái hiện tại.

  `home_button` và `estop_button` đều reset trạng thái nội bộ của `toggle_button` về
  "lần bấm tới = Stand" khi được bấm - tránh toggle bị lệch so với thực tế chỉ vì 1
  trong 2 nút kia đã đổi tư thế robot mà không đi qua toggle.
- **`keyboard_input.py`** - tương tự nhưng đọc bàn phím, dùng khi không có tay cầm.

## Cách chạy

```bash
ros2 launch joystick_bridge joystick.launch.py   # gamepad, hoặc `make joystick`
ros2 run joystick_bridge keyboard_input           # bàn phím, hoặc `make keyboard`
```

**Chạy song song** với 1 trong 2 launch có `controller` đang active:
- `sim.launch.py` (Gazebo, `make sim`)
- `real_ros2_control.launch.py` (robot thật)

**Không hoạt động với `rz_sim.launch.py`/`rz_real.launch.py`** (`make rz-sim`/
`make rz-real`) - 2 launch đó chỉ mở RViz (Grid+TF), không chạy
`controller_manager`/`controller` nên không ai subscribe `/control_input` cả;
publish vẫn ra bình thường nhưng rơi vào hư không.

## Đổi nút joystick

Mặc định theo layout tay cầm kiểu Xbox qua `joy_node`: A=home (index 0), B=toggle
(index 1), Back/Select=estop (index 6). Kiểm tra tay cầm khác:

```bash
ros2 topic echo /joy    # bấm từng nút, xem index nào đổi trong mảng "buttons"
ros2 launch joystick_bridge joystick.launch.py home_button:=2 toggle_button:=3 estop_button:=7
```
