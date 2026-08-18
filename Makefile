# Makefile tiện ích - chạy nhanh không cần nhớ lệnh ros2 launch/run dài dòng
# hay mở nhiều terminal. Xem GUIDE.md để biết chi tiết từng lệnh.
#
# Quy ước: mọi target ROS2 tự source /opt/ros/jazzy/setup.bash + install/setup.bash,
# không cần bạn source tay trước (nhưng phải `make build` ít nhất 1 lần trước).

ROS_DISTRO ?= jazzy
MROS_WS ?= $(HOME)/mros/mros_ws
IMU_SERIAL_DEV ?= /dev/ttyUSB0
IMU_SERIAL_BAUD ?= 921600
SHELL := /bin/bash
ROS_SETUP := source /opt/ros/$(ROS_DISTRO)/setup.bash && source install/setup.bash
MROS_SETUP := source /opt/ros/$(ROS_DISTRO)/setup.bash && source $(MROS_WS)/install/setup.bash

.PHONY: help build sim sim-no-rviz rz-sim real real-no-rviz rz-real gui keyboard joystick \
        imu-test imu-monitor firmware firmware-flash microros-lib controllers estop stand sit \
        clean firmware-clean kill

help:
	@echo "Mô phỏng (chạy trên máy dev, cần ROS2 Jazzy + Gazebo Harmonic):"
	@echo "  make build          - colcon build workspace ROS2"
	@echo "  make sim            - Gazebo + RViz + bộ điều khiển đứng/ngồi"
	@echo "  make sim-no-rviz    - giống trên nhưng không mở RViz"
	@echo "  make rz-sim         - CHỈ mở RViz (Grid+TF, không có robot bên trong lúc mở) -"
	@echo "                        chạy song song với make sim-no-rviz (terminal khác), bấm"
	@echo "                        TF sẽ hiện đúng trạng thái sim đang chạy"
	@echo "  make gui            - GUI chung: sim/real/FSM và hàng test IMU-only raw/filtered"
	@echo ""
	@echo "Robot thật (cần firmware đã nạp + micro_ros_agent workspace đã source, xem GUIDE.md):"
	@echo "  make real           - robot thật + RViz (TF2 cập nhật từ /joint_states thật)"
	@echo "  make real-no-rviz   - giống trên nhưng không mở RViz (vd chạy qua SSH không màn hình)"
	@echo "  make rz-real        - CHỈ mở RViz (Grid+TF, không có robot bên trong lúc mở) -"
	@echo "                        chạy song song với make real-no-rviz (terminal khác), bấm"
	@echo "                        TF sẽ hiện đúng trạng thái robot thật hiện tại"
	@echo "  make keyboard       - điều khiển bằng bàn phím ('1'=đứng,'2'=ngồi,'0'=estop)"
	@echo "  make joystick       - joy_node + joystick thật -> /control_input"
	@echo "  make stand / sit / estop - gửi lệnh 1 lần qua CLI (test nhanh không cần joystick)"
	@echo "  make controllers    - liệt kê trạng thái controller_manager"
	@echo "  make imu-test       - test MPU6050 thật độc lập: micro-ROS + Kalman + màn hình log"
	@echo "  make imu-monitor    - chỉ mở màn hình raw/filtered trên pipeline IMU đang chạy"
	@echo "  make kill           - tắt hết tiến trình Gazebo còn sót"
	@echo ""
	@echo "Firmware STM32H7 (cần arm-none-eabi-gcc, xem firmware/stm32h7/README.md):"
	@echo "  make microros-lib    - regenerate lib/type-support MCU sau khi đổi JointCmd/JointFb/ImuRaw"
	@echo "  make firmware       - build firmware/stm32h7"
	@echo "  make firmware-flash - build + nạp qua ST-Link (st-flash)"
	@echo "  make firmware-clean - xoá build firmware"
	@echo ""
	@echo "  make clean          - xoá build/install/log của workspace ROS2"

build:
	$(ROS_SETUP) && colcon build --symlink-install

sim:
	$(ROS_SETUP) && ros2 launch main_bot sim.launch.py

sim-no-rviz:
	$(ROS_SETUP) && ros2 launch main_bot sim.launch.py rviz:=false

rz-sim:
	$(ROS_SETUP) && ros2 launch main_bot rz_sim.launch.py

real:
	$(ROS_SETUP) && ros2 launch main_bot real_ros2_control.launch.py

real-no-rviz:
	$(ROS_SETUP) && ros2 launch main_bot real_ros2_control.launch.py rviz:=false

rz-real:
	$(ROS_SETUP) && ros2 launch main_bot rz_real.launch.py

gui:
	$(ROS_SETUP) && ros2 run gui gui

keyboard:
	$(ROS_SETUP) && ros2 run joystick_bridge keyboard_input

joystick:
	$(ROS_SETUP) && ros2 launch joystick_bridge joystick.launch.py

stand:
	$(ROS_SETUP) && ros2 topic pub -1 /control_input controller/msg/Inputs "{command: 1}"

sit:
	$(ROS_SETUP) && ros2 topic pub -1 /control_input controller/msg/Inputs "{command: 2}"

estop:
	$(ROS_SETUP) && ros2 topic pub -1 /control_input controller/msg/Inputs "{command: 9}"

controllers:
	$(ROS_SETUP) && ros2 control list_controllers

imu-test:
	BABYDOG_MROS_WS="$(MROS_WS)" bash src/gui/scripts/imu_real_test.sh \
		"$(IMU_SERIAL_DEV)" "$(IMU_SERIAL_BAUD)"

imu-monitor:
	export ROS_DOMAIN_ID=0 && $(ROS_SETUP) && ros2 run gui imu_monitor

kill:
	bash src/main_bot/scripts/kill_gz.sh

firmware:
	$(MAKE) -C firmware/stm32h7

microros-lib:
	cd firmware/stm32h7/microros && $(MROS_SETUP) && \
		ros2 run micro_ros_setup build_firmware.sh -- \
		$(CURDIR)/firmware/stm32h7/microros/toolchain.cmake \
		$(CURDIR)/firmware/stm32h7/microros/firmware/mcu_ws/colcon.meta

firmware-flash:
	$(MAKE) -C firmware/stm32h7 flash

firmware-clean:
	$(MAKE) -C firmware/stm32h7 clean

clean:
	rm -rf build install log
