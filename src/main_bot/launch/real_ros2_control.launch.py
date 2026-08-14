"""Chạy ros2_control THẬT (không mô phỏng) trên robot: robot_state_publisher +
controller_manager (ros2_control_node, KHÔNG qua Gazebo - khác hẳn sim.launch.py nơi
gz_ros2_control's plugin tự host controller_manager bên trong process Gazebo) + spawner
joint_state_broadcaster/controller, dùng main_bot_hardware/RealSystem
(babydog.xacro hardware:=real) relay qua micro-ROS/UART ("/joint_cmd"/"/joint_fb",
main_bot_hardware_msgs) tới firmware/stm32h7 - xem README.md. Tự khởi chạy luôn
micro_ros_agent (serial bridge UART<->DDS) - PHẢI source workspace có sẵn agent trước
khi chạy file này (vd `source ~/mros/mros_ws/install/setup.bash`, xem
~/OUT_SAVE/testSTM/README.md - micro_ros_agent KHÔNG nằm trong workspace babyDog).

LƯU Ý DOMAIN ID (bug thật đã gặp lúc bring-up, mất nhiều thời gian mới tìm ra):
DomainParticipant mà micro_ros_agent dùng để bridge XRCE-DDS sang ROS2 graph thật LẤY
DOMAIN THEO CHÍNH PHIÊN CLIENT (xem micro_ros_agent's Agent.cpp:
find_or_create_graph_manager(participant->get_domain_id())) - tức domain do FIRMWARE MCU
quyết định lúc build thư viện micro-ROS (mặc định chuẩn = 0, xem firmware/stm32h7/microros/),
KHÔNG đọc biến môi trường ROS_DOMAIN_ID của agent hay bất kỳ cờ CLI nào ("--discovery"/"-d"
chỉ áp dụng cho transport mạng UDP/TCP, agent tự cảnh báo "Not supported on selected
transport" khi dùng serial như ở đây - đã thử và không có tác dụng). Vì domain phía MCU cố
định = 0 và không đổi được từ máy tính (trừ khi build lại firmware), giải pháp đúng là ép
CHÍNH launch file này (và mọi tiến trình nó sinh ra: agent/robot_state_publisher/
ros2_control_node/spawner) LUÔN chạy domain 0 - bất kể shell gọi file này đang có
ROS_DOMAIN_ID gì - để không ai phải nhớ "export ROS_DOMAIN_ID=0" trước mỗi lần chạy nữa.
GUI (src/gui/gui/main_window.py) áp dụng đúng nguyên tắc tương tự cho nhánh joystick.

KHÔNG cần leg_pd_controller (board driver mỗi khớp tự làm PD cục bộ) - xem
controllers_real.yaml. joystick_bridge (không đổi, /control_input y hệt sim) vẫn dùng
được để điều khiển - `ros2 launch joystick_bridge joystick.launch.py` hay
`ros2 run joystick_bridge keyboard_input` ở terminal khác, giống hệt hướng dẫn sim
trong GUIDE.md.

firmware/stm32h7 vẫn giữ protocol CAN Stand/Sit/Estop cũ (CMD_CAN_ID/STATUS_CAN_ID,
stand_sit_fsm.c) làm fallback nội bộ - nếu /joint_cmd mất liên kết (JOINT_LINK_TIMEOUT_MS,
protocol.h), firmware tự rơi về FSM cục bộ. Node ROS2 Python từng forward lệnh qua đường
CAN đó (fdcan_bridge) đã bị xoá vì không còn dùng - đường ros2_control ở đây đã thay thế
hoàn toàn vai trò của nó.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, UnsetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Snap-confined apps (vd VS Code cai qua snap) leak SNAP_*/GTK_*/XDG_DATA_* vao
# terminal sinh ra tu chung. Khi co mat, dynamic linker cua Qt app (rviz2 o day)
# lay nham libpthread.so.0 khong tuong thich tu core20 snap, crash voi loi symbol
# lookup - cung fix nhu sim.launch.py/rz_sim.launch.py/rz_real.launch.py.
_SNAP_LEAK_VARS = [
    'SNAP', 'SNAP_LIBRARY_PATH', 'SNAP_NAME', 'SNAP_DATA', 'SNAP_USER_DATA',
    'SNAP_USER_COMMON', 'SNAP_COMMON', 'SNAP_ARCH', 'SNAP_REVISION',
    'SNAP_INSTANCE_NAME', 'SNAP_CONTEXT', 'SNAP_COOKIE', 'SNAP_REAL_HOME',
    'SNAP_EUID', 'SNAP_UID', 'SNAP_LAUNCHER_ARCH_TRIPLET', 'SNAP_VERSION',
    'GTK_PATH', 'GTK_EXE_PREFIX', 'GTK_IM_MODULE_FILE',
    'GDK_PIXBUF_MODULE_FILE', 'GDK_PIXBUF_MODULEDIR', 'GIO_MODULE_DIR',
    'GSETTINGS_SCHEMA_DIR', 'LOCPATH', 'XDG_DATA_DIRS', 'XDG_DATA_HOME',
]


def generate_launch_description():
    pkg_main_bot = get_package_share_directory('main_bot')

    xacro_file = PathJoinSubstitution(
        [pkg_main_bot, 'description', 'robot.urdf.xacro']
    )
    controllers_yaml = os.path.join(pkg_main_bot, 'config', 'controllers_real.yaml')
    # File RIENG cho robot that (khong dung chung babydog.rviz voi sim) - CHI Grid + TF
    # display, KHONG co RobotModel (khong can load URDF/mesh trong rviz de xem TF -
    # RobotModel's "Description Topic" trong babydog.rviz con bi mismatch QoS
    # (Volatile) so voi robot_state_publisher publish /robot_description dang
    # transient_local 1 lan duy nhat luc khoi dong - neu rviz mo cham hon 1 chut se
    # KHONG BAO GIO nhan duoc URDF, RobotModel dung im vinh vien; hoan toan khong anh
    # huong TF display vi TF chi can /tf, /tf_static, khong lien quan URDF). Muon xem
    # mesh 3D that su thi mo babydog.rviz thu cong roi doi Description Topic QoS sang
    # Transient Local.
    rviz_config = PathJoinSubstitution([pkg_main_bot, 'rviz', 'babydog_real.rviz'])

    serial_dev = LaunchConfiguration('serial_dev')
    declare_serial_dev = DeclareLaunchArgument(
        'serial_dev', default_value='/dev/ttyUSB0',
        description=(
            'Cong serial (CH340) noi toi UART1 cua firmware/stm32h7 (PA9/PA10, xem '
            'BOARD_FK743M5-XIH6.md) - truyen thang cho micro_ros_agent, main_bot_hardware/'
            'RealSystem KHONG can biet gia tri nay (chi la 1 publisher/subscriber ROS2 '
            'binh thuong, micro_ros_agent moi la noi thuc su noi UART<->DDS).'
        )
    )
    serial_baud = LaunchConfiguration('serial_baud')
    declare_serial_baud = DeclareLaunchArgument(
        'serial_baud', default_value='921600',
        description='PHAI khop UART_BAUDRATE trong firmware/stm32h7/main.c.'
    )

    rviz = LaunchConfiguration('rviz')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description=(
            'Mo RViz cung luc - hien TF2 CAP NHAT TU ROBOT THAT (robot_state_publisher '
            'o duoi da subscribe /joint_states thuc). Pass rviz:=false de bo qua (vd chay '
            'khong man hinh/qua SSH, roi mo rieng qua `make rz-real` khi can).'
        )
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' hardware:=real']),
        value_type=str
    )

    # Ep domain 0 cho MOI tien trinh file nay sinh ra (xem giai thich o dau file) - phai la
    # action DAU TIEN trong LaunchDescription (ben duoi) de co hieu luc truoc khi bat ky
    # Node nao khac khoi dong.
    set_domain_zero = SetEnvironmentVariable('ROS_DOMAIN_ID', '0')

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', serial_dev, '-b', serial_baud],
    )

    # use_sim_time=False TUONG MINH (khac sim.launch.py) - khong co /clock topic nao
    # tren robot that (khong qua Gazebo), dung dong ho he thong that. Day la gia tri
    # MAC DINH cua moi node ROS2 khong khai bao gi ca nen ve chuc nang khong doi neu bo
    # dong nay - nhung khai bao tuong minh de tranh nham lan neu sau nay co ai copy 1
    # parameter file tu sim sang (vd lo copy use_sim_time:=true) - lam TF2/rviz cho
    # /clock khong bao gio toi, dung hinh khong hien gi ca.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
    )

    # Subscribe /joint_states (tu joint_state_broadcaster <- RealSystem::read() <-
    # /joint_fb THAT) - TF2 hien thi day la TRANG THAI DO DUOC THUC SU cua robot. File
    # rviz RIENG (babydog_real.rviz, xem comment o rviz_config) - CHI Grid + TF display,
    # chu dong hung /tf truc tiep, khong phu thuoc /robot_description topic (tranh QoS
    # mismatch, xem comment tren).
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(rviz),
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description}, controllers_yaml],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--switch-timeout', '30'],
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['controller', '--switch-timeout', '30'],
    )

    # Khong can chain qua OnProcessExit nhu sim.launch.py (khong co leg_pd_controller
    # o day nen khong co rang buoc thu tu "controller nay phai active truoc controller
    # kia") - moi spawner tu doi service cua controller_manager xuat hien (hanh vi co
    # san cua `spawner`, xem comment trong sim.launch.py), an toan chay song song.
    unset_snap_vars = [UnsetEnvironmentVariable(var) for var in _SNAP_LEAK_VARS]

    return LaunchDescription([
        set_domain_zero,
        declare_serial_dev,
        declare_serial_baud,
        declare_rviz,
        *unset_snap_vars,
        micro_ros_agent,
        robot_state_publisher,
        rviz_node,
        controller_manager,
        joint_state_broadcaster_spawner,
        controller_spawner,
    ])
