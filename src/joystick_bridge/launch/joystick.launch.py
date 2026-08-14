"""Bring up the `joy` package's joy_node (reads the OS gamepad device) plus
joystick_input (translates buttons -> /control_input). Run this alongside
main_bot's sim.launch.py (simulation) or real_ros2_control.launch.py (real hardware) -
same joystick wiring either way.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    device_id = LaunchConfiguration('device_id')
    home_button = LaunchConfiguration('home_button')
    toggle_button = LaunchConfiguration('toggle_button')
    estop_button = LaunchConfiguration('estop_button')

    declare_device_id = DeclareLaunchArgument(
        'device_id', default_value='0',
        description=(
            "Index of /dev/input/js<N> for joy_node's device_id param. "
            'List available pads with `ls /dev/input/js*`.'
        )
    )
    declare_toggle_button = DeclareLaunchArgument(
        'toggle_button', default_value='0',
        description='Bam lan dau -> Ngoi, bam lai -> Dung, doi lien tuc - xem joystick_input.py.'
    )
    declare_home_button = DeclareLaunchArgument(
        'home_button', default_value='1',
        description='Luon gui Sit (ve home) bat ke trang thai hien tai - xem joystick_input.py.'
    )
    declare_estop_button = DeclareLaunchArgument(
        'estop_button', default_value='6',
        description='Dat so am (vd -1) de TAM THOI vo hieu hoa - CHI dung khi ban test khong co '
                     'rui ro that, dung de trong khi chay tren robot that.'
    )

    # ParameterValue(..., value_type=int) forces these to load as integers -
    # joy_node/joystick_input both declare_parameter with int defaults, and a
    # bare LaunchConfiguration is always a string, which rclpy's parameter
    # type checking rejects against an int-typed declared parameter.
    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
        parameters=[{'device_id': ParameterValue(device_id, value_type=int)}],
    )

    joystick_input = Node(
        package='joystick_bridge',
        executable='joystick_input',
        output='screen',
        parameters=[{
            'home_button': ParameterValue(home_button, value_type=int),
            'toggle_button': ParameterValue(toggle_button, value_type=int),
            'estop_button': ParameterValue(estop_button, value_type=int),
        }],
    )

    return LaunchDescription([
        declare_device_id,
        declare_home_button,
        declare_toggle_button,
        declare_estop_button,
        joy_node,
        joystick_input,
    ])
