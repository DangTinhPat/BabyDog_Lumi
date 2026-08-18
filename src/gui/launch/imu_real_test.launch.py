"""Read-only real MPU6050 test: micro-ROS, Kalman filter and Tk monitor only."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start the sensor pipeline without ros2_control or any motor publisher."""
    main_bot_share = get_package_share_directory('main_bot')
    filter_config = os.path.join(main_bot_share, 'config', 'imu_filter.yaml')

    serial_dev = LaunchConfiguration('serial_dev')
    serial_baud = LaunchConfiguration('serial_baud')

    agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='imu_test_micro_ros_agent',
        output='screen',
        emulate_tty=True,
        arguments=['serial', '--dev', serial_dev, '-b', serial_baud],
    )
    imu_filter = Node(
        package='imu_kalman_filter',
        executable='imu_kalman_node',
        name='imu_kalman_filter',
        output='screen',
        emulate_tty=True,
        parameters=[filter_config, {
            'input_source': 'compact',
            'compact_topic': '/imu/raw',
            'output_topic': '/imu/data',
            'use_sim_time': False,
        }],
    )
    monitor = Node(
        package='gui',
        executable='imu_monitor',
        name='imu_monitor',
        output='screen',
        emulate_tty=True,
    )

    stop_pipeline_when_monitor_closes = RegisterEventHandler(
        OnProcessExit(
            target_action=monitor,
            on_exit=[EmitEvent(event=Shutdown(reason='IMU monitor closed'))],
        )
    )

    return LaunchDescription([
        # Firmware micro-ROS is built for domain 0. Keep this first so every
        # process created below inherits the same DDS domain.
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        DeclareLaunchArgument(
            'serial_dev', default_value='/dev/ttyUSB0',
            description='STM32 micro-ROS UART device.'),
        DeclareLaunchArgument(
            'serial_baud', default_value='921600',
            description='Baud rate matching firmware UART1.'),
        stop_pipeline_when_monitor_closes,
        agent,
        imu_filter,
        monitor,
    ])
