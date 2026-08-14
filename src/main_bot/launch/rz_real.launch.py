"""RViz DUY NHAT, khong tu publish gi ca - khong robot_state_publisher, khong
joint_state_publisher, khong controller_manager. Chi ngoi cho va hien /tf dang co
tren ROS graph luc do (Grid + TF display, babydog_real.rviz - KHONG co RobotModel:
Description Topic cua no dung QoS Volatile, mismatch voi robot_state_publisher
publish /robot_description kieu transient_local 1 lan duy nhat luc khoi dong - de
dung hinh vinh vien neu rviz mo tre hon. TF display khong can /robot_description,
chi can /tf/tf_static nen khong bi anh huong).

Cap doi voi rz_sim.launch.py (dung script, chi khac use_sim_time mac dinh) - noi
dung y het nhau luc moi mo (Grid trong, khong co robot) - chi khac o CHUC NANG:
ban nay dung voi `make real`/real_ros2_control.launch.py (dong ho he thong that,
khong /clock). Chay SONG SONG voi `make real` o terminal khac.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, UnsetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# Snap-confined apps leak SNAP_*/GTK_*/XDG_DATA_* vao terminal sinh ra tu chung, lam
# rviz2 (Qt) crash voi loi symbol lookup (libpthread.so.0 tu core20 snap) - cung fix
# nhu sim.launch.py/real_ros2_control.launch.py.
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
    rviz_config = PathJoinSubstitution([pkg_main_bot, 'rviz', 'babydog_real.rviz'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='false (mac dinh) - dong ho he thong that, khong /clock.'
    )

    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    unset_snap_vars = [UnsetEnvironmentVariable(var) for var in _SNAP_LEAK_VARS]

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    for action in unset_snap_vars:
        ld.add_action(action)
    ld.add_action(node_rviz2)

    return ld
