from gui.main_window import SimControlGui


def gui_without_tk():
    gui = SimControlGui.__new__(SimControlGui)
    gui.procs = {
        'sim': None,
        'rviz': None,
        'joystick': None,
        'real': None,
        'rviz_real': None,
        'imu_view': None,
    }
    return gui


def test_imu_view_command_opens_monitor_without_its_own_agent():
    gui = gui_without_tk()
    assert gui._command_for('imu_view') == ['ros2', 'run', 'gui', 'imu_monitor']
