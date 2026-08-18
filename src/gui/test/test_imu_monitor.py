import math

from diagnostic_msgs.msg import DiagnosticStatus
from gui.imu_monitor import (
    diagnostic_level_value,
    quaternion_to_rpy_degrees,
    raw_status_text,
    RollingRate,
)
from main_bot_hardware_msgs.msg import ImuRaw
import pytest


def test_quaternion_to_rpy_degrees_normalizes_input():
    half_angle = math.radians(15.0) / 2.0
    roll, pitch, yaw = quaternion_to_rpy_degrees(
        2.0 * math.sin(half_angle), 0.0, 0.0, 2.0 * math.cos(half_angle))
    assert roll == pytest.approx(15.0)
    assert pitch == pytest.approx(0.0)
    assert yaw == pytest.approx(0.0)


def test_rolling_rate_uses_recent_monotonic_window():
    rate = RollingRate(window_seconds=2.0)
    for index in range(201):
        rate.add(index * 0.01)
    assert rate.hz() == pytest.approx(100.0)


def test_raw_status_text_exposes_errors():
    assert raw_status_text(ImuRaw.STATUS_OK) == 'OK'
    status = ImuRaw.STATUS_INIT_FAILED | ImuRaw.STATUS_READ_FAILED
    assert raw_status_text(status) == 'INIT_FAILED|READ_FAILED'


def test_diagnostic_level_handles_ros_byte_field():
    assert diagnostic_level_value(DiagnosticStatus.OK) == 0
    assert diagnostic_level_value(DiagnosticStatus.WARN) == 1
