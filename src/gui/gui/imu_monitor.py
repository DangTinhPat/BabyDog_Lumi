"""
Standalone real-IMU monitor for the babyDog micro-ROS pipeline.

This process is read-only: it subscribes to compact MCU data on ``/imu/raw``,
filtered ``sensor_msgs/Imu`` data on ``/imu/data`` and Kalman diagnostics.  It
never creates a ``/joint_cmd`` or ``/control_input`` publisher.
"""

from collections import deque
import math
import queue
import signal
import threading
import time
import tkinter as tk
from tkinter import scrolledtext, ttk

from diagnostic_msgs.msg import DiagnosticArray
from main_bot_hardware_msgs.msg import ImuRaw
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


BG = '#171a1f'
PANEL = '#22262d'
ENTRY = '#0f1115'
FG = '#e6edf3'
MUTED = '#9da7b3'
GREEN = '#2ea043'
YELLOW = '#d29922'
RED = '#da3633'
BLUE = '#1f6feb'

STALE_SECONDS = 0.5
LOG_INTERVAL_SECONDS = 0.2
MAX_LOG_LINES = 2000


def quaternion_to_rpy_degrees(x, y, z, w):
    """Return normalized quaternion roll/pitch/yaw in degrees."""
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        return (math.nan, math.nan, math.nan)
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-12:
        return (math.nan, math.nan, math.nan)
    x, y, z, w = (value / norm for value in values)

    sin_roll_cos_pitch = 2.0 * (w * x + y * z)
    cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll_cos_pitch, cos_roll_cos_pitch)

    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)

    sin_yaw_cos_pitch = 2.0 * (w * z + x * y)
    cos_yaw_cos_pitch = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)
    scale = 180.0 / math.pi
    return (roll * scale, pitch * scale, yaw * scale)


def raw_status_text(status):
    """Decode ImuRaw's bit-mask status without hiding unknown bits."""
    if status == ImuRaw.STATUS_OK:
        return 'OK'
    labels = []
    if status & ImuRaw.STATUS_INIT_FAILED:
        labels.append('INIT_FAILED')
    if status & ImuRaw.STATUS_READ_FAILED:
        labels.append('READ_FAILED')
    known = ImuRaw.STATUS_OK | ImuRaw.STATUS_INIT_FAILED | ImuRaw.STATUS_READ_FAILED
    if status & ~known:
        labels.append(f'UNKNOWN(0x{status & ~known:02X})')
    return '|'.join(labels) if labels else f'INVALID(0x{status:02X})'


def diagnostic_level_value(level):
    """Convert diagnostic_msgs/DiagnosticStatus byte level to an integer."""
    if isinstance(level, (bytes, bytearray)):
        return level[0] if level else 0
    return int(level)


class RollingRate:
    """Small monotonic-time rate estimator suitable for 100 Hz topic display."""

    def __init__(self, window_seconds=2.0):
        self.window_seconds = window_seconds
        self.samples = deque()

    def add(self, timestamp):
        self.samples.append(timestamp)
        cutoff = timestamp - self.window_seconds
        while len(self.samples) > 2 and self.samples[0] < cutoff:
            self.samples.popleft()

    def hz(self):
        if len(self.samples) < 2:
            return 0.0
        elapsed = self.samples[-1] - self.samples[0]
        return (len(self.samples) - 1) / elapsed if elapsed > 0.0 else 0.0


class ImuMonitorNode(Node):
    """Thread-safe subscriber backing the Tk display."""

    def __init__(self):
        super().__init__('imu_monitor')
        self._lock = threading.Lock()
        self._events = queue.Queue()
        self._raw = None
        self._filtered = None
        self._diagnostic = None
        self._raw_rate = RollingRate()
        self._filtered_rate = RollingRate()
        self._last_raw_status = None
        self._last_filter_state = None

        self.create_subscription(
            ImuRaw, '/imu/raw', self._raw_callback, qos_profile_sensor_data)
        self.create_subscription(
            Imu, '/imu/data', self._filtered_callback, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, '/diagnostics', self._diagnostic_callback, 10)

    @property
    def events(self):
        return self._events

    def _raw_callback(self, message):
        received = time.monotonic()
        accel_integer = tuple(message.linear_acceleration_milli_ms2)
        gyro_integer = tuple(message.angular_velocity_mrad_s)
        status = int(message.status)
        with self._lock:
            first_sample = self._raw is None
            self._raw_rate.add(received)
            self._raw = {
                'received': received,
                'rate': self._raw_rate.hz(),
                'stamp_ms': int(message.stamp_ms),
                'status': status,
                'status_text': raw_status_text(status),
                'accel_integer': accel_integer,
                'gyro_integer': gyro_integer,
                'accel': tuple(value / 1000.0 for value in accel_integer),
                'gyro': tuple(value / 1000.0 for value in gyro_integer),
            }
            status_changed = status != self._last_raw_status
            self._last_raw_status = status
        if first_sample:
            self._events.put('Đã nhận mẫu /imu/raw đầu tiên từ STM32.')
        if status_changed:
            self._events.put(f'Trạng thái raw: {raw_status_text(status)}.')

    def _filtered_callback(self, message):
        received = time.monotonic()
        quaternion = (
            message.orientation.x, message.orientation.y,
            message.orientation.z, message.orientation.w)
        with self._lock:
            first_sample = self._filtered is None
            self._filtered_rate.add(received)
            self._filtered = {
                'received': received,
                'rate': self._filtered_rate.hz(),
                'stamp': message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                'frame_id': message.header.frame_id,
                'quaternion': quaternion,
                'rpy': quaternion_to_rpy_degrees(*quaternion),
                'accel': (
                    message.linear_acceleration.x,
                    message.linear_acceleration.y,
                    message.linear_acceleration.z),
                'gyro': (
                    message.angular_velocity.x,
                    message.angular_velocity.y,
                    message.angular_velocity.z),
                'orientation_covariance': tuple(message.orientation_covariance),
            }
        if first_sample:
            self._events.put('Kalman đã hiệu chuẩn xong; bắt đầu nhận /imu/data.')

    def _diagnostic_callback(self, message):
        for status in message.status:
            if status.name != 'babyDog/imu_kalman':
                continue
            values = {item.key: item.value for item in status.values}
            state = values.get('state', 'unknown')
            diagnostic = {
                'received': time.monotonic(),
                'level': diagnostic_level_value(status.level),
                'message': status.message,
                'state': state,
                'calibration_count': values.get('calibration_samples', '0'),
                'calibration_target': values.get('calibration_target', '200'),
                'received_samples': values.get('received_samples', '0'),
                'published_samples': values.get('published_samples', '0'),
                'rejected_samples': values.get('rejected_samples', '0'),
                'gyro_bias': values.get('gyro_bias_rad_s', '-'),
            }
            with self._lock:
                state_changed = state != self._last_filter_state
                self._diagnostic = diagnostic
                self._last_filter_state = state
            if state_changed:
                self._events.put(f'Kalman state: {state} — {status.message}.')
            break

    def snapshot(self):
        with self._lock:
            return {
                'raw': dict(self._raw) if self._raw is not None else None,
                'filtered': dict(self._filtered) if self._filtered is not None else None,
                'diagnostic': (
                    dict(self._diagnostic) if self._diagnostic is not None else None),
            }


class ImuMonitorWindow:
    """Tk view. ROS callbacks stay on a dedicated rclpy spin thread."""

    def __init__(self, root, node, spin_thread):
        self.root = root
        self.node = node
        self.spin_thread = spin_thread
        self.closed = False
        self.log_paused = False
        self.last_logged_stamp = None
        self.last_periodic_log = 0.0
        self.log_lines = 0
        self.after_id = None

        root.title('babyDog — IMU thật qua micro-ROS')
        root.geometry('1120x820')
        root.minsize(920, 680)
        root.configure(bg=BG)
        self._configure_style()
        self._build_widgets()
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.after_id = root.after(100, self._refresh)

    def _configure_style(self):
        style = ttk.Style(self.root)
        try:
            style.theme_use('clam')
        except tk.TclError:
            pass
        style.configure('TFrame', background=BG)
        style.configure('TLabelframe', background=BG, bordercolor='#343a43')
        style.configure('TLabelframe.Label', background=BG, foreground=FG,
                        font=('TkDefaultFont', 11, 'bold'))
        style.configure('TLabel', background=BG, foreground=FG)
        style.configure('Title.TLabel', font=('TkDefaultFont', 16, 'bold'))
        style.configure('Muted.TLabel', foreground=MUTED)
        style.configure('TButton', background=PANEL, foreground=FG, padding=(12, 7))

    def _build_widgets(self):
        header = ttk.Frame(self.root, padding=(16, 12))
        header.pack(fill='x')
        ttk.Label(header, text='IMU MPU6050 — micro-ROS + Kalman',
                  style='Title.TLabel').pack(side='left')
        tk.Label(
            header, text='READ-ONLY · KHÔNG GỬI /joint_cmd', bg=GREEN, fg='white',
            font=('TkDefaultFont', 10, 'bold'), padx=10, pady=5).pack(side='right')

        status_row = ttk.Frame(self.root, padding=(16, 0, 16, 8))
        status_row.pack(fill='x')
        self.raw_badge = self._badge(status_row, 'RAW: WAITING')
        self.raw_badge.pack(side='left')
        self.filtered_badge = self._badge(status_row, 'FILTER: WAITING')
        self.filtered_badge.pack(side='left', padx=(8, 0))
        self.kalman_badge = self._badge(status_row, 'KALMAN: WAITING')
        self.kalman_badge.pack(side='left', padx=(8, 0))

        values = ttk.Frame(self.root, padding=(16, 0, 16, 8))
        values.pack(fill='x')
        values.columnconfigure(0, weight=1)
        values.columnconfigure(1, weight=1)
        raw_frame = ttk.LabelFrame(values, text='/imu/raw — STM32 transport', padding=12)
        raw_frame.grid(row=0, column=0, sticky='nsew', padx=(0, 6))
        filtered_frame = ttk.LabelFrame(values, text='/imu/data — Kalman filtered', padding=12)
        filtered_frame.grid(row=0, column=1, sticky='nsew', padx=(6, 0))

        self.raw_vars = self._field_table(raw_frame, [
            ('Rate / Age', 'rate_age'), ('Status', 'status'), ('MCU stamp', 'stamp'),
            ('Accel raw ×1000', 'accel_integer'), ('Gyro raw ×1000', 'gyro_integer'),
            ('Accel [m/s²]', 'accel'), ('Gyro [rad/s]', 'gyro'),
        ])
        self.filtered_vars = self._field_table(filtered_frame, [
            ('Rate / Age', 'rate_age'), ('Frame / stamp', 'frame_stamp'),
            ('Roll Pitch Yaw [°]', 'rpy'), ('Quaternion x y z w', 'quaternion'),
            ('Accel [m/s²]', 'accel'), ('Gyro [rad/s]', 'gyro'),
            ('Cov roll pitch yaw', 'covariance'),
        ])

        diagnostic_frame = ttk.LabelFrame(
            self.root, text='Kalman diagnostics', padding=10)
        diagnostic_frame.pack(fill='x', padx=16, pady=(0, 8))
        self.diagnostic_vars = self._field_table(diagnostic_frame, [
            ('State', 'state'), ('Calibration', 'calibration'), ('Message', 'message'),
            ('Gyro bias [rad/s]', 'bias'), ('Rx / Published / Rejected', 'counts'),
        ], columns=2)

        controls = ttk.Frame(self.root, padding=(16, 0, 16, 6))
        controls.pack(fill='x')
        self.pause_button = ttk.Button(
            controls, text='Pause log', command=self._toggle_pause)
        self.pause_button.pack(side='left')
        ttk.Button(controls, text='Clear log', command=self._clear_log).pack(
            side='left', padx=(8, 0))
        ttk.Button(controls, text='Close', command=self.close).pack(side='right')
        ttk.Label(
            controls, text='Calibration cần robot đứng yên khoảng 2 giây.',
            style='Muted.TLabel').pack(side='left', padx=(12, 0))

        log_frame = ttk.LabelFrame(self.root, text='RAW + FILTERED LOG', padding=1)
        log_frame.pack(fill='both', expand=True, padx=16, pady=(0, 16))
        self.log = scrolledtext.ScrolledText(
            log_frame, bg=ENTRY, fg=FG, insertbackground=FG,
            font=('DejaVu Sans Mono', 9), wrap='none')
        self.log.pack(fill='both', expand=True)
        self._append_log('Monitor khởi động; đang chờ /imu/raw từ STM32...')

    @staticmethod
    def _badge(parent, text):
        return tk.Label(
            parent, text=text, bg='#3a4049', fg='white',
            font=('TkDefaultFont', 9, 'bold'), padx=10, pady=5)

    @staticmethod
    def _field_table(parent, fields, columns=1):
        variables = {}
        for index, (label, key) in enumerate(fields):
            group = index % columns
            row = index // columns
            label_column = group * 2
            value_column = label_column + 1
            parent.columnconfigure(value_column, weight=1)
            ttk.Label(parent, text=label, style='Muted.TLabel').grid(
                row=row, column=label_column, sticky='w', padx=(0, 8), pady=3)
            variable = tk.StringVar(value='—')
            ttk.Label(parent, textvariable=variable, font=('DejaVu Sans Mono', 9)).grid(
                row=row, column=value_column, sticky='w', padx=(0, 14), pady=3)
            variables[key] = variable
        return variables

    @staticmethod
    def _vector(values, precision=4):
        return '(' + ', '.join(f'{value:+.{precision}f}' for value in values) + ')'

    @staticmethod
    def _age(data, now):
        return now - data['received'] if data is not None else math.inf

    @staticmethod
    def _set_badge(widget, text, color):
        widget.configure(text=text, bg=color)

    def _refresh(self):
        if self.closed:
            return
        if not rclpy.ok():
            self.close()
            return

        snapshot = self.node.snapshot()
        now = time.monotonic()
        raw = snapshot['raw']
        filtered = snapshot['filtered']
        diagnostic = snapshot['diagnostic']

        if raw is None:
            self._set_badge(self.raw_badge, 'RAW: WAITING', '#3a4049')
        else:
            raw_age = self._age(raw, now)
            raw_ok = raw['status'] == ImuRaw.STATUS_OK and raw_age <= STALE_SECONDS
            color = GREEN if raw_ok else (YELLOW if raw_age <= STALE_SECONDS else RED)
            label = 'RAW: OK' if raw_ok else (
                f"RAW: {raw['status_text']}" if raw_age <= STALE_SECONDS else 'RAW: STALE')
            self._set_badge(self.raw_badge, label, color)
            self.raw_vars['rate_age'].set(f"{raw['rate']:.1f} Hz / {raw_age * 1000:.0f} ms")
            self.raw_vars['status'].set(raw['status_text'])
            self.raw_vars['stamp'].set(f"{raw['stamp_ms']} ms")
            self.raw_vars['accel_integer'].set(str(raw['accel_integer']))
            self.raw_vars['gyro_integer'].set(str(raw['gyro_integer']))
            self.raw_vars['accel'].set(self._vector(raw['accel'], 3))
            self.raw_vars['gyro'].set(self._vector(raw['gyro'], 4))

        if filtered is None:
            self._set_badge(self.filtered_badge, 'FILTER: WAITING', '#3a4049')
        else:
            filtered_age = self._age(filtered, now)
            filtered_ok = filtered_age <= STALE_SECONDS
            self._set_badge(
                self.filtered_badge,
                'FILTER: OK' if filtered_ok else 'FILTER: STALE',
                GREEN if filtered_ok else RED)
            self.filtered_vars['rate_age'].set(
                f"{filtered['rate']:.1f} Hz / {filtered_age * 1000:.0f} ms")
            self.filtered_vars['frame_stamp'].set(
                f"{filtered['frame_id']} / {filtered['stamp']:.3f}")
            self.filtered_vars['rpy'].set(self._vector(filtered['rpy'], 2))
            self.filtered_vars['quaternion'].set(self._vector(filtered['quaternion'], 5))
            self.filtered_vars['accel'].set(self._vector(filtered['accel'], 3))
            self.filtered_vars['gyro'].set(self._vector(filtered['gyro'], 4))
            covariance = filtered['orientation_covariance']
            self.filtered_vars['covariance'].set(
                self._vector((covariance[0], covariance[4], covariance[8]), 5))

        if diagnostic is None:
            self._set_badge(self.kalman_badge, 'KALMAN: WAITING', '#3a4049')
        else:
            state = diagnostic['state']
            color = GREEN if state == 'ready' else (
                RED if diagnostic['level'] >= 2 else YELLOW)
            self._set_badge(self.kalman_badge, f'KALMAN: {state.upper()}', color)
            self.diagnostic_vars['state'].set(state)
            self.diagnostic_vars['calibration'].set(
                f"{diagnostic['calibration_count']} / {diagnostic['calibration_target']}")
            self.diagnostic_vars['message'].set(diagnostic['message'])
            self.diagnostic_vars['bias'].set(diagnostic['gyro_bias'])
            self.diagnostic_vars['counts'].set(
                f"{diagnostic['received_samples']} / "
                f"{diagnostic['published_samples']} / {diagnostic['rejected_samples']}")

        self._drain_events()
        if not self.log_paused and raw is not None:
            stamp = raw['stamp_ms']
            if (stamp != self.last_logged_stamp and
                    now - self.last_periodic_log >= LOG_INTERVAL_SECONDS):
                self.last_logged_stamp = stamp
                self.last_periodic_log = now
                filtered_text = 'FILTER waiting'
                if filtered is not None:
                    filtered_text = (
                        f"FILTER rpy°={self._vector(filtered['rpy'], 2)} "
                        f"a={self._vector(filtered['accel'], 3)} "
                        f"g={self._vector(filtered['gyro'], 4)}")
                self._append_log(
                    f"RAW t={stamp:10d} status={raw['status_text']:<12} "
                    f"a={self._vector(raw['accel'], 3)} "
                    f"g={self._vector(raw['gyro'], 4)} | {filtered_text}")

        self.after_id = self.root.after(100, self._refresh)

    def _drain_events(self):
        while True:
            try:
                event = self.node.events.get_nowait()
            except queue.Empty:
                return
            self._append_log(f'EVENT: {event}')

    def _append_log(self, message):
        timestamp = time.strftime('%H:%M:%S')
        milliseconds = int((time.time() % 1.0) * 1000)
        self.log.insert('end', f'[{timestamp}.{milliseconds:03d}] {message}\n')
        self.log_lines += 1
        if self.log_lines > MAX_LOG_LINES:
            remove = self.log_lines - MAX_LOG_LINES
            self.log.delete('1.0', f'{remove + 1}.0')
            self.log_lines = MAX_LOG_LINES
        self.log.see('end')

    def _toggle_pause(self):
        self.log_paused = not self.log_paused
        self.pause_button.configure(text='Resume log' if self.log_paused else 'Pause log')
        self._append_log('Log paused.' if self.log_paused else 'Log resumed.')

    def _clear_log(self):
        self.log.delete('1.0', 'end')
        self.log_lines = 0
        self._append_log('Log cleared.')

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.after_id is not None:
            try:
                self.root.after_cancel(self.after_id)
            except tk.TclError:
                pass
        try:
            self.node.destroy_node()
        except Exception:  # rcl context may already be shutting down after Ctrl+C
            pass
        if rclpy.ok():
            rclpy.shutdown()
        self.spin_thread.join(timeout=2.0)
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = ImuMonitorNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    root = tk.Tk()
    window = ImuMonitorWindow(root, node, spin_thread)
    signal.signal(signal.SIGINT, lambda _signal, _frame: window.close())
    try:
        root.mainloop()
    finally:
        window.close()


if __name__ == '__main__':
    main()
