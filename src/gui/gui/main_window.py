"""Tkinter control panel for the main_bot Gazebo sim and RViz view.

Ported as-is from superDog's gui/main_window.py (same process-control
mechanism, styling, log panel, kill/shutdown buttons). The only functional
change is the FSM row: superDog's movement joysticks/trot button/balance
chart depended on unitree_guide_controller's gait controller (lx/ly/rx/ry
fields on Inputs, active-balance IMU state) which babyDog does not use yet (see
README.md "Giới hạn đã biết") - controller/msg/Inputs only carries
a bare `command` field, so that row is replaced with babyDog's actual
Stand/Sit/Estop buttons (same 3 actions make stand/sit/estop and
keyboard_input.py already expose). Also added a Joystick row (start/stop
`ros2 launch joystick_bridge joystick.launch.py` with configurable button
indices) - a real gamepad and the panel's own Stand/Sit/Estop buttons both
just publish to the same /control_input, so either (or both) can be used
interchangeably while Sim is running.

Replaces the "open one terminal per task" workflow (launch the sim in one
terminal, RViz in another, kill leftover gz/ros processes in a third, ...)
with a single window. The read-only real-IMU pipeline is also available as an
explicitly isolated row; it is mutually exclusive with motion backends.
"""

import os
import queue
import signal
import subprocess
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import scrolledtext, ttk

from ament_index_python.packages import get_package_share_directory
import psutil

# micro_ros_agent (goi cho nut "Start real hardware") KHONG nam trong workspace babyDog -
# xem real_ros2_control.launch.py's module docstring - nam rieng o ~/mros/mros_ws. Nguoi
# dung phai tu source workspace do TRUOC khi mo GUI, neu khong "ros2 launch" se bao "package
# 'micro_ros_agent' not found" ngay khi bam nut do (sim/rviz/joystick khong can, chi 'real'
# can). Tu merge san o day (1 lan luc GUI khoi dong) thay vi bat nho source moi lan - doc
# setup.bash bang 1 subshell bash roi gop moi bien no dat (AMENT_PREFIX_PATH, PATH,
# PYTHONPATH, ...) vao os.environ cua chinh tien trinh GUI, subprocess.Popen o start_process()
# ke thua os.environ mac dinh nen tu dong thay duoc. An toan bo qua neu duong dan nay khong
# ton tai tren may khac (chi nut 'real' bi anh huong, khong lam sim/rviz/joystick hong).
def _merge_mros_workspace_env():
    setup_bash = os.path.expanduser('~/mros/mros_ws/install/setup.bash')
    if not os.path.isfile(setup_bash):
        return
    try:
        result = subprocess.run(
            ['bash', '-c', f'source "{setup_bash}" && env'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired):
        return
    if result.returncode != 0:
        return
    for line in result.stdout.splitlines():
        key, sep, value = line.partition('=')
        if sep:
            os.environ[key] = value


_merge_mros_workspace_env()

# Ep domain 0 truoc khi rclpy duoc dung (phai dat truoc rclpy.init() ben duoi, RMW doc
# bien nay luc init; dat SAU _merge_mros_workspace_env() o tren de gia tri nay luon la
# tieng noi cuoi cung, khong bi setup.bash nao ghi de) - khop voi real_ros2_control.launch.py's
# SetEnvironmentVariable: micro_ros_agent's DDS participant bridge lay domain THEO CHINH
# PHIEN CLIENT (tuc do firmware MCU quyet dinh luc build thu vien micro-ROS, mac dinh = 0,
# xem real_ros2_control.launch.py's module docstring), khong doc ROS_DOMAIN_ID cua may tinh -
# ep GUI (va moi tien trinh no spawn qua subprocess.Popen: sim/rviz/joystick/real, ke thua
# os.environ mac dinh) cung domain 0 de LUON khop voi backend that, khong ai phai nho
# "export ROS_DOMAIN_ID=0" truoc khi mo GUI nua. Vo hai voi sim (domain nao cung duoc,
# chi can nhat quan giua cac tien trinh).
os.environ['ROS_DOMAIN_ID'] = '0'

import rclpy
from controller.msg import Inputs

STOP_GRACE_SECONDS = 5

BG = '#1e1e1e'
BG_PANEL = '#2d2d2d'
BG_ENTRY = '#3c3c3c'
BORDER = '#3c3c3c'
FG = '#d4d4d4'
FG_MUTED = '#9d9d9d'
ACCENT_START = '#2ea043'
ACCENT_START_HOVER = '#3fb950'
ACCENT_STOP = '#da3633'
ACCENT_STOP_HOVER = '#f85149'
ACCENT_KILL = '#9e6a03'
ACCENT_KILL_HOVER = '#bb8009'
LOG_BG = '#0c0c0c'
LOG_FG = '#d4d4d4'

_STATUS_STYLES = {
    'idle': 'BadgeIdle.TLabel',
    'running': 'BadgeRunning.TLabel',
    'stopping': 'BadgeStopping.TLabel',
}


class SimControlGui:

    def __init__(self, root):
        self.root = root
        self.root.title('babyDog Control + IMU')
        self.root.geometry('980x780')
        self.root.minsize(760, 520)

        self.procs = {
            'sim': None, 'rviz': None, 'joystick': None, 'real': None,
            'rviz_real': None, 'imu_view': None,
        }
        self.proc_widgets = {}
        self.fsm_widgets = []
        self.log_queue = queue.Queue()

        # A persistent rclpy node/publisher, created once and reused for every
        # FSM command. Earlier version (superDog) spawned a fresh `ros2 topic
        # pub` subprocess per click, which pays rclpy-interpreter-startup +
        # DDS discovery cost (can be 1s+) every single time - visibly laggy.
        # publish() on an already-discovered publisher is near-instant.
        rclpy.init()
        self.ros_node = rclpy.create_node('gui_control_panel')
        self.control_input_pub = self.ros_node.create_publisher(Inputs, '/control_input', 10)

        self._build_widgets()
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        # ID of the recurring after() timer, so _on_close can cancel it
        # explicitly - otherwise it can fire after root.destroy() and Tk
        # raises "invalid command name ..." trying to run the dead callback.
        self._poll_log_after_id = self.root.after(100, self._poll_log_queue)

    def _setup_style(self):
        self.root.configure(bg=BG)
        tkfont.nametofont('TkDefaultFont').configure(family='TkDefaultFont', size=10)

        style = ttk.Style(self.root)
        try:
            style.theme_use('clam')
        except tk.TclError:
            pass

        style.configure('TFrame', background=BG)
        style.configure('TLabelframe', background=BG, bordercolor=BORDER, relief='flat')
        style.configure('TLabelframe.Label', background=BG, foreground=FG_MUTED,
                         font=('TkDefaultFont', 9, 'bold'))
        style.configure('TLabel', background=BG, foreground=FG)
        style.configure('Header.TLabel', background=BG, foreground=FG,
                         font=('TkDefaultFont', 15, 'bold'))
        style.configure('RowLabel.TLabel', background=BG, foreground=FG_MUTED,
                         font=('TkDefaultFont', 10, 'bold'))
        style.configure('TEntry', fieldbackground=BG_ENTRY, foreground=FG,
                         insertcolor=FG, bordercolor=BORDER,
                         lightcolor=BG_ENTRY, darkcolor=BG_ENTRY, padding=5)

        style.configure('TButton', background=BG_PANEL, foreground=FG,
                         font=('TkDefaultFont', 10), padding=(14, 8), borderwidth=0,
                         focusthickness=0)
        style.map('TButton',
                  background=[('active', '#3c3c3c'), ('disabled', '#2a2a2a')],
                  foreground=[('disabled', '#6a6a6a')])

        for name, base, hover in (
            ('Start.TButton', ACCENT_START, ACCENT_START_HOVER),
            ('Stop.TButton', ACCENT_STOP, ACCENT_STOP_HOVER),
            ('Kill.TButton', ACCENT_KILL, ACCENT_KILL_HOVER),
        ):
            style.configure(name, background=base, foreground='white',
                             font=('TkDefaultFont', 10, 'bold'), padding=(14, 8),
                             borderwidth=0, focusthickness=0)
            style.map(name,
                      background=[('active', hover), ('disabled', '#3c3c3c')],
                      foreground=[('disabled', '#6a6a6a')])

        for name, bg in (
            ('BadgeIdle.TLabel', '#3c3c3c'),
            ('BadgeRunning.TLabel', ACCENT_START),
            ('BadgeStopping.TLabel', ACCENT_KILL),
        ):
            style.configure(name, background=bg, foreground='white',
                             font=('TkDefaultFont', 10, 'bold'), padding=(10, 5))

    def _build_widgets(self):
        self._setup_style()

        header = ttk.Frame(self.root, padding=(16, 14, 16, 4))
        header.pack(fill='x')
        ttk.Label(header, text='babyDog Control + IMU', style='Header.TLabel').pack(side='left')

        params = ttk.LabelFrame(self.root, text='SPAWN PARAMETERS', padding=(14, 10))
        params.pack(fill='x', padx=16, pady=(10, 6))

        self.world_var = tk.StringVar(value='')
        self.robot_name_var = tk.StringVar(value='babydog')
        self.x_var = tk.StringVar(value='0.0')
        self.y_var = tk.StringVar(value='0.0')
        self.z_var = tk.StringVar(value='0.5')

        fields = [
            ('World (blank = default)', self.world_var, 16),
            ('Robot name', self.robot_name_var, 10),
            ('X', self.x_var, 6),
            ('Y', self.y_var, 6),
            ('Z', self.z_var, 6),
        ]
        for col, (label, var, width) in enumerate(fields):
            ttk.Label(params, text=label).grid(
                row=0, column=2 * col, padx=(0 if col == 0 else 4, 6), pady=4, sticky='w')
            ttk.Entry(params, textvariable=var, width=width).grid(
                row=0, column=2 * col + 1, padx=(0, 14), pady=4, sticky='w')

        sim_row = ttk.Frame(self.root, padding=(16, 4))
        sim_row.pack(fill='x')
        ttk.Label(sim_row, text='Sim', style='RowLabel.TLabel', width=6).pack(side='left')
        self._build_process_controls(
            sim_row, key='sim', start_text='▶  Start sim', stop_text='■  Stop sim')
        ttk.Label(
            sim_row, foreground=FG_MUTED,
            text='  (Gazebo, dùng nút Đứng lên/Ngồi xuống bên dưới)'
        ).pack(side='left')

        rviz_row = ttk.Frame(self.root, padding=(16, 4))
        rviz_row.pack(fill='x')
        ttk.Label(rviz_row, text='RViz', style='RowLabel.TLabel', width=6).pack(side='left')
        self._build_process_controls(
            rviz_row, key='rviz', start_text='▶  Open RViz', stop_text='■  Close RViz')
        ttk.Label(
            rviz_row, foreground=FG_MUTED,
            text=(
                '  (chỉ Grid+TF, không có robot lúc mở - chạy song song với Sim ở trên, '
                'bấm TF sẽ hiện đúng trạng thái sim đang chạy)'
            )
        ).pack(side='left')

        self._build_real_row()
        self._build_imu_row()
        self._build_joystick_row()
        self._build_fsm_panel()

        util_row = ttk.Frame(self.root, padding=(16, 4))
        util_row.pack(fill='x')
        ttk.Label(util_row, text='', width=6).pack(side='left')
        self.kill_button = ttk.Button(
            util_row, text='Kill gz/ros traces', style='Kill.TButton', command=self.kill_traces)
        self.kill_button.pack(side='left')

        self.shutdown_button = ttk.Button(
            util_row, text='⏻  Tắt hết & Thoát', style='Stop.TButton',
            command=self.shutdown_everything)
        self.shutdown_button.pack(side='left', padx=(10, 0))

        log_frame = ttk.LabelFrame(self.root, text='LOG', padding=(1, 1))
        log_frame.pack(fill='both', expand=True, padx=16, pady=(6, 16))

        # Kept in 'normal' state (not 'disabled') so mouse selection and Ctrl+C
        # copy work reliably across Tk versions; _block_edit below blocks
        # actual typing while still letting programmatic .insert() calls through.
        self.log_text = scrolledtext.ScrolledText(
            log_frame, wrap='word', font=('DejaVu Sans Mono', 10),
            bg=LOG_BG, fg=LOG_FG, insertbackground=LOG_FG,
            selectbackground='#264f78', selectforeground='white',
            relief='flat', borderwidth=0, padx=10, pady=8)
        self.log_text.pack(fill='both', expand=True)
        self.log_text.bind('<Key>', self._block_edit)
        self.log_text.vbar.configure(
            bg=BG_PANEL, troughcolor=BG, activebackground=BORDER,
            bd=0, highlightthickness=0)

    def _build_process_controls(self, parent, key, start_text, stop_text):
        start_button = ttk.Button(
            parent, text=start_text, style='Start.TButton',
            command=lambda: self.start_process(key))
        start_button.pack(side='left')

        stop_button = ttk.Button(
            parent, text=stop_text, style='Stop.TButton',
            command=lambda: self.stop_process(key), state='disabled')
        stop_button.pack(side='left', padx=(10, 0))

        status_label = ttk.Label(parent, style='BadgeIdle.TLabel')
        status_label.pack(side='right')

        self.proc_widgets[key] = {'start': start_button, 'stop': stop_button, 'status': status_label}
        self._set_badge(status_label, 'Idle', 'idle')

    def _build_real_row(self):
        # real_ros2_control.launch.py (main_bot_hardware/RealSystem, KHONG qua Gazebo) -
        # doi lap voi 'sim' o tren, cung mo khoa FSM panel ben duoi (xem
        # _set_fsm_controls_enabled) vi ca 2 deu la 1 backend ros2_control hop le, chi
        # khac hardware plugin. Domain ID tu dong khop 0 voi backend (xem os.environ o
        # dau file) - khong can nhap gi them o day.
        self.serial_dev_var = tk.StringVar(value='/dev/ttyUSB0')
        self.serial_baud_var = tk.StringVar(value='921600')

        real_row = ttk.Frame(self.root, padding=(16, 4))
        real_row.pack(fill='x')
        ttk.Label(real_row, text='Real', style='RowLabel.TLabel', width=6).pack(side='left')
        self._build_process_controls(
            real_row, key='real', start_text='▶  Start real hardware', stop_text='■  Stop real hardware')
        ttk.Label(real_row, text='Serial', foreground=FG_MUTED).pack(side='left', padx=(14, 4))
        ttk.Entry(real_row, textvariable=self.serial_dev_var, width=14).pack(side='left')
        ttk.Label(real_row, text='Baud', foreground=FG_MUTED).pack(side='left', padx=(10, 4))
        ttk.Entry(real_row, textvariable=self.serial_baud_var, width=8).pack(side='left')
        ttk.Label(
            real_row, foreground=FG_MUTED,
            text='  (micro-ROS/UART + motor)'
        ).pack(side='left', padx=(8, 0))

        rviz_real_row = ttk.Frame(self.root, padding=(16, 4))
        rviz_real_row.pack(fill='x')
        ttk.Label(rviz_real_row, text='RViz', style='RowLabel.TLabel', width=6).pack(side='left')
        self._build_process_controls(
            rviz_real_row, key='rviz_real', start_text='▶  Open RViz', stop_text='■  Close RViz')
        ttk.Label(
            rviz_real_row, foreground=FG_MUTED,
            text=(
                '  (chỉ Grid+TF, không có robot lúc mở - chạy song song với Real ở trên, '
                'bấm TF sẽ hiện đúng trạng thái robot thật hiện tại)'
            )
        ).pack(side='left')

    def _build_imu_row(self):
        imu_view_row = ttk.Frame(self.root, padding=(16, 4))
        imu_view_row.pack(fill='x')
        ttk.Label(imu_view_row, text='IMU', style='RowLabel.TLabel', width=6).pack(side='left')
        self._build_process_controls(
            imu_view_row, key='imu_view', start_text='▶  View IMU log',
            stop_text='■  Close IMU log')
        ttk.Label(
            imu_view_row, foreground=FG_MUTED,
            text=(
                '  (chỉ mở cửa sổ xem /imu/raw + /imu/data, KHÔNG mở micro_ros_agent riêng - '
                'dùng khi Sim/Real đã đang chạy sẵn pipeline IMU)'
            )
        ).pack(side='left', padx=(10, 0))

    def _build_joystick_row(self):
        # Index cua nut tren gamepad - khac nhau tuy tay cam (mac dinh 0/1/6 khop kieu
        # Xbox theo joystick_input.py's own defaults). Do khong biet truoc index dung
        # cua tay cam nguoi dung, tu do la o nhap thay vi hardcode - xem
        # src/joystick_bridge/README.md muc "Doi nut joystick" de biet cach do index
        # that (ros2 topic echo /joy, bam tung nut xem so nao doi tu 0 sang 1).
        self.home_button_var = tk.StringVar(value='0')
        self.toggle_button_var = tk.StringVar(value='1')
        self.estop_button_var = tk.StringVar(value='6')

        joystick_row = ttk.Frame(self.root, padding=(16, 4))
        joystick_row.pack(fill='x')
        ttk.Label(joystick_row, text='Joystick', style='RowLabel.TLabel', width=6).pack(side='left')
        self._build_process_controls(
            joystick_row, key='joystick', start_text='▶  Start joystick', stop_text='■  Stop joystick')

        joystick_params = ttk.Frame(self.root, padding=(16, 0, 16, 4))
        joystick_params.pack(fill='x')
        ttk.Label(joystick_params, text='', width=6).pack(side='left')
        for label, var in (
            ('Home btn', self.home_button_var),
            ('Toggle btn (Đứng⇄Ngồi)', self.toggle_button_var),
            ('Estop btn', self.estop_button_var),
        ):
            ttk.Label(joystick_params, text=label, foreground=FG_MUTED).pack(side='left', padx=(0, 4))
            ttk.Entry(joystick_params, textvariable=var, width=4).pack(side='left', padx=(0, 14))
        ttk.Label(
            joystick_params, foreground=FG_MUTED,
            text='(index nút gamepad - xem src/joystick_bridge/README.md nếu chưa biết)'
        ).pack(side='left')

    def _build_fsm_panel(self):
        # babyDog's controller only knows 3 commands (see
        # controller/msg/Inputs.msg and GUIDE.md's /control_input
        # table): Stand/Sit/Estop. No trot/movement - no gait controller at
        # this stage (see README.md "Giới hạn đã biết").
        fsm_frame = ttk.LabelFrame(
            self.root, text='FSM (needs Sim or Real hardware running)', padding=(14, 10))
        fsm_frame.pack(fill='x', padx=16, pady=(6, 6))

        fsm_row = ttk.Frame(fsm_frame)
        fsm_row.pack(fill='x')
        stand_button = ttk.Button(
            fsm_row, text='Đứng lên', command=lambda: self._publish_control_input(1))
        stand_button.pack(side='left')
        sit_button = ttk.Button(
            fsm_row, text='Ngồi xuống', command=lambda: self._publish_control_input(2))
        sit_button.pack(side='left', padx=(8, 0))
        estop_button = ttk.Button(
            fsm_row, text='Estop', style='Stop.TButton',
            command=lambda: self._publish_control_input(9))
        estop_button.pack(side='left', padx=(8, 0))

        self.fsm_widgets = [stand_button, sit_button, estop_button]
        self._set_fsm_controls_enabled(False)

    def _set_fsm_controls_enabled(self, enabled):
        state = 'normal' if enabled else 'disabled'
        for widget in self.fsm_widgets:
            widget.configure(state=state)

    def _publish_control_input(self, command):
        msg = Inputs()
        msg.command = command
        self.control_input_pub.publish(msg)

    def _block_edit(self, event):
        ctrl_held = bool(event.state & 0x4)
        if ctrl_held and event.keysym.lower() in ('c', 'a'):
            return None
        if event.keysym in ('Left', 'Right', 'Up', 'Down', 'Prior', 'Next', 'Home', 'End'):
            return None
        return 'break'

    def _append_log(self, text):
        self.log_text.insert('end', text)
        self.log_text.see('end')

    def _set_badge(self, label, text, kind):
        label.configure(text='●  ' + text, style=_STATUS_STYLES[kind])

    def _command_for(self, key):
        if key == 'sim':
            cmd = [
                'ros2', 'launch', 'main_bot', 'sim.launch.py',
                'robot_name:=' + self.robot_name_var.get(),
                'x:=' + self.x_var.get(),
                'y:=' + self.y_var.get(),
                'z:=' + self.z_var.get(),
                # sim.launch.py opens its own RViz by default; the GUI keeps a
                # separate dedicated RViz row/button, so skip the bundled one
                # here to avoid two RViz windows opening at once.
                'rviz:=false',
            ]
            world = self.world_var.get().strip()
            if world:
                cmd.append('world:=' + world)
            return cmd
        if key == 'rviz':
            # rz_sim.launch.py - CHI mo RViz (Grid+TF, khong co robot ben trong luc mo,
            # khong tu chay robot_state_publisher/joint_state_publisher gia nao ca) -
            # ngoi cho va hien TF THAT dang duoc sim (dang chay o 'sim' row, rviz:=false)
            # publish. Thay the ban rz.launch.py cu (co RobotModel + joint_state_
            # publisher_gui gia, tung xung dot /joint_states voi sim khi chay cung luc).
            return ['ros2', 'launch', 'main_bot', 'rz_sim.launch.py', 'use_sim_time:=true']
        if key == 'real':
            return [
                'ros2', 'launch', 'main_bot', 'real_ros2_control.launch.py',
                'serial_dev:=' + self.serial_dev_var.get(),
                'serial_baud:=' + self.serial_baud_var.get(),
                # real_ros2_control.launch.py mo RViz kem theo mac dinh - GUI co rieng 1
                # hang 'RViz (real)' (key 'rviz_real') giong het pattern sim/rviz o tren,
                # nen tat bundled rviz o day de tranh mo trung 2 cua so.
                'rviz:=false',
            ]
        if key == 'rviz_real':
            # rz_real.launch.py - CHI mo RViz (Grid+TF, khong co robot ben trong luc mo) -
            # ngoi cho va hien TF THAT dang duoc robot that (dang chay o 'real' row,
            # rviz:=false) publish.
            return ['ros2', 'launch', 'main_bot', 'rz_real.launch.py']
        if key == 'joystick':
            return [
                'ros2', 'launch', 'joystick_bridge', 'joystick.launch.py',
                'home_button:=' + self.home_button_var.get(),
                'toggle_button:=' + self.toggle_button_var.get(),
                'estop_button:=' + self.estop_button_var.get(),
            ]
        if key == 'imu_view':
            # Chi mo cua so Tk subscribe /imu/raw + /imu/data - KHONG tu mo
            # micro_ros_agent rieng, nen dung duoc song song voi Sim/Real dang
            # chay san pipeline IMU cua chinh no (xem real_ros2_control.launch.py
            # / sim.launch.py da co imu_kalman_filter). Khong tham gia mutual
            # exclusion vi khong tu mo agent nao ca.
            return ['ros2', 'run', 'gui', 'imu_monitor']
        raise ValueError(key)

    def start_process(self, key):
        if self.procs[key] is not None:
            return

        cmd = self._command_for(key)
        self._append_log(f'$ [{key}] ' + ' '.join(cmd) + '\n')

        try:
            proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, start_new_session=True)
        except OSError as exc:
            self._append_log(f'Failed to start {key}: {exc}\n')
            return

        self.procs[key] = proc
        widgets = self.proc_widgets[key]
        widgets['start'].configure(state='disabled')
        widgets['stop'].configure(state='normal')
        self._set_badge(widgets['status'], f'Running (pid {proc.pid})', 'running')
        if key in ('sim', 'real'):
            self._set_fsm_controls_enabled(True)

        threading.Thread(target=self._read_proc_output, args=(key, proc), daemon=True).start()

    def _read_proc_output(self, key, proc):
        for line in proc.stdout:
            self.log_queue.put(('log', key, line))
        returncode = proc.wait()
        self.log_queue.put(('exited', key, returncode))

    def _poll_log_queue(self):
        try:
            while True:
                kind, key, payload = self.log_queue.get_nowait()
                if kind == 'log':
                    self._append_log(payload)
                elif kind == 'exited':
                    self._append_log(f'--- [{key}] exited (code {payload}) ---\n')
                    self.procs[key] = None
                    widgets = self.proc_widgets[key]
                    widgets['start'].configure(state='normal')
                    widgets['stop'].configure(state='disabled')
                    self._set_badge(widgets['status'], 'Idle', 'idle')
                    if key in ('sim', 'real'):
                        # Chi khoa lai FSM neu CA HAI backend deu da tat - vd sim dang
                        # chay + real cung dang chay (song song, khong bi cam), tat rieng
                        # sim thi real van con giu FSM mo khoa duoc.
                        if self.procs['sim'] is None and self.procs['real'] is None:
                            self._set_fsm_controls_enabled(False)
                elif kind == 'kill_done':
                    self.kill_button.configure(state='normal')
        except queue.Empty:
            pass
        self._poll_log_after_id = self.root.after(100, self._poll_log_queue)

    def stop_process(self, key):
        proc = self.procs[key]
        if proc is None:
            return
        self._set_badge(self.proc_widgets[key]['status'], 'Stopping...', 'stopping')
        if key in ('sim', 'real'):
            other_key = 'real' if key == 'sim' else 'sim'
            if self.procs[other_key] is None:
                self._set_fsm_controls_enabled(False)
        pid = proc.pid
        self._send_signal_to_group(pid, signal.SIGINT)
        self.root.after(STOP_GRACE_SECONDS * 1000, lambda: self._escalate_stop(key, pid))

    def _escalate_stop(self, key, pid):
        proc = self.procs[key]
        if proc is None or proc.pid != pid:
            return
        self._append_log(f'--- [{key}] still alive after SIGINT, sending SIGTERM ---\n')
        self._send_signal_to_group(pid, signal.SIGTERM)

    def _send_signal_to_group(self, pid, sig):
        # os.killpg CHI dung neu moi tien trinh con van con trong CUNG process group voi
        # pid goc - nhung 'ros2 launch' tu dat cac tien trinh no sinh ra (ros2_control_node,
        # micro_ros_agent, robot_state_publisher, spawner) vao session/process-group RIENG
        # (chinh no lam vay de Ctrl+C ngoai terminal khong giet con truoc khi launch kip
        # dong bo tat het) - nen chi killpg(pid goc) la KHONG DU, de sot ca cay tien trinh
        # con song ngam (da gap nhieu lan: bam "Stop real hardware" tuong da xong nhung
        # ros2_control_node/micro_ros_agent van con song, chong len lan Start tiep theo -
        # co luc toi 5 bo song song cung luc, gay loi kho hieu/khong tai hien duoc).
        # Dung psutil di het ca cay con (de quy, khong phu thuoc process group) roi gui
        # tin hieu rieng cho TUNG tien trinh, cong voi killpg nhu lop du phong.
        try:
            children = psutil.Process(pid).children(recursive=True)
        except psutil.NoSuchProcess:
            children = []
        for child in children:
            try:
                child.send_signal(sig)
            except psutil.NoSuchProcess:
                pass
        try:
            os.killpg(os.getpgid(pid), sig)
        except ProcessLookupError:
            pass

    def kill_traces(self):
        self.kill_button.configure(state='disabled')
        threading.Thread(target=self._run_kill_traces, daemon=True).start()

    def _run_kill_traces(self):
        self.log_queue.put(('log', None, '$ ros2 run main_bot kill_gz.sh\n'))
        try:
            result = subprocess.run(
                ['ros2', 'run', 'main_bot', 'kill_gz.sh'],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            self.log_queue.put(('log', None, result.stdout))
        except OSError as exc:
            self.log_queue.put(('log', None, f'Failed to run kill_gz.sh: {exc}\n'))
        self.log_queue.put(('kill_done', None, None))

    def shutdown_everything(self):
        """One-click full shutdown: stop every running launch, sweep any stray
        gz/ros processes with kill_gz.sh, then close the app - so quitting
        never requires switching to the terminal the GUI was started from."""
        self.shutdown_button.configure(state='disabled', text='Đang tắt...')
        self.kill_button.configure(state='disabled')
        for proc in self.procs.values():
            if proc is not None:
                self._send_signal_to_group(proc.pid, signal.SIGINT)
        # Poll instead of a blind STOP_GRACE_SECONDS wait - most of the time
        # every launch has already exited well before the grace period, and
        # there is no reason to make the user stare at "Dang tat..." for the
        # full 5s just because that's the worst-case timeout. Still falls
        # back to the same deadline for a launch that's slow/stuck to exit.
        self._shutdown_deadline = time.monotonic() + STOP_GRACE_SECONDS
        self._wait_for_shutdown()

    def _wait_for_shutdown(self):
        if all(proc is None for proc in self.procs.values()):
            self._finish_shutdown()
            return
        if time.monotonic() >= self._shutdown_deadline:
            self._finish_shutdown()
            return
        self.root.after(200, self._wait_for_shutdown)

    def _finish_shutdown(self):
        try:
            subprocess.run(
                ['ros2', 'run', 'main_bot', 'kill_gz.sh'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
        except (OSError, subprocess.TimeoutExpired):
            pass
        self._on_close()

    def _on_close(self):
        for proc in self.procs.values():
            if proc is not None:
                self._send_signal_to_group(proc.pid, signal.SIGINT)
        # Cancel the recurring timer explicitly - otherwise it can fire after
        # root.destroy() below and Tk raises "invalid command name ..." trying
        # to run a callback tied to a now-dead widget.
        self.root.after_cancel(self._poll_log_after_id)
        self.ros_node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()


def main():
    root = tk.Tk()
    gui = SimControlGui(root)
    try:
        root.mainloop()
    finally:
        # Luoi an toan cuoi cung - neu GUI thoat BAT THUONG (Ctrl+C trong terminal, dong
        # terminal, kill tu ngoai) thay vi qua nut X/"Tat het & Thoat" (ca 2 deu goi
        # _on_close() o tren, da tu dung tien trinh con roi), _on_close() KHONG chay,
        # de sot tien trinh con (sim/rviz/joystick/real) song ngam - lan sau mo GUI lai
        # se dung do 2 bo controller_manager/micro_ros_agent cung ton tai, gay loi kho
        # hieu (da gap nhieu lan qua dem nay). Goi lai vo hai neu _on_close() da chay roi
        # (self.procs luc do da toan None, vong lap duoi khong lam gi ca).
        for proc in gui.procs.values():
            if proc is not None:
                gui._send_signal_to_group(proc.pid, signal.SIGINT)


if __name__ == '__main__':
    main()
