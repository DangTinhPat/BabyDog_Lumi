# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

`babyDog`: a real 4-legged robot (RDK X5/laptop + STM32H7 MCU), current scope is **Stand/Sit only**
(no gait/walking or active balance yet), controlled via joystick/keyboard. It has an MPU6050 data
path with EC-side Kalman filtering, plus both a Gazebo simulation
(ROS 2 Jazzy + Gazebo Harmonic) and the real-hardware stack. It's a deliberately trimmed-down fork
of `../superDog` (full walking-gait project) — reuses the physical description + `leg_pd_controller`
but drops the full gait/balance estimator since this phase doesn't need it.

## Roadmap

Current repo scope is **Phase 1 of 3** — a longer-term plan, not the final destination. Don't treat
the missing pieces below as bugs or gaps to fill unprompted; they're intentionally out of scope for
now.

- **Phase 1 (current)** — basic system so the robot can stand up / sit down, and move with a basic
  gait (simple, not yet optimized). MPU6050 sensing/filtering has landed early, but does not alter
  motor commands.
- **Phase 2 (planned)** — use the filtered IMU for active/dynamic balance while moving.
- **Phase 3 (planned)** — add reinforcement learning.

The current FSM (`Passive`/`HoldPose` only, no `Estimator`/`BalanceCtrl` QP/gait generator — compare
with `../superDog`'s full state machine) is deliberately scoped to Phase 1.

## ⚠️ Mandatory operating rules

This is a real robot with real motors — these aren't suggestions, they prevent physical
damage/injury and hours of "why is this still broken" debugging. This section is the single source
of truth for these rules; `NOTE.md` §4 points back here instead of duplicating them.

- **Only ever run ONE `real_ros2_control.launch.py` process at a time.** Two processes both sending
  PD commands over the same wire to real motors is a genuine physical hazard, not just a software
  bug. Before launching, check: `lsof /dev/ttyUSB0` and
  `ps aux | grep -E "micro_ros_agent|ros2_control_node|robot_state_publisher"`.
- **`pkill -f ...` can silently fail to kill a matching process.** Always verify with `ps aux`/`lsof`
  after pkill; `kill -9 <PID>` directly on anything still alive.
- **Always `export ROS_DOMAIN_ID=0`** before any ad-hoc `ros2 ...` CLI/RViz aimed at the real robot
  from a plain terminal. The launch file forces domain 0 for its own children, but this machine's
  `~/.bashrc` defaults a fresh terminal to domain 7 — the failure mode is silent (you just see
  nothing), not an error.
- **Real hardware work needs 3 workspaces sourced**: `/opt/ros/jazzy`, `~/mros/mros_ws`,
  `babyDog/install` — skip `mros_ws` and `micro_ros_agent` won't be found.
- **Flashing firmware must use `--reset`** (`make flash`/`make firmware-flash` already do this
  correctly). A hand-rolled `st-flash` command without `--reset` leaves the board silently running
  the OLD firmware — a classic "I fixed the code but it's still broken" trap.
- **Before changing any calibration/limit value** (kp/kd, the limit table, stand/sit pose), read
  "Critical invariants" below first — this exact class of bug has recurred multiple times.
- **Do not enable IMU-to-motor correction before verifying all three physical axes/signs.** A wrong
  roll/pitch sign turns negative feedback into positive feedback. The current IMU path is sensing-only.

## Commands

All commands run from the repo root (`babyDog/`). Run `make build` once before anything else.
The `Makefile` wraps `ros2 launch`/`ros2 run`/`ros2 topic pub` so you don't need to source
workspaces by hand — every ROS2 target auto-sources `/opt/ros/jazzy/setup.bash` + `install/setup.bash`.

```bash
make build              # colcon build --symlink-install (whole ROS2 workspace)
make sim                # Gazebo + RViz + controllers (simulation)
make sim-no-rviz        # same, no RViz
make gui                # Tk control panel: sim/real/FSM plus isolated IMU-only monitor row
make keyboard           # keyboard teleop: '1'=stand '2'=sit '0'=estop
make joystick           # real joystick/gamepad -> /control_input
make stand / sit / estop  # one-shot CLI command, no joystick needed
make controllers        # ros2 control list_controllers (verify 3 controllers "active")
make kill               # kill stray Gazebo processes

make real                # real hardware + RViz (requires firmware flashed, micro_ros_agent workspace sourced)
make real-no-rviz        # same, no RViz (e.g. over SSH)
make imu-test            # real IMU only: agent + Kalman + raw/filtered monitor, no ros2_control
make imu-monitor         # monitor an already-running IMU pipeline (read-only)

make microros-lib         # regenerate MCU type-support after firmware message changes
make firmware            # build firmware/stm32h7 (needs arm-none-eabi-gcc in PATH)
make firmware-test       # host-side BabyAlpha2 codec unit tests
make firmware-flash      # build + flash via ST-Link (st-flash --reset write)
make firmware-clean

make clean                # rm -rf build install log
```

To build/test a single ROS2 package: `colcon build --packages-select <pkg>` (source
`/opt/ros/jazzy/setup.bash` first). `imu_kalman_filter` has synthetic gtests; the existing motion
stack still relies on simulation observation and proportionate real-hardware testing.

Firmware alone, without the Make wrapper:
```bash
cd firmware/stm32h7 && make && make flash    # make flash = st-flash --reset write (see safety rules above)
```

See [GUIDE.md](GUIDE.md) for the full walkthrough (including joystick button remapping) and
[firmware/stm32h7/README.md](firmware/stm32h7/README.md) before touching real hardware — it documents
exactly what has and hasn't been verified on the physical board.

## Architecture

```
Joystick/keyboard (joystick_bridge) --/control_input (controller/msg/Inputs: command 0/1/2/9)-->
                                                    |
                    ┌───────────────────────────────┴───────────────────────────────┐
                    ▼ SIMULATION (dev PC)                    ▼ REAL (RDK X5/laptop)
        controller (FSM Passive/Stand/Sit,           controller (same FSM, claims real
        ros2_control)                                 hardware directly)
              │                                             │
              ▼                                             ▼
        leg_pd_controller (tau = kp·Δq+kd·Δq̇)      main_bot_hardware / RealSystem
              │                                       (publishes /joint_cmd, subscribes
              ▼                                       /joint_fb + /joint_diag via micro-ROS)
        gz_ros2_control -> Gazebo (12 joints)               │ micro_ros_agent (UART1, ~921600 baud)
                                                              ▼
                                                    STM32H7 (firmware/stm32h7/, board
                                                    FK743M5-XIH6, no HAL — direct register
                                                    access) -> CAN-FD -> 12 BabyAlpha2 joint
                                                    driver boards (each runs its own local
                                                    PD loop + PWM + 6-wire encoder reading)
```

Both branches (sim/real) share the same `controller` package FSM. On real hardware, `controller`
runs FK/IK and sends joint angles down to the STM32; the firmware itself has no FSM or built-in
Stand/Sit pose — it's a dumb relay + watchdog.

The sensing path is separate from actuation: real `MPU6050 -> STM32 /imu/raw` and simulated
`Gazebo -> /imu/sim_raw` both enter `imu_kalman_filter` on the EC and publish standard `/imu/data`.
The Stand/Sit controller does not consume this topic for motor correction yet.

## Data flow: input to actuation

The diagram above shows boxes; this is the step-by-step sequence a button press actually travels
through. Both branches share steps 1-4 (same FSM/IK code) and diverge only in how the resulting
joint targets get to the motors.

**SIM branch** (button press → Gazebo joint moves):
1. `joystick_bridge` (`joystick_input.py`/`keyboard_input.py`) reads `/joy` or keyboard → publishes `/control_input` (`controller/msg/Inputs{command}`).
2. `StandSitController::update()` (runs every ros2_control cycle) reads `/control_input` → FSM picks a state (`StatePassive`/`StateHoldPose`, the latter shared by both Stand and Sit).
3. `StateHoldPose::run()`: tanh-interpolates the target foot position in Cartesian space → `solveFootIK()` (`RobotLeg::calcQPosition`, KDL LMA solver) → per-cycle joint angle targets.
4. Writes joint angles, joint velocities derived from consecutive valid IK solutions (`qd_des=(q_des[k]-q_des[k-1])/period`), and kp/kd into ros2_control command interfaces. During Stand it also ramps static load feedforward `effort = -scale * J(q_measured)^T * F_support`; during Sit it ramps that term back to zero.
5. `leg_pd_controller::update()` reads those interfaces + current state → computes `tau = effort_ff + kp*(q_des-q) + kd*(qd_des-qd)` → writes the `effort` interface `gz_ros2_control` exports to Gazebo.
6. Gazebo applies this torque to the 12 physical joints in sim.
7. `joint_state_broadcaster` reads joint state from Gazebo → publishes `/joint_states` → drives RViz (via `robot_state_publisher`/TF) and feeds back as the state interface input to the next `update()` cycle (FK "current position" for IK).

**REAL branch** (button press → real motor turns):
1-4. Same as SIM steps 1-4 — `controller` shares 100% of its FSM/IK/Tff code across both branches. Real hardware exposes `effort` as the per-joint BabyAlpha2 Tff field. Simulation uses `tau_ff_scale=1.0`; the current real-hardware tuning value lives in `controllers_real.yaml`, with a 1.0 s ramp and a controller-frame sign of `-J^T*F` confirmed by a zero-Tff real baseline.
5. `main_bot_hardware::RealSystem::write()` reads 12 independent position/velocity/kp/kd/effort command interfaces, independently clamps Tff to ±10 N·m, packs fixed arrays in `JointCmd{target_angle_mrad[12], target_velocity_mrad_s[12], kp_x100[12], kd_x100[12], tau_ff_mnm[12], seq}`, then publishes `/joint_cmd` over micro-ROS. `effort` is BabyAlpha2 feedforward torque in the same PD frame, not effort-only mode.
6. `micro_ros_agent` (running on the PC/RDK) forwards this DDS message over UART1 serial to the STM32H7.
7. STM32 (`microros_bridge.c`): `JointCmdCallback()` decodes all five 12-element arrays to SI units → calls `Actuator_SetTarget(angles, velocities, kp, kd, tau_ff)`.
8. `actuator_if.c` clamps each joint independently. `LogicalToRaw()` converts position with per-leg sign + home offset; velocity and Tff apply the same sign but never a position offset — see Critical invariants.
9. `baby_alpha2_protocol.c` packs a 12-byte CAN-FD PD frame containing target/v_des/kp/kd/Tff and sends it on the correct bus. `v_des` is offset-binary over ±45 rad/s (`0x8000` = zero); the operational clamp is 5 rad/s/joint on both EC and MCU.
10. The BabyAlpha2 joint driver board receives the frame and runs its own local PD loop (`pwm = kp*(target-measured) + kd*(v_des-measured_velocity) + tau_ff`) driving the motor and reading its encoder.
11. The driver board sends measured position/velocity/torque plus status back. STM32 decodes all three physical values and applies RAW→LOGIC sign conversion to each; position alone also receives the home offset.
12. `microros_bridge.c` publishes `/joint_fb` at 200 Hz with position/velocity/effort and a per-joint freshness mask, plus `/joint_diag` at 10 Hz with driver status, telemetry age, runtime-fault/bus-off masks and transport counters.
13. `RealSystem` writes only valid, fresh `/joint_fb` samples into the raw ros2_control position/velocity state interfaces. Before first feedback, after 100 ms without the topic, or when a validity bit clears, they become NaN and effort becomes zero so Stand/Sit cannot keep trusting stale state. Separate `visual_position`/`visual_velocity` interfaces retain the last finite sample (HOME `0` before the first one); `joint_state_broadcaster` maps only these display interfaces into `/joint_states`, so invalid control feedback cannot create NaN TF in `robot_state_publisher`/RViz.

**Safety note**: firmware boot begins with a best-effort `MOTOR_DISABLE` sweep so an MCU-only reset
cannot leave joint drivers holding an old PD target during initialization. If `/joint_cmd` stops or
the micro-ROS bridge disconnects, the MCU sends a zero-velocity/zero-Kp/zero-Kd/zero-Tff frame instead
of holding the last target. Runtime CAN bus-off, continuous telemetry loss, or an unexpected
Disabled/Standby driver status latches the affected joint offline and queues an explicit
`MOTOR_DISABLE`; recovery requires a full reboot/recalibration because hot-plug clears driver RAM limits.

## IMU data flow

**REAL:** J4 supplies the MPU6050 on I2C1 (`PB8=SCL`, `PB7=SDA`, address `0x68`). Firmware verifies
`WHO_AM_I`, configures ±2g/±250°/s, DLPF and 100 Hz sampling, then publishes fixed-size `ImuRaw` on
`/imu/raw`. Missing/read-failed IMU status is reported without blocking CAN, joint watchdog, or FSM.

**SIM:** the native Gazebo IMU on `imu_link` publishes noisy `sensor_msgs/Imu` on `/imu/sim_raw` at
100 Hz. `ros_gz_bridge` forwards it to ROS.

For both inputs, EC package `imu_kalman_filter` applies configurable axis permutation/sign, requires
200 consecutive stationary samples for gyro-bias calibration, runs roll/pitch angle+bias Kalman
filters with adaptive accelerometer covariance, and scalar Kalman filters on published accel/gyro.
It outputs `/imu/data` plus `/diagnostics`. Yaw is gyro-integrated only and carries high covariance
because MPU6050 has no magnetometer.

For sensor-only bring-up, `make imu-test` runs a dedicated launch containing only the
`micro_ros_agent`, Kalman node and read-only Tk monitor. Its preflight script rejects an occupied
serial device or an already-running real agent/controller; it never starts `ros2_control` and the
monitor creates no motor/control publishers.

## Message contract (`/joint_cmd`/`/joint_fb`/`/joint_diag`/`/imu/raw`)

- `/joint_cmd` (ROS2 -> MCU): `JointCmd{ int16[12] target_angle_mrad; int16[12] target_velocity_mrad_s; uint16[12] kp_x100; uint16[12] kd_x100; int16[12] tau_ff_mnm; uint8 seq }`. Every actuator has independent velocity/gains/Tff. Passive/ESTOP zeros velocity, Kp, Kd and Tff; Tff ramps up during Stand and back down during Sit. EC and MCU both clamp it per joint.
- `/joint_fb` (MCU -> ROS2): `JointFb{ int16[12] measured_angle_mrad; int16[12] measured_velocity_mrad_s; int16[12] measured_effort_mnm; uint16 valid_mask }`. Bit `i` is set only while joint `i` is initialized and its telemetry is younger than 100 ms.
- `/joint_diag` (MCU -> ROS2, 10 Hz): fixed-size `JointDiag` with ready/fresh/runtime-fault masks, raw driver status, telemetry age, CAN bus-off/TX failures, command sequence anomalies and micro-ROS publish failures.
- `/imu/raw` (MCU -> ROS2): `ImuRaw{int16[3] linear_acceleration_milli_ms2; int16[3]
  angular_velocity_mrad_s; uint32 stamp_ms; uint8 status}`. The MCU timestamp is delta-time only;
  the EC assigns ROS time to `/imu/data`.
- `target_velocity_mrad_s` comes from consecutive valid IK commands divided by the actual controller period. It resets to zero on state entry/exit, Passive/ESTOP, IK failure, invalid period, and firmware watchdog/disable.

## Critical invariants

Four things that, if violated, can reproduce real bugs or unsafe feedback in this repo.

### The "LOGIC vs RAW" joint-space split

- **LOGIC space**: the joint angle as ROS2/URDF understands it — the *same* number means the same physical pose across all 4 legs, because axis mirroring is already baked into the URDF per leg.
- **RAW space**: what's actually sent to the motor. Motors are mirrored per leg, so the *same* physical motion needs an opposite-sign RAW command depending on which leg (`MOTOR_JOINT_SIGN[LegGroup_t][JointType_t]` in `motor_calib.c`).
- Conversion happens in exactly one place: `LogicalToRaw()`/`RawToLogical()`, `LogicalVelocityToRaw()`, and `LogicalTorqueToRaw()` in `firmware/stm32h7/app/src/actuator_if.c`. Position uses sign + home offset; velocity/torque use sign only. Everything else operates in LOGIC space.

### "home = 0" calibration (the single most load-bearing invariant in this repo)

Joint value `0` (logical) means "robot lying prone" (nằm xấp). This requires **both** layers to be
correct simultaneously — fixing only one reproduces the bug:
1. **Firmware**: `g_home_offset_rad[]` in `actuator_if.c`, recomputed every boot (the BabyAlpha2
   driver's absolute zero drifts every power cycle per its spec) so that the currently-measured
   HOME position always maps to `ASSUMED_REST_LOGICAL_RAD = 0.0`.
2. **URDF**: `babydog.xacro`'s `<origin rpy>`/`<limit>` per joint are shifted to match
   (`abad=±0.360, hip=1.238, knee=-2.705` rad, limits shifted via `new = old − home`).

Any hardcoded pose/limit value written *before* this recalibration needs re-derivation against it —
this has been the root cause of multiple real bugs (stale sim `initial_value`s, an inverted firmware
limit table, stand/sit poses computed with the wrong transform). See `NOTE.md` §2.6 for the full
incident history before changing calibration-adjacent code.

Abad (hip-abduction) joints are a special case: all 4 share the same unmirrored `axis="1 0 0"`
(unlike hip/knee, whose axis mirrors per leg), so a shared stand/sit target for abad is wrong —
it needs opposite-sign values per leg side.

### CAN-FD, not Classic CAN

The MCU-to-joint-driver protocol is confirmed CAN-FD (`fd_format=true` in every built frame in
`baby_alpha2_protocol.c`, `CCCR_FDOE_Msk|CCCR_BRSE_Msk` set on the FDCAN peripheral, 12-byte payload
which physically exceeds Classic CAN's 8-byte max). `motor_topology.h` (renamed from
`motor_protocol.h`) only holds joint-index/bus/CAN-ID topology lookups, not wire encoding — the
actual frame encode/decode lives in `baby_alpha2_protocol.h/.c`.

### IMU axes are not motor signs

Raw MPU6050 axes are mapped once on the EC by `axis_map`/`axis_sign` in `imu_filter.yaml` into
`imu_link` (`x` forward, `y` left, `z` up). Never reuse `MOTOR_JOINT_SIGN` or change firmware sensor
signs to compensate for mounting. Identity is only a safe placeholder until the secured physical
axis test in `GUIDE.md` passes; active balance must stay disabled until then.

## Directory tree and file roles

```
src/
├── controller/                          FSM Passive/Stand-Sit + FK/IK, see Critical invariants
│   ├── controller.xml                     pluginlib plugin declaration (type: controller/StandSitController)
│   ├── msg/Inputs.msg                     command: uint8 (0 None/1 Stand/2 Sit/9 Estop)
│   ├── include/controller/
│   │   ├── StandSitController.h             Main controller class (ros2_control ChainableControllerInterface)
│   │   ├── CtrlInterfaces.h                  Wraps the command/state interfaces (position/velocity/kp/kd/torque...)
│   │   ├── common/mathTypes.h                Shared Eigen type aliases (Vec3, Mat3...), ported unchanged from superDog
│   │   ├── common/enumClass.h                Command constants (matches Inputs.msg), FSM state enum
│   │   ├── FSM/FSMState.h                    Abstract base for one FSM state (enter/run/exit)
│   │   ├── FSM/StatePassive.h                Passive state (torque=0, "soft")
│   │   ├── FSM/StateHoldPose.h                Stand/Sit shared state (Cartesian interpolation + IK)
│   │   └── robot/RobotLeg.h, robot/QuadrupedRobot.h   FK/IK/Jacobian for one leg / whole robot (KDL), ported from superDog
│   └── src/                                 .cpp files, 1:1 with the .h files above
├── leg_pd_controller/                     Low-level PD used in SIM only (copied unchanged from superDog)
│   └── src/LegPdController.cpp              tau = effort_ff + kp*(q_des-q) + kd*(qd_des-qd), writes into Gazebo
├── joystick_bridge/                       Python node: input -> /control_input
│   └── joystick_bridge/
│       ├── joystick_input.py                 Reads /joy (real gamepad), maps buttons -> Inputs.command
│       └── keyboard_input.py                 Reads terminal keyboard ('1'/'2'/'0') -> Inputs.command
├── main_bot_hardware/                     ros2_control <-> real firmware bridge
│   ├── main_bot_hardware.xml                 SystemInterface plugin declaration
│   └── src/real_system.cpp                   RealSystem: exports fail-safe raw + last-finite visualization
│                                              position/velocity states, measured effort and per-joint PD/Tff
│                                              commands; bridges /joint_cmd, /joint_fb and /joint_diag
├── main_bot_hardware_msgs/                Shared ROS2 <-> firmware message definitions
│   └── msg/JointCmd.msg, JointFb.msg, JointDiag.msg, ImuRaw.msg   See "Message contract" above
├── imu_kalman_filter/                    EC-side MPU6050/sim filtering
│   ├── src/imu_kalman.cpp                  testable angle+bias + scalar Kalman core
│   ├── src/imu_kalman_node.cpp             raw input, calibration, /imu/data + diagnostics
│   └── test/test_imu_kalman.cpp             synthetic bias/tilt/noise/dt tests
├── main_bot/                              Robot description + world + launch + config (no C++/Python logic)
│   ├── description/
│   │   ├── robot.urdf.xacro                   The REAL xacro entry point (what launch files process,
│   │   │                                       NOT babydog.xacro directly) — includes Gazebo IMU
│   │   ├── babydog.xacro                       Physical structure (link/joint/inertial), has the
│   │   │                                       calibrated "home=0" origin — see Critical invariants
│   │   ├── babydog.ros2_control.xacro           Declares ros2_control state/command interfaces (12 joints)
│   │   └── sensors/imu.xacro                    100 Hz noisy Gazebo counterpart of MPU6050
│   ├── config/
│   │   ├── controllers.yaml                    controller_manager config for SIM (has the "effort"
│   │   │                                       command interface — the Tff path, see Critical invariants)
│   │   ├── controllers_real.yaml                controller_manager config for REAL (PD + Tff interfaces)
│   │   ├── gz_bridge.yaml                       bridges /clock + /imu/sim_raw
│   │   └── imu_filter.yaml                      mapping, calibration and Kalman parameters
│   ├── launch/
│   │   ├── sim.launch.py                        Gazebo + RViz + controllers (simulation)
│   │   ├── real_ros2_control.launch.py           Full real-hardware stack (forces ROS_DOMAIN_ID=0)
│   │   ├── rz_sim.launch.py, rz_real.launch.py    RViz-only (doesn't reload the whole stack)
│   └── scripts/kill_gz.sh                       Cleans up stray Gazebo processes (used by `make kill`)
├── gui/                                  Tkinter tools
│   ├── gui/main_window.py                  sim/real/FSM + mutually-exclusive IMU-only launcher
│   ├── gui/imu_monitor.py                  read-only /imu/raw + /imu/data + diagnostics display
│   ├── launch/imu_real_test.launch.py      agent + Kalman + monitor only; no ros2_control
│   └── scripts/imu_real_test.sh            serial/process preflight + domain/workspace setup

firmware/stm32h7/
├── main.c                                 Main loop — wires tick/CAN/micro-ROS/actuator together
├── Makefile                               `make`/`make flash`/`make clean` (see Commands)
├── app/inc, app/src/                      Application code specific to this Stand/Sit phase
│   ├── motor_topology.h                     joint<->bus<->CAN-ID table (JointIndex_t/LegGroup_t/JointType_t)
│   ├── motor_calib.h/.c                     Calibration constants (Kp/Kd max, limit table, MOTOR_JOINT_SIGN)
│   ├── actuator_if.h/.c                     LogicalToRaw/RawToLogical (the ONE place space conversion
│   │                                        happens), adaptive home offset, sends commands + reads CAN feedback
│   ├── baby_alpha2_protocol.h/.c             Real 12-byte CAN-FD frame encode/decode per vendor spec
│   ├── microros_bridge.h/.c                   micro-ROS node, sub /joint_cmd, pub /joint_fb + /joint_diag + /imu/raw
│   ├── app_i2c.h/.c, mpu6050.h/.c             app-local I2C1 + 100 Hz MPU6050 acquisition
│   ├── microros_transport.h/.c                UART1 transport layer for micro-ROS (ISR-driven RX ring)
│   ├── microros_time.c                        clock_gettime() for rcutils, backed by Tick_GetMs() (no FreeRTOS)
│   └── tick.h/.c                              Free-running millisecond counter (TIM2), replaces lib/systick.h (blocking)
└── lib/inc, lib/src/                      Shared register-level drivers (rcc/gpio/can/tim/systick/usart/dma...),
                                           copied from ~/OUT_SAVE/babyDog_fwSTM/, DO NOT modify (see firmware README)
```

## Maintenance policy for this file and `NOTE.md`

Use judgment, not a mechanical rule — but both files exist to stay trustworthy, so don't churn them
on routine work either.

- **`CLAUDE.md`** holds the project's living structure (architecture, data flow, invariants) — not a
  scratchpad. Only edit it when something actually changes at that level:
  - a structural change (package/directory added or removed, a plugin/interface renamed, a
    responsibility moved between modules),
  - a major phase/milestone completing (see Roadmap — e.g. Phase 1 finishing, Phase 2 IMU work
    landing),
  - the data/workflow flow itself changing (a step added/removed/reordered, the message contract
    changing).
  Don't edit it for a one-off fix, an exploratory session, or every commit — if nothing at that level
  moved, this file doesn't need to move either.
- **`NOTE.md`** is not a log of every fixed bug. Only add an entry for something **genuinely severe**
  (physical/safety risk) or that **took a long, non-obvious debugging effort** — a bug class a future
  session would likely waste real time re-diagnosing if it recurred (sign/limit-table mismatches,
  micro-ROS session instability, CAN bus-off, `make flash` not resetting the chip are the existing
  examples). An ordinary fix that a commit message already explains adequately doesn't need one.

## Where to look next

- **`NOTE.md`** — running incident log for real-hardware bugs meeting the bar above (root causes,
  fixes, what was ruled out). Check it before re-investigating something that looks like a
  previously-solved bug class.
- **`CONTEXT.md`** — point-in-time snapshot written by the `precompact` skill right before context
  is compacted (architecture notes, in-progress work, open questions). Overwritten each run, not a
  history log — check it after resuming a compacted session.
- **`GUIDE.md`** — task-oriented walkthrough (how to run sim/real/firmware, joystick remapping).
- **`firmware/stm32h7/README.md`** — what has and hasn't actually been verified on real hardware.
