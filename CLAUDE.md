# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

`babyDog`: a real 4-legged robot (RDK X5/laptop + STM32H7 MCU), current scope is **Stand/Sit only**
(no gait/walking, no IMU yet), controlled via joystick/keyboard. Includes both a Gazebo simulation
(ROS 2 Jazzy + Gazebo Harmonic) and the real-hardware stack. It's a deliberately trimmed-down fork
of `../superDog` (full walking-gait project) — reuses the physical description + `leg_pd_controller`
but drops gait/balance/IMU since this phase doesn't need them.

## Roadmap

Current repo scope is **Phase 1 of 3** — a longer-term plan, not the final destination. Don't treat
the missing pieces below as bugs or gaps to fill unprompted; they're intentionally out of scope for
now.

- **Phase 1 (current)** — basic system so the robot can stand up / sit down, and move with a basic
  gait (simple, not yet optimized). No IMU, no active balance.
- **Phase 2 (planned)** — add IMU, active/dynamic balance while moving.
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

## Commands

All commands run from the repo root (`babyDog/`). Run `make build` once before anything else.
The `Makefile` wraps `ros2 launch`/`ros2 run`/`ros2 topic pub` so you don't need to source
workspaces by hand — every ROS2 target auto-sources `/opt/ros/jazzy/setup.bash` + `install/setup.bash`.

```bash
make build              # colcon build --symlink-install (whole ROS2 workspace)
make sim                # Gazebo + RViz + controllers (simulation)
make sim-no-rviz        # same, no RViz
make gui                # Tkinter control panel (start/stop sim+rviz, Stand/Sit/Estop, logs)
make keyboard           # keyboard teleop: '1'=stand '2'=sit '0'=estop
make joystick           # real joystick/gamepad -> /control_input
make stand / sit / estop  # one-shot CLI command, no joystick needed
make controllers        # ros2 control list_controllers (verify 3 controllers "active")
make kill               # kill stray Gazebo processes

make real                # real hardware + RViz (requires firmware flashed, micro_ros_agent workspace sourced)
make real-no-rviz        # same, no RViz (e.g. over SSH)

make firmware            # build firmware/stm32h7 (needs arm-none-eabi-gcc in PATH)
make firmware-flash      # build + flash via ST-Link (st-flash --reset write)
make firmware-clean

make clean                # rm -rf build install log
```

To build/test a single ROS2 package: `colcon build --packages-select <pkg>` (source
`/opt/ros/jazzy/setup.bash` first). There is no automated test suite in this repo — "testing" so far
means running `make sim` and observing behavior, or (for firmware) flashing to real hardware.

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
              ▼                                       /joint_fb via micro-ROS)
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

## Data flow: input to actuation

The diagram above shows boxes; this is the step-by-step sequence a button press actually travels
through. Both branches share steps 1-4 (same FSM/IK code) and diverge only in how the resulting
joint targets get to the motors.

**SIM branch** (button press → Gazebo joint moves):
1. `joystick_bridge` (`joystick_input.py`/`keyboard_input.py`) reads `/joy` or keyboard → publishes `/control_input` (`controller/msg/Inputs{command}`).
2. `StandSitController::update()` (runs every ros2_control cycle) reads `/control_input` → FSM picks a state (`StatePassive`/`StateHoldPose`, the latter shared by both Stand and Sit).
3. `StateHoldPose::run()`: tanh-interpolates the target foot position in Cartesian space → `solveFootIK()` (`RobotLeg::calcQPosition`, KDL LMA solver) → per-cycle joint angle targets.
4. Writes joint angles + kp/kd (and `effort` if settled and `tau_ff_scale>0`) into ros2_control command interfaces (`joint_position_command_interface_` etc., see `CtrlInterfaces.h`).
5. `leg_pd_controller::update()` reads those interfaces + current state → computes `tau = effort_ff + kp*(q_des-q) + kd*(qd_des-qd)` → writes the `effort` interface `gz_ros2_control` exports to Gazebo.
6. Gazebo applies this torque to the 12 physical joints in sim.
7. `joint_state_broadcaster` reads joint state from Gazebo → publishes `/joint_states` → drives RViz (via `robot_state_publisher`/TF) and feeds back as the state interface input to the next `update()` cycle (FK "current position" for IK).

**REAL branch** (button press → real motor turns):
1-4. Same as SIM steps 1-4 — `controller` shares 100% of its FSM/IK code across both branches; they only diverge in how the resulting joint angles get consumed. On real, `effort` is never written (the interface doesn't exist in `controllers_real.yaml`, so Tff is automatically inert).
5. `main_bot_hardware::RealSystem::write()` reads the position/kp/kd command interfaces → packs `JointCmd{target_angle_mrad, kp_x100, kd_x100, seq}` → publishes `/joint_cmd` over micro-ROS.
6. `micro_ros_agent` (running on the PC/RDK) forwards this DDS message over UART1 serial to the STM32H7.
7. STM32 (`microros_bridge.c`): `JointCmdCallback()` (rclc executor callback on new data) decodes mrad→rad, kp_x100/kd_x100→float → calls `Actuator_SetTarget(angles_rad, kp, kd)`.
8. `actuator_if.c`: stores the target; `LogicalToRaw()` converts the 12 angles from LOGIC to RAW space (per-leg sign flip via `MOTOR_JOINT_SIGN`, adds `g_home_offset_rad`) — see Critical invariants.
9. The main loop (`main.c`) periodically calls `baby_alpha2_protocol.c`'s `BA2_BuildPdFrame()` to pack a 12-byte CAN-FD frame (target/kp/kd, `v_des` hardcoded ~0) and sends it via `can.c` on the correct bus (`CAN_INSTANCE_1`/`_2` per `motor_topology.h`).
10. The BabyAlpha2 joint driver board receives the frame and runs its own local PD loop (`pwm = kp*(target-measured) + kd*(0-measured_velocity)`) driving the motor and reading its encoder.
11. The driver board sends a telemetry frame (measured position/velocity) back — STM32 receives it over CAN, decodes with `RawToLogical()` (RAW→LOGIC).
12. `microros_bridge.c`'s `MicroRosBridge_PublishJointFb()` periodically publishes `/joint_fb` (`measured_angle_mrad`, `measured_velocity_mrad_s` × 12) back up over micro-ROS/UART.
13. `RealSystem` receives `/joint_fb` → writes it into the state interfaces → feeds `joint_state_broadcaster` (→ `/joint_states` → RViz TF) and becomes the FK input for the next `update()` cycle.

**Safety note**: if `/joint_cmd` stops arriving (firmware watchdog timeout), the MCU cuts torque
rather than holding the last command — this is why a lost connection on real hardware fails soft
instead of leaving the robot frozen under load.

## Message contract (`/joint_cmd`/`/joint_fb`)

- `/joint_cmd` (ROS2 -> MCU): `JointCmd{ int16[12] target_angle_mrad; uint16 kp_x100; uint16 kd_x100; uint8 seq }` — one shared kp/kd pair for all 12 joints (architectural constraint, not per-joint).
- `/joint_fb` (MCU -> ROS2): `JointFb{ int16[12] measured_angle_mrad; int16[12] measured_velocity_mrad_s }`.
- There is no `vel_des` field anywhere in this pipeline even though the BabyAlpha2 CAN-FD protocol supports it — firmware always encodes velocity desired as ~0. Known gap, not yet implemented.

## Critical invariants

Three things that, if violated, reproduce real bugs that have already happened in this repo.

### The "LOGIC vs RAW" joint-space split

- **LOGIC space**: the joint angle as ROS2/URDF understands it — the *same* number means the same physical pose across all 4 legs, because axis mirroring is already baked into the URDF per leg.
- **RAW space**: what's actually sent to the motor. Motors are mirrored per leg, so the *same* physical motion needs an opposite-sign RAW command depending on which leg (`MOTOR_JOINT_SIGN[LegGroup_t][JointType_t]` in `motor_calib.c`).
- Conversion happens in exactly one place: `LogicalToRaw()`/`RawToLogical()` in `firmware/stm32h7/app/src/actuator_if.c`. Everything else (limits, home offset, targets) operates purely in LOGIC space.

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

## Directory tree and file roles

```
src/
├── controller/                          FSM Passive/Stand-Sit + FK/IK, see Critical invariants
│   ├── controller.xml                     pluginlib plugin declaration (type: controller/StandSitController)
│   ├── msg/Inputs.msg                     command: uint8 (0 None/1 Stand/2 Sit/9 Estop)
│   ├── include/controller/
│   │   ├── StandSitController.h             Main controller class (ros2_control ChainableControllerInterface)
│   │   ├── CtrlInterfaces.h                  Wraps the command/state interfaces (position/kp/kd/torque...)
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
│   └── src/real_system.cpp                   RealSystem: exports state/command interfaces, publishes
│                                              /joint_cmd, subscribes /joint_fb over micro-ROS
├── main_bot_hardware_msgs/                Shared ROS2 <-> firmware message definitions
│   └── msg/JointCmd.msg, msg/JointFb.msg     See "Message contract" above
├── main_bot/                              Robot description + world + launch + config (no C++/Python logic)
│   ├── description/
│   │   ├── robot.urdf.xacro                   The REAL xacro entry point (what launch files process,
│   │   │                                       NOT babydog.xacro directly) — doesn't include
│   │   │                                       sensors/*.xacro yet since the real robot has no IMU/camera
│   │   ├── babydog.xacro                       Physical structure (link/joint/inertial), has the
│   │   │                                       calibrated "home=0" origin — see Critical invariants
│   │   ├── babydog.ros2_control.xacro           Declares ros2_control state/command interfaces (12 joints)
│   │   └── sensors/imu.xacro                    IMU sensor for Gazebo — WRITTEN but NOT YET included
│   │                                            into robot.urdf.xacro (prepped for Roadmap Phase 2)
│   ├── config/
│   │   ├── controllers.yaml                    controller_manager config for SIM (has the "effort"
│   │   │                                       command interface — the Tff path, see Critical invariants)
│   │   ├── controllers_real.yaml                controller_manager config for REAL (no "effort")
│   │   └── gz_bridge.yaml                       ros_gz_bridge topic list for sim — no /imu yet
│   ├── launch/
│   │   ├── sim.launch.py                        Gazebo + RViz + controllers (simulation)
│   │   ├── real_ros2_control.launch.py           Full real-hardware stack (forces ROS_DOMAIN_ID=0)
│   │   ├── rz_sim.launch.py, rz_real.launch.py    RViz-only (doesn't reload the whole stack)
│   └── scripts/kill_gz.sh                       Cleans up stray Gazebo processes (used by `make kill`)
├── gui/gui/main_window.py                 Tkinter control panel (start/stop sim, Stand/Sit/Estop, logs)

firmware/stm32h7/
├── main.c                                 Main loop — wires tick/CAN/micro-ROS/actuator together
├── Makefile                               `make`/`make flash`/`make clean` (see Commands)
├── app/inc, app/src/                      Application code specific to this Stand/Sit phase
│   ├── motor_topology.h                     joint<->bus<->CAN-ID table (JointIndex_t/LegGroup_t/JointType_t)
│   ├── motor_calib.h/.c                     Calibration constants (Kp/Kd max, limit table, MOTOR_JOINT_SIGN)
│   ├── actuator_if.h/.c                     LogicalToRaw/RawToLogical (the ONE place space conversion
│   │                                        happens), adaptive home offset, sends commands + reads CAN feedback
│   ├── baby_alpha2_protocol.h/.c             Real 12-byte CAN-FD frame encode/decode per vendor spec
│   ├── microros_bridge.h/.c                   micro-ROS node (`stm32_joint_node`), sub /joint_cmd, pub /joint_fb
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
