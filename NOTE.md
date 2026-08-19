# NOTE.md — Real-Hardware Incident Log (babyDog)

Debugging history for **real hardware only** (not sim): root causes, fixes, and system facts learned
during bring-up. Append a new entry after fixing a real bug or landing a non-trivial calibration
change — see `CLAUDE.md` → "Where to look next" for the policy.

---

## 1. Hardware reference

```
PC/RDK X5 (ROS2 Jazzy)
   │  micro-ROS over UART1 (/dev/ttyUSB0, 921600 baud, PA9=TX/PA10=RX on MCU)
   │  topics: /joint_cmd (down), /joint_fb + /joint_diag (up)
   ▼
STM32H7 (FK743M5-XIH6, Cortex-M7 480MHz)
   │  CAN-FD, 12-byte frames, BabyAlpha2 protocol (baby_alpha2_protocol.h)
   │  2 independent buses:
   ├─ CAN_INSTANCE_1 (FDCAN1, PA11=RX/PA12=TX, AF9) → 6 FRONT-leg drivers (ID 1-6)
   └─ CAN_INSTANCE_2 (FDCAN2, PB5=RX/PB6=TX, AF9)  → 6 HIND-leg drivers  (ID 1-6)
   ▼
12× "BabyAlpha2" driver boards (run their own local PD loop from kp/kd sent by the MCU)
```

**Joint → bus/ID mapping** (`motor_topology.h`, verified against vendor docs
https://github.com/DungTranBK/BabyAlpha2_Docs):

| JointIndex | Name | Bus | CAN ID |
|---|---|---|---|
| 0,1,2 | FRONT_RIGHT abad/hip/knee | CAN_INSTANCE_1 | 1,2,3 |
| 3,4,5 | FRONT_LEFT abad/hip/knee | CAN_INSTANCE_1 | 4,5,6 |
| 6,7,8 | HIND_RIGHT abad/hip/knee | CAN_INSTANCE_2 | 1,2,3 |
| 9,10,11 | HIND_LEFT abad/hip/knee | CAN_INSTANCE_2 | 4,5,6 |

Formula: `bus = joint<6 ? CAN_INSTANCE_1 : CAN_INSTANCE_2`, `id = (joint%6)+1`.

**BabyAlpha2 driver notes** (per vendor docs):
- Needs a stable **24V–28V** supply to reach the PING-ready state.
- CAN ID is **fixed by driver firmware**: commands go to ID 0x001–0x006, but every reply
  (PING/HANDSHAKE/SETUP/telemetry) comes back on the shared ID 0x000, disambiguated by `data[0]`.
- No DIP switch or other ID configuration exists.

**Related files/dirs**:
- `firmware/stm32h7/` — MCU firmware (own Makefile: `make` / `make flash`).
- `src/main_bot_hardware/` — `RealSystem` (ros2_control hardware_interface; bridges `/joint_fb` ↔
  state interfaces, `/joint_cmd` ↔ command interfaces).
- `src/controller/` — FSM Passive/HoldPose (Stand/Sit), the ros2_control controller.
- `src/main_bot/launch/real_ros2_control.launch.py` — main real-hardware launch file (forces
  `ROS_DOMAIN_ID=0`).
- `/home/dvt/OUT_SAVE/babyDog_test/oneLeg/` — reference project, already proven on real hardware
  (`leg1_proven_2026-08-08/`, `main_12joint_hold_proven.c.bak`) — cross-check here whenever firmware
  behavior is in doubt.

---

## 2. Resolved firmware bugs

### 2.1. `CAN_Start()` didn't confirm it actually left Init mode
- **Symptom**: CAN never sent a single frame, no clear error.
- **Root cause**: wrote `CCCR.INIT=0` once and assumed success, without reading back to confirm. On
  real hardware, `CCCR` sometimes still reads `INIT=1` afterward.
- **Fix**: `CAN_Start()` changed from `void`→`bool`; clears `CCCR_INIT_Msk|CCCR_CCE_Msk` together and
  polls for confirmation (with timeout) before returning `true`.
- File: `firmware/stm32h7/lib/src/can.c`.

### 2.2. Wrong CAN-FD data-phase bit timing (DBTP)
- **Symptom**: no PING response even after the 2.1 fix.
- **Root cause**: data-phase was set to nominal (1 Mbit/s) under the assumption "BRS=false so DBTP
  doesn't matter" — wrong per the Bosch M_CAN spec (CRC fixed-stuff-bit behavior for CAN-FD depends
  on data-phase config even when a given frame has BRS=0).
- **Fix**: `data_tseg1=3, data_tseg2=1, data_sjw=1` (5 Mbit/s), matching the already-proven oneLeg
  config.
- File: `firmware/stm32h7/main.c` (`CAN_BUS_CONFIG`).

### 2.3. Self-inflicted regression: `while(1)` hang on `CAN_Start()` failure
- Added the same hang guard as `CAN_Init()`, which stalled ALL of micro-ROS if either CAN bus failed
  — losing the ability to debug remotely over `/joint_fb`.
- **Fix**: call `(void)CAN_Start(...)` non-blocking; the system keeps booting even if a bus fails
  (safe: a failed joint reports TIMEOUT over ROS2 instead of receiving wrong commands).

### 2.4. micro-ROS session instability (repeated re-establish)
- **Symptom**: `/stm32_joint_node` never fully appeared; agent log alternated "session established"
  / "session re-established" roughly every ~10s.
- **Root cause**: `Transport_Write()` used an async TX ring buffer (interrupt-driven) — the XRCE
  client library called `write()` and immediately started its response timeout, without waiting for
  confirmation that bytes had actually left the UART.
- **Fix (currently active)**: `Transport_Write()` permanently uses blocking byte-by-byte polling.
  The unused asynchronous TX ring was removed after repeated 30-40s tests showed 0 re-establishes,
  `/stm32_joint_node` fully visible and TF updating correctly. RX remains interrupt-driven through
  its independent ring buffer.
- File: `firmware/stm32h7/app/src/microros_transport.c`.
- **Ruled out**: raw USART1 register/GPIO layer (polling test, 80,000 bytes, 0 errors) and the
  ISR+ring-buffer RX layer (tested through production `Transport_Open()`/`Transport_Read()`, 80,000
  bytes, 0 errors) — neither was the cause.

### 2.5. `make flash` didn't reset the chip after writing
- **Symptom**: flashing new firmware repeatedly but the board kept running OLD behavior — caused
  hours of "I fixed the code but it's still broken" confusion.
- **Root cause**: the board's NRST pin isn't wired to the ST-Link, so `st-flash write` without
  `--reset` doesn't restart the core after flashing — the CPU can be left hung/crashed (its running
  flash region was just overwritten underneath it) until manually reset.
- **Fix**: `Makefile`'s `flash` target now runs `st-flash --reset write ...`.
- File: `firmware/stm32h7/Makefile`.

### ⭐ 2.6. [Load-bearing invariant] "Angle 0 = prone pose" calibration — needs BOTH layers

**The single most important thing to check first when debugging any pose/display issue.** Fixing
only one of these two layers reproduces the bug — this was reverted and reapplied several times
before both were fixed together:

| | Before fix | After fix (current, active) |
|---|---|---|
| **Firmware** (`actuator_if.c`) | `RawToLogical`/`LogicalToRaw` only flipped sign (±1), no offset — reported values were raw encoder counts that **drift randomly every power cycle** | Added `g_home_offset_rad[12]` + `ASSUMED_REST_LOGICAL_RAD=0.0`, recomputed every boot so the reported HOME position always reads exactly **0.000**, regardless of encoder drift |
| **URDF** (`babydog.xacro`) | `origin rpy="0 0 0"` — angle=0 draws the geometric **"fully extended"** pose (links stacked in a straight line) | `origin rpy` rotated (abad=±0.360, hip=1.238, knee=-2.705 rad, `<limit>` shifted to match) — angle=0 now draws the actual **prone pose** |
| **Combined result** | firmware reports 0 (or a random drifted value) + URDF treats "0" as "extended" → RViz **always shows the wrong extended pose** | firmware always reports exactly 0 at boot + URDF treats "0" as "prone" → RViz **always shows the correct prone pose**, stable across every power cycle |

**Why both layers are required**: fixing only firmware makes "0" stable but still means "extended" in
the URDF (looks like a new bug). Fixing only the URDF makes "0" mean "prone" but the reported value
still drifts every session, so it's rarely actually 0. Both together give an angle-0 convention that
is stable AND correctly meaningful.

**How the URDF values were derived**: measured via free-fall in Gazebo (controller in
Passive/torque=0), letting the robot settle under gravity, then reading `/joint_states`. Confirmed
prone (`z≈0.054m`, orientation ≈ flat) across three independent measurements (no damping / with
damping / low friction) that converged on the same values — ruling out damping/friction artifacts.
Final values: **`abad=0.360, hip=-1.238, knee=2.705`** rad.
Compensation formula (verified by rotation-matrix math, error ~1e-16): for `axis="1 0 0"` (abad) →
`origin rpy = (home, 0, 0)`; for `axis="0 -1 0"` (hip/knee) → `origin rpy = (0, -home, 0)`. Limits
must shift by the same amount (`new = old − home`) since they don't move automatically with origin.

**Current status (verify these two spots first if "extended pose" ever reappears)**:
1. `firmware/stm32h7/app/src/actuator_if.c` — `g_home_offset_rad`/`ASSUMED_REST_LOGICAL_RAD` present,
   `LogicalToRaw`/`RawToLogical` applying the offset.
2. `src/main_bot/description/babydog.xacro` — the 12 revolute joints' `origin rpy` are
   `±0.360`/`1.238`/`-2.705`, not `0 0 0`.
3. **Real hardware only**: this fix is complete for sim/URDF/RViz. On the physical robot, firmware
   must also apply `g_home_offset_rad` at boot (see above) for `/joint_fb` to report a calibrated
   value — both layers must stay in sync.

---

### 2.7. Tff sign must be selected from a zero-feedforward baseline

- **Symptom**: at real scale `0.70`, the robot tiptoed/folded its hips, hummed and oscillated;
  synchronized `/joint_cmd` + `/joint_fb` showed Tff opposite the instantaneous P term on all 12
  joints.
- **Debugging trap**: comparing Tff against `Kp*(q_des-q_measured)` while Tff is already active does
  **not** identify the correct feedforward sign. The closed-loop P term moves to oppose an injected
  feedforward torque, even when that feedforward uses the correct convention. This false criterion
  briefly led to changing `kTffSign` from `-1` to `+1`; a scale-`0.10` capture again showed Tff
  opposite P on 12/12 joints, falsifying the criterion rather than validating the sign flip.
- **Decisive test**: set real `tau_ff_scale=0`, keep the same target and Kp/Kd, capture a settled
  Stand sample, then evaluate the controller's KDL Jacobian at those exact measured angles. The
  zero-Tff PD baseline matched `-J(q)^T F_support` on **11/12 joints** and `+J(q)^T F_support` on
  only **1/12**; `dot(PD,-J^T F)=+28.74` versus `dot(PD,+J^T F)=-28.74`. The lone FR-hip exception
  had only `|J^T F|=0.020 N.m`, near that joint's sign crossing and too small to select a convention.
- **Fix/current convention**: `RobotLeg::calcTorque()` remains the generic `+J^T F`; only
  `StateHoldPose` applies `kTffSign=-1`. Firmware must not compensate again: position and Tff both
  use the same `MOTOR_JOINT_SIGN` when converting LOGIC to RAW. Real scale restarts conservatively
  at `0.10`; do not return directly to `0.70`.
- **Post-fix real verification**: with `kTffSign=-1`, scale `0.10`, and a settled 502-sample window
  (`v_des=0` on all 12 joints), Tff aligned with the zero-Tff load baseline on 12/12 joints. Absolute
  target error decreased on 10/12 joints and aggregate joint-PD magnitude (`sum(abs(P+D))`) dropped
  from `19.07` to `9.72 N.m`. The two error exceptions were the front hip joints; keep scale at
  `0.10` until their load/pose behavior is characterized instead of increasing the global scale.

### 2.8. Watchdog timestamp underflow can alternate active/passive CAN frames

- **Failure scenario**: `main.c` sampled `now_ms`, then `MicroRosBridge_SpinSome()` ran a callback
  that stored a newer `last_joint_cmd_ms`. If the callback crossed a 1 ms tick boundary,
  `now_ms - last_joint_cmd_ms` underflowed as `uint32_t` and looked like a huge timeout.
- **Physical effect**: one valid command could send 12 active PD frames and the same superloop
  iteration immediately followed them with watchdog zero-Kp frames. Repetition can present as buzz,
  vibration or hesitant motion even though `/joint_cmd` is arriving at the expected rate.
- **Fix**: sample watchdog time after `SpinSome()`, disable immediately when the bridge reports
  disconnected, and use an explicit received-command flag plus command counter instead of timestamp
  value `0` as a sentinel. Agent health ping is bounded to 50 ms x 2, below the 200 ms watchdog;
  the fail-soft frame now zeros velocity, Kp, Kd and Tff.
- **Verification status**: the exact source-level underflow path is eliminated and firmware builds
  cleanly; confirm on hardware by checking that `/joint_diag` sequence/TX counters remain stable.

### 2.9. Passive damping caused false runtime faults; invalid feedback poisoned TF

- **Symptom**: the dedicated zero-gain transport test was stable, but the normal real launch quickly
  changed `ready_mask=0x03E` into `runtime_fault_mask=0x03E`. RViz then printed continuous
  `TF_NAN_INPUT`/`TF_DENORMALIZED_QUATERNION` errors for all 12 moving links.
- **Root cause**: `StatePassive` still sent `Kd=1.0`. Damping is torque-producing, so firmware
  correctly treated the command as active and required Active driver status; the five responding
  drivers reported Standby `0x0001`, which latched them offline. Their cleared `valid_mask` made
  `RealSystem` publish NaN raw state; `joint_state_broadcaster` forwarded it into TF.
- **Fix**: Passive/ESTOP now sends velocity/Kp/Kd/Tff all zero. `RealSystem` keeps NaN on raw
  position/velocity for FSM safety, but exports separate last-finite visualization interfaces that
  `joint_state_broadcaster` maps into `/joint_states`.
- **Verification**: a 20-second host/hardware run observed 3,981 zero-only commands at 200.001 Hz,
  3,981 finite 12-joint state messages, 0 TF NaN and 0 denormalized quaternions. The old MCU fault
  latch remained because no ST-Link was detected for reset; repeat boot verification is still
  required before any Stand/Sit command.
- **Related diagnostic trap**: restarting the EC publisher while the MCU remains up resets the host
  sequence to zero. Old firmware reports the cross-process jump as packet loss. Source now resets
  the sequence baseline at the watchdog boundary; this change built successfully but was not flashed
  in the same session because the ST-Link was absent.

### 2.10. Unverified SETUP echo gate disabled seven valid joints

- **Symptom**: after firmware hardening, RViz updated only the front legs and diagnostics reported
  `ready_mask=fresh_mask=0x03E`; joint 0 and all six FDCAN2 joints appeared offline even though the
  same wiring and drivers had previously worked.
- **Root cause**: initialization promoted the undocumented full `SETUP_LIMITS` echo into a mandatory
  readiness gate. Several driver revisions provide valid PING/HANDSHAKE/HOME telemetry but do not
  return the expected SETUP ACK/payload. Firmware then cleared `g_joint_ok`, sent `MOTOR_DISABLE` and
  stopped probing those joints. Their preserved nonzero status plus `runtime_fault_mask=0` proved
  that CAN2 had worked during boot and the rejection happened after HOME, not in the CAN peripheral.
- **Fix**: keep `SETUP_LIMITS` best-effort as in the proven 12-joint firmware. PING, HANDSHAKE and
  fresh HOME telemetry remain mandatory; the MCU independently clamps every target before CAN TX,
  so a missing SETUP ACK never bypasses the firmware limit table.
- **Verification**: after flashing with `--reset`, zero-force operation reported
  `ready_mask=fresh_mask=0xFFF`, all telemetry ages about 2 ms, `runtime_fault_mask=0`, both CAN buses
  not bus-off, and finite `/joint_states` for all 12 joints. Position, velocity, Kp, Kd and Tff in
  `/joint_cmd` were all zero.

### 2.11. Direct 250 Hz command relay drove both CAN buses bus-off

- **Test scenario**: with the robot zero-force, EC `controller_manager.update_rate` and MCU
  `/joint_fb` were raised together from 200 Hz/5 ms to 250 Hz/4 ms. UART payload estimates suggested
  enough byte bandwidth, but each received command also causes 12 CAN-FD PD frames and corresponding
  driver telemetry, so UART capacity alone was not a valid acceptance criterion.
- **Observed failure**: `/joint_cmd` remained entirely zero, but MCU diagnostics latched
  `can_bus_off_mask=0x03`, `runtime_fault_mask=0xFFF`, `ready_mask=0`, followed by loss of outbound
  `/joint_fb`, `/imu/raw` and `/joint_diag`. The firmware safety path disabled all joints; the real
  stack was stopped immediately.
- **Recovery/current limit**: restore both EC and MCU to 200 Hz, rebuild, flash with `--reset`, and
  require a clean boot. Post-rollback verification gave `ready_mask=fresh_mask=0xFFF`, telemetry age
  3 ms, no bus-off/runtime fault, `/joint_cmd=200.003 Hz`, `/joint_fb` about 199.5-200 Hz, and zero
  sequence/publish errors after more than 11,000 zero-force commands.
- **Rule**: 200 Hz is the highest verified real-hardware rate. Do not retry a direct 250 Hz relay
  until CAN waveform/load and driver processing limits are measured, or command computation is
  decoupled from a separately bounded CAN transmit scheduler. The exact physical/protocol cause of
  bus-off at 250 Hz remains to be isolated.

### 2.12. Status `0x0001` caused a false 12-joint fault on the first Stand command

- **Symptom**: the zero-force 200 Hz baseline was stable with all 12 joints ready, but the first
  Stand command immediately produced `runtime_fault_mask=0xFFF`, `ready_mask=fresh_mask=0`, while
  `can_bus_off_mask=0`. The EC then printed repeated all-zero `Tff settled diag` blocks.
- **Root cause**: runtime safety treated raw driver status `0x0001` as proven Standby. All 12 physical
  drivers continued to report exactly `0x0001` after accepting the first active PD frame, so the
  newly-set `active_required` flag converted valid telemetry into a false fault and queued
  `MOTOR_DISABLE` for every joint. The exact status contract is revision-dependent and remains
  unverified on this driver firmware.
- **Fix**: `0x0001` is now diagnostic-only; only all-zero `0x0000` is treated as explicitly Disabled.
  Freshness timeout, CAN bus-off detection, command watchdog and all MCU clamps remain active. On the
  EC, Stand/Sit now zeros Kp/Kd/Tff and restarts from a coherent snapshot whenever any raw feedback
  is invalid; the settled diagnostic also requires valid feedback for all 12 joints.
- **Verification**: source/build verification and `st-flash --reset` completed successfully. After
  reconnecting USB-UART, the zero-force run reached `ready_mask=fresh_mask=0xFFF` while all 12
  drivers reported `status=0x0001`; runtime fault and bus-off stayed zero through 22,866 commands,
  with no sequence or publish errors. Steady-state rates were `/joint_cmd=200.00 Hz`,
  `/joint_fb=199.6-199.8 Hz`, and `/imu/raw=100 Hz`.
- **Remaining staged test**: no active command was sent. After physically restoring the prone HOME
  pose and resetting the MCU again, 11 measured joints were `0.000 rad` and the remaining joint was
  `-0.005 rad`, with clean masks/counters. The HOME prerequisite now passes; the first force-bearing
  Stand remains a user-controlled physical test.

### 2.13. Incoherent FK/IK start snapshot could hold HOME with active gains

- **Symptom**: after pressing Stand, the joints became stiff but the commanded position stayed at
  HOME and the interpolation never advanced. Motor signs, CAN topology and URDF joint axes were
  unchanged from the known-working `origin/main` baseline.
- **Root cause**: a controller safety patch clamped an encoder sample used as the first IK seed but
  still computed the Cartesian start foot from the unclamped sample. A knee only `+0.005 rad` past
  its exact URDF upper limit (`0.0`) therefore described two different starting configurations. The
  first constrained IK solve failed every cycle while Kp/Kd were already enabled, which exactly
  produced a stiff robot holding HOME.
- **Fix**: make one finite measured snapshot, permit at most `0.02 rad` of encoder/backlash excess,
  clamp it once, and use that same bounded snapshot for both FK and IK. Larger excess is rejected
  before any Kp/Kd/Tff is enabled; the warning is emitted once rather than at the 200 Hz control
  rate.
- **Verification**: an offline real-stack test with synthetic `+0.005 rad` knee feedback reached
  the expected logical Stand targets (`abad=+/-0.360`, `hip=+0.490`, `knee=-1.210 rad`) without an
  IK error. A `+0.050 rad` test kept all Kp/Kd/Tff fields at zero. Gazebo Stand and the controller
  build also passed. A secured physical Stand retest remains operator-controlled.

## 3. Known open issues

### 3.1. CAN bus-off recovery is deliberately manual
- Runtime now checks both buses every superloop. Bus-off latches all affected joints offline,
  queues `MOTOR_DISABLE`, clears their `/joint_fb.valid_mask` bits and reports the fault on
  `/joint_diag`.
- Firmware deliberately does **not** call `CAN_Start()` and resume force automatically. A driver
  reset/hot-plug can erase RAM limits and home state, so recovery requires a controlled reboot with
  the robot in its required prone calibration pose.

---

## 4. Operational safety rules

Moved to `CLAUDE.md` → "⚠️ Mandatory operating rules" (single source of truth — do not duplicate
here). That section covers: single `real_ros2_control.launch.py` process at a time, `pkill`
verification, `ROS_DOMAIN_ID=0` for ad-hoc terminals, the 3 required sourced workspaces, and
`--reset` on flashing.

---

## 5. Quick diagnostic commands

```bash
# Check for a conflicting process before launching real hardware
lsof /dev/ttyUSB0
ps aux | grep -E "micro_ros_agent|ros2_control_node|robot_state_publisher"

# Read live joint data
export ROS_DOMAIN_ID=0
ros2 topic echo /joint_fb --once
ros2 topic echo /joint_diag --once
ros2 control list_controllers
```

For build/flash/launch commands, see `CLAUDE.md` → "Commands".
