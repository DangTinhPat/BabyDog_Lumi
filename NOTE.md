# NOTE.md — Real-Hardware Incident Log (babyDog)

Debugging history for **real hardware only** (not sim): root causes, fixes, and system facts learned
during bring-up. Append a new entry after fixing a real bug or landing a non-trivial calibration
change — see `CLAUDE.md` → "Where to look next" for the policy.

---

## 1. Hardware reference

```
PC/RDK X5 (ROS2 Jazzy)
   │  micro-ROS over UART1 (/dev/ttyUSB0, 921600 baud, PA9=TX/PA10=RX on MCU)
   │  topics: /joint_cmd (down), /joint_fb (up)
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
- **Fix (currently active)**: `microros_transport.c` has `#define TX_BLOCKING_POLLING_TEST 1`, which
  switches `Transport_Write()` to blocking byte-by-byte polling instead of the ring buffer. Verified
  stable across repeated 30-40s tests: 0 re-establishes, `/stm32_joint_node` fully visible, TF
  updating correctly.
- **Open item**: the original `tx_ring_enqueue()` ring-buffer code is still present but unused
  (`-Wunused-function`). Still to decide: keep blocking polling permanently (simple, proven) or
  rebuild the ring buffer with correct write-confirmation.
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

## 3. Known open issues

### 3.1. No CAN bus-off runtime monitoring
- `g_joint_ok[j]` is only set **once at boot** in `Actuator_Init()` and never re-evaluated.
  `CAN_IsBusOff()` (already implemented in `can.c`) is **never called** in the main loop.
- **Risk**: if a CAN bus enters Bus-Off state during real operation, firmware won't detect it,
  won't self-recover (re-run `CAN_Start()`), and won't alert anyone — stuck until a manual reset.
- **Proposed fix (not yet done)**: periodic check (e.g. every 1s) in `main.c`'s main loop calling
  `CAN_IsBusOff(instance)` for both buses, auto-recovering via `CAN_Start(instance)`; consider
  surfacing bus health on `/joint_fb` or a dedicated topic.

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
ros2 control list_controllers
```

For build/flash/launch commands, see `CLAUDE.md` → "Commands".
