# INF2004 ESP – Rover II: Project Overview

*Autonomous Marker-Grid Exploration Rover Upgrade*
*Last updated: 2026-09-30 (aligned with system statechart v7)*

---

## 1. What This Project Is

Rover II is the upgrade of an existing mecanum-wheel rover, separate from
INF2004's default graded module project (the line-following car). The base
rover already drives manually with no orientation sensing. This project adds
two autonomous phases that run on a **grid of ArUco markers**:

- **Phase 1 – Explore & Map:** the rover explores the grid on its own using
  3 ToF distance sensors (front / left / right) and an IMU for orientation,
  replacing the current LIDAR + claw setup. Every marker's ID says whether it
  is a **junction** or a **plain (non-junction)** marker. At each junction the
  rover picks an unexplored exit; when every exit is blocked or already
  explored it backs up to the most recent junction that still has unexplored
  exits. Phase 1 ends **only when all N markers have been found**.
- **Phase 2 – Go to Marker (manual target):** once the map is complete, the
  operator picks a target marker (e.g. "go to marker 1, then marker 5"). The
  rover plans a route with A* over the map from Phase 1 and drives it marker by
  marker. It does not give up: if an exit is blocked it marks it, reroutes, and
  keeps trying until it reaches the target.

The rover's mecanum drivetrain gives it strafe and diagonal mobility that a
normal 4-wheel rover doesn't have — exploiting that (moving in one continuous
motion instead of stop / turn / go) is called out as a "big requirement" for
the exploration/navigation approach, not just an incidental feature.

The rover is told only **how many markers exist (N)** and **what the two
marker types are**. It is **not** given the grid layout.

## 2. Current System Architecture

Two Raspberry Pi Pico (W) boards communicating over UART, plus an ESP32-S3
camera and a laptop:

- **`firmware/robo_pico/`** — motor control (mecanum drive), WiFi/MQTT, and the
  top-level state machine (currently a LIDAR-based reactive wander `AUTO_*`
  state machine that will be replaced by the statechart in §3.1).
- **`firmware/sensor_pico/`** — sensor aggregation: IMU (LSM303DLHC),
  3x VL53L1X ToF (`tof.c`), 4x HC-SR04 ultrasonic sensors, RPLIDAR A1 (UART1
  IRQ-driven, being removed), SD card logging. Sends a packed sensor struct to
  the Robo Pico over UART0.
- **ESP32-S3 camera (`ESP32-LogiCam-Test/firmware`)** — USB-host UVC webcam,
  joins WiFi and serves an MJPEG stream (`/stream`) and snapshots (`/capture`).
- **Laptop** — `dashboard/robot_dashboard.py` (Tkinter + MQTT) and the ArUco
  detection scripts (`ESP32-LogiCam-Test/software/*.py`, OpenCV). ArUco
  detection runs here, **not** on the ESP32 or the Picos.
- **Planned data path for localisation:** ESP32 video → laptop (ArUco decode) →
  MQTT → `robo_pico`. Not implemented yet.

Repositories in the org: `integrated-rover-ii` (firmware + dashboard),
`ESP32-LogiCam-Test`, `IMU-Test`, `3TOF-Test`, and this overview repo.

### 2.1 Sensor roles

| Sensor | Role in the statechart |
|---|---|
| ToF FRONT / LEFT / RIGHT | The three candidate exits at a junction; obstacle detection while driving (far limit → exit blocked / reroute, very close → emergency stop) |
| IMU | Heading during localisation and driving between markers |
| ESP32 camera + laptop ArUco | Marker ID (junction / plain), marker count, position fix at each marker |
| Ultrasonic x4 | Existing close-range sensing (role to be confirmed) |

## 3. Settled Decisions

These are confirmed and should be treated as the current plan, not open
questions:

| Decision | Detail |
|---|---|
| **LIDAR removal** | The RPLIDAR A1 is being removed entirely (weight + no longer needed for the ToF-based approach). |
| **Replacement sensor** | 3x VL53L1X Time-of-Flight sensors (LEFT / FRONT / RIGHT), replacing LIDAR for horizontal distance sensing. |
| **Sensor addresses** | LEFT `0x30`, FRONT `0x31`, RIGHT `0x32` (default boot address `0x29`, reassigned one at a time via XSHUT sequencing). |
| **XSHUT pin relocation** | Moved from the original test's GP6/GP7/GP8 to **GP18 (LEFT), GP19 (FRONT), GP20 (RIGHT)** — GP6/GP7 are already the left ultrasonic sensor's trig/echo pins on the real board, so the original wiring would have conflicted. |
| **I2C bus sharing** | ToF sensors share I2C0 with the IMU. The IMU driver (`accelero.c`) initializes the bus at 100kHz; ToF sensors need 400kHz. `i2c_init()` reconfigures the whole peripheral, so **call order matters** — whichever init runs last sets the speed for every device on the bus. The integration must explicitly re-init I2C0 to 400kHz after the IMU init, or add a bus abstraction that avoids the conflict entirely. |
| **Mapping approach** | **Superseded by the statechart (see §3.1).** The old wording was "horizontal scan by default, with on-demand vertical scanning for full 3D coverage". The statechart is 2D and marker-based, and the org profile was already changed from 3D to 2D. Vertical scanning is not part of the statechart; the team should confirm it is dropped (see §5). |
| **Org name** | Jeremy created "INF2004 ESP - Rover II" as the organisation for this work. |

### 3.1 Decisions from the system statechart review (v7)

Agreed during the statechart review on 2026-09-30. The team should confirm
them, but they are the working plan.

**Environment and markers**

| Decision | Detail |
|---|---|
| **Grid of ArUco markers** | Markers are arranged in a grid. Each marker ID maps to one of two types: **junction** or **not junction**. |
| **What the rover knows** | Only N (the number of markers) and the two marker types. **Not** the layout or positions. |
| **Marker count** | "All N markers found" must count **distinct marker IDs**, not scans, otherwise revisiting a marker while backing up would double-count it. |

**Phase 1 – Explore & Map**

| Decision | Detail |
|---|---|
| **Stop condition** | Phase 1 ends only when all N markers have been found; the rover then enters MAP COMPLETE / GO-TO READY. |
| **Plain marker** | Count it and keep driving. |
| **Junction marker** | Enter AT JUNCTION and pick an unexplored exit: front, left, or right (the rover came from the rear). |
| **Blocked exit** | If an obstacle is seen within the far limit, mark that exit blocked, return to the junction, and try the next exit. There is no global replan counter; the limit is the 3 exits at that junction. |
| **Backing up** | When all 3 exits at a junction are blocked or explored, use A* to go back to the **most recent** junction that still has unexplored exits. |
| **Missed marker** | The rover cannot tell at the time that it passed a marker unscanned. This shows up only when exploration is exhausted with fewer than N found, which is a **Fault**. The operator restarts the rover manually. |

**Phase 2 – Go to Marker**

| Decision | Detail |
|---|---|
| **Target selection** | Manual: the operator picks a target marker from GO-TO READY. |
| **Persistence** | The rover keeps trying until it reaches the target. On an obstacle it marks the exit blocked (REROUTE) and re-plans with A*. There is no retry limit. |
| **Only exit besides success** | "No route left" (every path to the target is blocked) returns to GO-TO READY. |
| **Localise** | A marker scan plus IMU heading at each marker, between driving segments. |

**Operator control and safety**

| Decision | Detail |
|---|---|
| **Operator stop** | Available at any time. From Phase 1 it returns to READY. From Phase 2 it returns to GO-TO READY (map kept). |
| **Obstacle thresholds** | Two distances: a **far limit** (mark exit blocked / reroute) and a **very close** distance (emergency stop → Fault). Values are still to be measured. |
| **Fault states** | Init fail, self-check fail, sensor timeout / link lost, emergency stop, Phase 2 marker timeout, and Phase 1 "exploration exhausted (fewer than N found)". Recovery is an operator reset back to SELF-CHECK. |

### 3.2 System statechart (v7)

States: BOOT & INIT → SELF-CHECK → READY (N set) → **PHASE 1: EXPLORE & MAP**
(AT JUNCTION, DRIVE TO NEXT MARKER, EXIT BLOCKED, BACK UP) → MAP COMPLETE /
GO-TO READY → **PHASE 2: GO TO MARKER** (PATH PLANNING, LOCALISE, EXECUTE
SEGMENT, REROUTE) → back to GO-TO READY. FAULT / SAFE STOP is reachable from
any state.

| From | To | Trigger |
|---|---|---|
| Boot & Init | Self-check | init complete |
| Self-check | Ready | pass |
| Ready | Phase 1 (At Junction) | start explore `[N set]` |
| At Junction | Drive to Next Marker | exit chosen |
| Drive to Next Marker | At Junction | junction marker scanned |
| Drive to Next Marker | Drive to Next Marker | plain marker scanned (count it, keep going) |
| Drive to Next Marker | Exit Blocked | obstacle within far limit |
| Exit Blocked | At Junction | back at junction |
| At Junction | Back Up | all 3 exits blocked / explored |
| Back Up | At Junction | junction reached |
| Phase 1 | Map Complete / Go-To Ready | all N markers found |
| Map Complete / Go-To Ready | Path Planning | target marker set |
| Path Planning | Localise | path computed |
| Localise | Execute Segment | pose fixed |
| Execute Segment | Localise | marker reached |
| Execute Segment | Reroute | obstacle seen |
| Reroute | Path Planning | reroute |
| Path Planning | Go-To Ready | no route left |
| Localise | Go-To Ready | target reached (result sent over MQTT) |
| Phase 1 | Ready | operator stop |
| Phase 2 | Go-To Ready | operator stop |
| any / listed states | Fault | see the fault list in §3.1 |
| Fault | Self-check | operator reset |

Diagram files: `rover_system_statechart_v7.png` and `rover_statechart_v7.md`
(PlantUML and Mermaid/ELK source). Add them to the repo and link them here.

## 4. Current Status

| Item | Status |
|---|---|
| Standalone 3-ToF bring-up test (`3TOF-Test`, `read_3_tofs.c`) | **Working**, confirmed by Jeremy. Used as the reference implementation for sensor init/read pattern. |
| ToF sensors in the real `sensor_pico` firmware (`integrated-rover-ii`) | **Pushed** (commit "Initial project + ToF sensor readings", 2026-09-27): `tof.c` does the XSHUT sequencing and address assignment, and `main.c` prints the readings. **Not yet in the UART packet**, so `robo_pico` cannot see them. Hardware verification is not recorded in the repo. `VL53L1X_platform.h` still sets `VL53L1X_I2C_BAUDRATE` to 100000, which needs checking against the 400kHz plan. |
| `integration_test.c` (parallel bring-up test in `3TOF-Test/integration_test/`) | Written, not yet compiled or flashed. Compare against the pushed `tof.c` rather than assuming either is final. |
| IMU | One LSM303DLHC driver in `sensor_pico` (heading from the magnetometer; gyro fields are zero). `IMU-Test` holds a GY-6500 reader sketch (config file added 2026-09-30). Which IMU provides heading is not decided. |
| ArUco detection | Laptop-side Python scripts exist (`ESP32-LogiCam-Test/software`) and the ESP32 firmware streams video. **Not connected to the rover.** The junction / plain ID scheme, distinct-ID counting and the MQTT link to `robo_pico` are not started. |
| Rover state machine | `robo_pico` still has the LIDAR-based `AUTO_*` wander. Boot self-check, fault handling, operator stop, Explore and Go-to-Marker states are **not implemented**. |
| System statechart | **v7 done** (see §3). The subsystem statecharts and the Week 6 design document still need updating to match. |
| A* (Phase 1 back-up and Phase 2) | Not started. |
| Dashboard | Still LIDAR-oriented. Needs N entry, target-marker selection, state display, and stop / reset controls. |
| IMU placement (Sensor Pico vs Robot Pico) | Recommendation given (keep on Sensor Pico, given current loop timing), not finalized. |

## 5. Open Questions / Decisions Still Needed

- Final IMU placement — Sensor Pico (current) vs moving to Robot Pico — and which IMU provides heading.
- Navigation compute: where does A* run (onboard Pico vs laptop)? The graph is now small (the marker grid), so either is feasible. A laptop is easier because ArUco detection already runs there, but it needs the WiFi/MQTT link, and a lost link is already a Fault.
- Encoders — are wheel encoders being added? Without them, position between markers comes from IMU and time, and the markers give the only reliable position fix.
- Grid size, marker spacing and N. This replaces the earlier "target room size".
- ArUco ID scheme: which IDs are junctions and which are plain, and which dictionary is used.
- Start condition: the statechart assumes the rover starts at a marker. Confirm.
- Far-limit and very-close distances (mm), to be measured.
- Phase 1 stop: after an operator stop, does exploring resume (keeping what was learned) or restart from scratch? Also, after a Fault reset, is the partial map kept?
- Phase 2: do blocked-exit marks expire, or wait and retry, in case an obstacle was temporary?
- How the operator enters N and picks the target marker (dashboard over MQTT is the obvious route).
- Vertical / 3D scanning: the statechart is 2D. Confirm it is dropped (the older text in §3 still mentions it).
- Whether the claw is being removed (raised as an option alongside the LIDAR swap, not yet confirmed as decided).
- Whether Rover II needs to follow the INF2004 module's Week 4 process rules (public repo, pin-map sign-off before wiring, AI-usage documentation, Week 8 tribunal recording) — only relevant if this is being submitted as/alongside the module's default project. Worth clarifying with the instructor if there's any ambiguity.

## 6. Risks / Gotchas to Track

- **I2C bus speed ordering** (see §3) — easy to reintroduce silently if init order changes later. The pushed ToF library defines 100kHz, so confirm which speed the bus actually ends up at.
- **XSHUT pin conflicts** — any future pin reassignment needs to be checked against the full existing pin map (motor control, ultrasonic x4, IMU, UART links, SD card SPI), not just the sensors being added. GP23/GP24/GP25/GP29 are also reserved internally on Pico **W** boards for the wireless chip and are not usable as general GPIO — relevant since the current firmware drives GP25 (LED) directly, which likely needs to go through `cyw43_arch_gpio_put()` instead on real Pico W hardware.
- **Credential exposure**: `PINOUT.md` in the Senior Project repo currently contains a plaintext WiFi SSID/password and MQTT broker IP. If this repo (or a Rover II repo derived from it) is ever made public, these need to be rotated and removed first.
- **Loop timing**: the sensor_pico main loop is not a clean fixed-rate loop today — four sequential ultrasonic reads with ~15ms gaps plus timeouts already stretch a nominal 20Hz loop. Adding 3 ToF polls will stretch it further; worth measuring actual loop time once the real integration is in, especially if heading/IMU timing accuracy matters.
- **Missed markers are found late.** By design, a marker passed unscanned only shows up when exploration is exhausted with fewer than N found, and the operator must restart. Document this in the report.
- **Marker counting.** Count distinct IDs. A false or misread ID could end Phase 1 early or hide a missing marker, so consider requiring a confirmed read before an ID counts.
- **Localisation depends on seeing a marker.** Camera field of view, lighting and motion blur affect this. A Phase 2 marker timeout is a Fault.
- **Dependence on the laptop link.** ArUco results reach the rover over WiFi/MQTT, so a link loss stops the rover (Fault).
- **Two obstacle distances.** The far limit and the very-close distance must not overlap or swap. The emergency stop should not wait on the state machine; it belongs in the motor control task.
- **LIDAR still in the firmware** while being removed, and the existing `AUTO_*` wander depends on it. Remove the two together to avoid a half-working state.
- **Don't overstate verification status** in any documentation or reporting: "working" should mean actually compiled, flashed, and tested on hardware — not just written or reasoned about as correct.

## 7. Suggested Build Order

1. Verify the pushed ToF integration on hardware against the pin map and I2C bus-speed plan; reconcile with `integration_test.c` if useful.
2. Add the three ToF readings to the UART packet so `robo_pico` can use them.
3. Measure actual loop timing with all sensors active.
4. Decide the open items in §5 (IMU, encoders, A* location, ID scheme, thresholds).
5. Build the safe skeleton of the statechart on `robo_pico`: Boot, Self-check, Ready, Fault, operator stop and emergency stop, replacing the `AUTO_*` wander.
6. Connect ArUco: laptop detection → MQTT → `robo_pico`, with junction / plain IDs and distinct-ID counting.
7. Build Phase 1: junction logic, blocked-exit marking, back-up (A*) to the most recent junction with unexplored exits, and the all-N-found stop.
8. Build Phase 2: target selection, A*, localise / execute loop, reroute.
9. Update the dashboard (N entry, target selection, state display, stop / reset).
10. Remove the LIDAR and its code once nothing depends on it.

---
*This overview reuses the structure of the original `overview.md` design document, updated with the decisions agreed in the system statechart review (v7) on 2026-09-30. Repo status was read from the org repositories on the same date.*
