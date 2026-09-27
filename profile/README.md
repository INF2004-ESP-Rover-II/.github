# INF2004 ESP – Rover II: Project Overview

*Autonomous Room-Mapping Rover Upgrade*
*Last updated: 2026-09-27*

---

## 1. What This Project Is

Rover II is the upgrade of an existing mecanum-wheel rover, separate from
INF2004's default graded module project (the line-following car). The base
rover already drives manually with no orientation sensing. This project adds:

- **Phase 1 – Mapping:** 3D room mapping using distance sensors and a gyroscope
  for orientation, replacing the current LIDAR + claw setup.
- **Phase 2 – Navigation:** Autonomous A* pathfinding using the map built in
  Phase 1, plus ArUco-marker localization.

The rover's differential/mecanum drivetrain gives it strafe and diagonal
mobility that a normal 4-wheel rover doesn't have — exploiting that is called
out as a "big requirement" for the mapping/navigation approach, not just an
incidental feature.

## 2. Current System Architecture

Two Raspberry Pi Pico (W) boards, communicating over UART:

- **`firmware/robo_pico/`** — motor control (mecanum drive)
- **`firmware/sensor_pico/`** — sensor aggregation: currently IMU (LSM303DLHC),
  4x HC-SR04 ultrasonic sensors, RPLIDAR A1 (UART1 IRQ-driven), SD card
  logging. Sends a packed sensor struct to the Robo Pico over UART0.
- **`dashboard/robot_dashboard.py`** — host-side visualization/control

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
| **Mapping approach** | Horizontal scan by default, with on-demand vertical scanning for full 3D coverage (rather than continuous 3D scanning). |
| **Org name** | Jeremy created "INF2004 ESP - Rover II" as the organisation for this work. |

## 4. Current Status

| Item | Status |
|---|---|
| Standalone 3-ToF bring-up test (`read_3_tofs.c`) | **Working**, confirmed by Jeremy. Used as the reference implementation for sensor init/read pattern. |
| ToF sensors integrated with the real `sensor_pico` firmware (IMU + ultrasonic + 3x ToF together) | **In progress.** A teammate has reportedly completed this integration, but the code has not yet been pushed to the shared repo. It has not been reviewed or verified against the pin/bus-speed plan above — treat it as unconfirmed until it's pushed and checked. |
| `integration_test.c` (a parallel bring-up test built this session, living in `3TOF/integration_test/`, not the real firmware) | Written, not yet compiled or flashed — exists to prove the wiring/pin/bus-speed plan works using the real drivers verbatim, but is unverified hardware-side. Once the teammate's version is pushed, compare the two rather than assuming either is final. |
| ArUco marker localization | Not started. |
| A* navigation (Phase 2) | Not started. |
| IMU placement (Sensor Pico vs Robot Pico) | Recommendation given (keep on Sensor Pico, given current loop timing), not finalized. |

## 5. Open Questions / Decisions Still Needed

- Final IMU placement — Sensor Pico (current) vs moving to Robot Pico.
- Navigation compute: where does A* actually run (onboard Pico vs host dashboard)?
- Encoders — are wheel encoders being added for odometry, or is mapping purely sensor-fusion based?
- Target room size / mapping tolerance — drives sensor range/accuracy requirements.
- Whether the claw is being removed (raised as an option alongside the LIDAR swap, not yet confirmed as decided).
- Whether Rover II needs to follow the INF2004 module's Week 4 process rules (public repo, pin-map sign-off before wiring, AI-usage documentation, Week 8 tribunal recording) — only relevant if this is being submitted as/alongside the module's default project. Worth clarifying with the instructor if there's any ambiguity.

## 6. Risks / Gotchas to Track

- **I2C bus speed ordering** (see §3) — easy to reintroduce silently if init order changes later.
- **XSHUT pin conflicts** — any future pin reassignment needs to be checked against the full existing pin map (motor control, ultrasonic x4, IMU, UART links, SD card SPI), not just the sensors being added. GP23/GP24/GP25/GP29 are also reserved internally on Pico **W** boards for the wireless chip and are not usable as general GPIO — relevant since the current firmware drives GP25 (LED) directly, which likely needs to go through `cyw43_arch_gpio_put()` instead on real Pico W hardware.
- **Credential exposure**: `PINOUT.md` in the Senior Project repo currently contains a plaintext WiFi SSID/password and MQTT broker IP. If this repo (or a Rover II repo derived from it) is ever made public, these need to be rotated and removed first.
- **Loop timing**: the sensor_pico main loop is not a clean fixed-rate loop today — four sequential ultrasonic reads with ~15ms gaps plus timeouts already stretch a nominal 20Hz loop. Adding 3 ToF polls will stretch it further; worth measuring actual loop time once the real integration is in, especially if heading/IMU timing accuracy matters for mapping.
- **Don't overstate verification status** in any documentation or reporting: "working" should mean actually compiled, flashed, and tested on hardware — not just written or reasoned about as correct.

## 7. Suggested Build Order

1. Get the teammate's ToF integration pushed and merged.
2. Verify it against the pin map and I2C bus-speed plan above; reconcile with the parallel `integration_test.c` if useful.
3. Confirm real hardware bring-up (compile, flash, log output) for IMU + ultrasonic + 3x ToF together.
4. Measure actual loop timing with all sensors active.
5. Decide open items in §5 (IMU placement, encoders, navigation compute).
6. Build the mapping pipeline (horizontal scan → map; on-demand vertical scan).
7. Add ArUco localization.
8. Add A* navigation (Phase 2).

---
*This overview reuses the structure of the original `overview.md` design document, updated with decisions and status confirmed during this session.*
