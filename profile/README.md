# INF2004 ESP – Rover II

*Autonomous Omni-Wheel Mapping Rover · INF2004 Embedded Systems Programming · Group P1-SE10*
*Last updated: 2026-10-23 (Week 6 Design Review "updated proposal", Tribunal 1 Versions A–C, repos as of 2026-10-23)*

---

## 1. What this project is

Rover II upgrades an existing mecanum-wheel rover that could only be driven by hand. It
adds two autonomous phases on a **grid of ArUco markers**:

- **Phase 1 – Explore & Map.** The rover explores by **depth-first search (DFS)** on the
  Robo Pico, using 3 ToF sensors (front / left / right) and a GY-6500 gyro for heading.
  Each marker ID says whether it is a **junction** or a **plain** marker. At a junction the
  rover takes an unexplored exit; a blocked exit is marked and the next one tried; when a
  junction is used up it backtracks along its DFS stack. The laptop builds the map from
  the rover's events. Phase 1 ends **only when all N distinct markers are found**.
- **Phase 2 – Go to Marker.** The operator picks a target on the dashboard; the **laptop
  runs A\*** and sends the route **one segment at a time**. A blocked exit is marked and the
  route re-planned, with **no retry limit**.
- **Manual driving** is available in READY, including the mecanum moves (strafe,
  diagonal, arc turn, rotate on the spot, parallel park).

The rover is told only **N** and the **two marker types**, not the grid layout. There are
**no wheel encoders**: distance is open-loop and position is corrected at each marker.
Autonomous driving uses **forward/back and rotate on the spot only**; the other mecanum
moves are manual-only.

## 2. Repositories

| Repo | What it holds | Owner | Toolchain |
|---|---|---|---|
| [`integrated-rover-ii`](https://github.com/INF2004-ESP-Rover-II/integrated-rover-ii) | Main codebase: `firmware/robo_pico/`, `firmware/sensor_pico/`, `dashboard/robot_dashboard.py` | Everyone | Pico SDK 2.3.1, C; Python dashboard |
| ↳ branch [`JiaHui`](https://github.com/INF2004-ESP-Rover-II/integrated-rover-ii/tree/JiaHui) | Dashboard rework, `fake_rover` test script ([`37dd313`](https://github.com/INF2004-ESP-Rover-II/integrated-rover-ii/commit/37dd313d0d31994587137a936b0884e7a7eccb95)), dashboard link watchdog, Robo Pico FR-08 link timeout | Jia Hui | **Not merged into `main` yet** |
| [`ESP32-LogiCam-Test`](https://github.com/INF2004-ESP-Rover-II/ESP32-LogiCam-Test) | ESP32-S3 USB-host UVC → MJPEG stream (`/stream`, `/capture`); laptop ArUco detection, camera calibration, distance/bearing (`software/`) | Jeremy | ESP-IDF **v5.2.7** (v6.x breaks `usb_stream`); Python + `opencv-contrib-python` |
| [`Motor-Control-Test`](https://github.com/INF2004-ESP-Rover-II/Motor-Control-Test) | Standalone Motor Control module (kinematics, L298N, heading hold, safe stop), host unit tests, FreeRTOS bench firmware. Merged into `integrated-rover-ii` via PR #1 | Zi Ying | Pico SDK + FreeRTOS-Kernel; host `gcc` for tests |
| `Mapping-Path-Planning` (private) | Placeholder for the laptop map + A\* planner. **Empty so far** | Jeslyn | Python (planned) |
| [`3TOF-Test`](https://github.com/INF2004-ESP-Rover-II/3TOF-Test) | Working 3× VL53L1X bring-up (`read_3_tofs/`) and `integration_test/`. Note: `read_3_tofs` uses the **old** XSHUT pins GP6/7/8; the rover now uses GP18/19/20 | Jeremy (bring-up for Subsystem 3) | Pico SDK, C |
| [`IMU-Test`](https://github.com/INF2004-ESP-Rover-II/IMU-Test) | GY-6500 (MPU6500) reader sketch: SDA GP6, SCL GP7, address `0x68` | Jeslyn (bring-up for Subsystem 6) | Arduino (arduino-pico core) |
| `.github` | This page (`profile/README.md`) | — | Markdown |

**Build quick reference**

```bash
# integrated-rover-ii (both Picos)
export PICO_SDK_PATH=/path/to/pico-sdk
mkdir build && cd build && cmake .. && make -j4
#   -> firmware/robo_pico/mecanum_robot.uf2, firmware/sensor_pico/sensor_pico.uf2 (BOOTSEL + copy)
cd dashboard && pip install paho-mqtt && python robot_dashboard.py

# ESP32-LogiCam-Test
. /path/to/esp-idf-v5.2.7/export.sh
cd firmware && idf.py set-target esp32s3 && idf.py -p <port> flash monitor
cd software && python3 esp32_aruco_stream.py      # set ESP32_IP first

# Motor-Control-Test
cd test && make                                    # host unit tests
```

> Wi-Fi credentials are hard-coded in some firmware and docs. Do not copy them into
> issues, reports or this page, and rotate them before any public demo.

## 3. System architecture

Two Raspberry Pi Picos (each running its own FreeRTOS) linked by UART, an ESP32-S3
camera node, and a laptop.

| Part | Role |
|---|---|
| **Sensor Pico** (Pico, FreeRTOS) | 3× VL53L1X ToF + GY-6500 IMU; Collision Detection (emergency stop); sends 20 Hz data frames and an immediate STOP frame to the Robo Pico |
| **Robo Pico W** (Pico W, FreeRTOS) | WiFi/MQTT client, UART receive, Exploration (DFS + segment driving), Motor Control |
| **ESP32-S3 + Logitech USB webcam** | USB-host UVC, MJPEG over WiFi. **No detection on the ESP32** |
| **Drive** | 2× L298N on a 12 V motor-only battery, 4 DC motors with mecanum wheels. Picos on a separate battery, common ground |
| **Laptop** | Python + Mosquitto: Mission Controller, Dashboard, Logger, Map Builder + A\*, ArUco Detector |

### 3.1 Tasks and owners

| Task | Board | Purpose | Rate / trigger | Priority | Owner |
|---|---|---|---|---|---|
| Collision Detection | Sensor Pico | Compare ToF with the very-close distance, trigger emergency stop | Every ToF cycle | **Highest** | Vivi |
| ToF Sensing | Sensor Pico | Poll 3 ToF, discard invalid readings, ToF timeout fault | ~100 ms | High | Vivi |
| IMU | Sensor Pico | Read gyro, zero at start-up, integrate yaw into heading | 10 ms (100 Hz) | High | Zi Ying |
| Sensor Comms | Sensor Pico | Pack ToF + heading into UART frames; send STOP at once | 50 ms (20 Hz) + STOP on event | Medium | Jia Hui |
| Motor Control | Robo Pico | Commands → PWM for 4 wheels, heading hold, ramping, immediate stop | 20 ms; STOP notification wakes it at once | High | Zi Ying |
| Robo Comms | Robo Pico | UART receive, MQTT client, routing; stops motors on laptop link loss | Event-driven | Medium | Jia Hui |
| Exploration | Robo Pico | DFS, blocked exits, backtracking; executes Phase 1 and 2 segments | Event-driven | Medium | Vivi |
| Camera Stream | ESP32-S3 | Capture UVC MJPEG and stream to the laptop | Per frame | Separate node | Jeremy |

Laptop programs are Python processes, not RTOS tasks: Mission Controller + Dashboard +
Logger (Jia Hui), Map Builder + A\* (Jeslyn), ArUco Detector (Jeremy).

**Emergency stop path:** Collision Detection → immediate UART STOP frame (not queued behind
data) → Robo Pico UART interrupt buffers it and notifies Motor Control directly → PWM cut
with no ramp. Target: physical halt within **200 ms** (NFR-03), not yet measured.

### 3.2 Pin map

**Sensor Pico**

| Device | Signal | GPIO | Interface | Notes |
|---|---|---|---|---|
| 3× VL53L1X ToF | SDA / SCL | GP4 / GP5 | I2C0, 400 kHz | Shared bus |
| ToF LEFT / FRONT / RIGHT | XSHUT | GP18 / GP19 / GP20 | GPIO out | Addresses `0x30` / `0x31` / `0x32` set at boot |
| GY-6500 IMU | SDA / SCL | GP6 / GP7 | I2C1 | Address `0x68`, own bus, no magnetometer |
| Ultrasonic FRONT (HC-SR04) | TRIG / ECHO | GP26 / GP27 | GPIO | Existing, not used by current FRs |
| Ultrasonic BACK (HC-SR04) | TRIG / ECHO | GP0 / GP1 | GPIO | Existing, not used by current FRs |
| SD card | SCK / MOSI / MISO / CS | GP10 / GP11 / GP12 / GP15 | SPI1 | Kept for now. **MOSI/MISO order to be verified** |
| UART to Robo Pico | TX → RX | GP16 → GP13 | UART0 | 20 Hz frames + immediate STOP |
| UART from Robo Pico | RX ← TX | GP17 ← GP12 | UART0 | Movement direction (4-bit mask + 4-bit sequence) for Collision Detection |

**Robo Pico W**

| Device | Signal | GPIO | Interface | Notes |
|---|---|---|---|---|
| Front L298N | ENA / ENB | GP4 / GP5 | PWM 10 kHz | |
| Front L298N | IN1–IN4 | GP3 / GP2 / GP16 / GP17 | GPIO | |
| Back L298N | ENA / ENB | GP27 / GP26 | PWM 10 kHz | PWM slice 5 (see risk R7) |
| Back L298N | IN1–IN4 | GP28 / GP7 / GP0 / GP1 | GPIO | GP28 needs the VBAT jumper open; stdio over USB only |
| Arm motor | M1B (up) / M1A (down) | GP10 / GP11 | GPIO on/off | Not used by current FRs; driven low at boot. Shares PWM slice 5 with GP26/GP27, so no PWM |
| Claw motor | M1A (close) / M1B (open) | GP8 / GP9 | PWM / GPIO | Not used by current FRs; driven low at boot |
| UART to Sensor Pico | RX / TX | GP13 / GP12 | UART0 | |
| WiFi | — | onboard CYW43 | WiFi / MQTT | GP23/24/25/29 reserved on the Pico W |

**ESP32-S3:** Logitech webcam on the USB-OTG port (UVC, MJPEG, 5 V from the ESP32);
streams to the laptop over WiFi.

**Power:** 12 V battery for the two L298N drivers only; separate battery for the Picos;
grounds connected.

## 4. Team

| Buddy | Name | GitHub | Subsystem | FRs |
|---|---|---|---|---|
| 1 | Genevivi Mia Er (Vivi) | `Meditix1` | 3. Exploration & Obstacle Sensing | FR-16 – FR-23 |
| 2 | Lee Ru Yan Jeslyn | `Jes000` | 4. Mapping & Path Planning | FR-24 – FR-29 |
| 3 | Lee Zong Yang Jeremy | `jialomi` | 5. Vision Localisation | FR-30 – FR-36 |
| 4 | Lim Zi Ying | `limziying` | 6. Motion Control & Heading | FR-37 – FR-42 |
| 5 | Teo Jia Hui | `2502328` | 1. Mission Control & Fault Handling; 2. Operator Interface & Communication | FR-01 – FR-15 |

Subsystems are split by responsibility, not by board. Buddy 5 owns the two lighter, closely
linked subsystems; everyone else owns one harder one. Buddy 5 also owns the wire format,
the shared header and the fault bitmask.

## 5. Requirements

Values marked *(provisional)* come from the Week 6 assumptions (A9–A12) and will be set by
testing.

### 5.1 Functional requirements

**1. Mission Control & Fault Handling (Jia Hui)**

| ID | Requirement |
|---|---|
| FR-01 | On power-up, run Boot & Init (both Picos, UART, WiFi/MQTT, ESP32 camera stream), then a Self-Check of the ToF sensors, IMU, UART link, camera stream and MQTT link. |
| FR-02 | If init or self-check fails, enter Fault / Safe Stop and show which check failed. |
| FR-03 | Enter Ready only after self-check passes; do not start Phase 1 until N has been entered and the camera detects a starting marker. |
| FR-04 | Accept an operator stop at any time. Phase 1 stop → Ready with the partial map kept; Phase 2 stop → Go-To Ready with the map kept. |
| FR-05 | Enter Map Complete / Go-To Ready only when the distinct marker count equals N. |
| FR-06 | Enter Fault / Safe Stop from any state on emergency stop, ToF timeout, lost UART or MQTT link, camera fault in Phase 1 or 2, Phase 2 marker timeout, or Phase 1 exhausted with fewer than N markers. |
| FR-07 | In Fault / Safe Stop, stop all motion until the operator issues a fault reset, which returns to Self-Check. |
| FR-08 | The Robo Pico stops the motors on its own if no laptop message arrives within the link timeout (1.5 s with a 500 ms heartbeat, *provisional*). |

**2. Operator Interface & Communication (Jia Hui)**

| ID | Requirement |
|---|---|
| FR-09 | Dashboard: enter N, start exploration, stop, select a target marker, reset after a fault, reset the map. |
| FR-10 | Dashboard manual driving in Ready: forward/back, diagonal, strafe, arc turn, rotate on the spot, parallel park. |
| FR-11 | Dashboard shows system state, markers found out of N, latest ToF readings, heading, and the live map with explored/blocked exits and, in Phase 2, the planned route. |
| FR-12 | Laptop and Robo Pico exchange commands, telemetry and events over WiFi/MQTT on agreed topics; the Robo Pico reconnects automatically. |
| FR-13 | The Sensor Pico sends ToF and gyro data to the Robo Pico over UART at least 20 times per second (NFR-04), and sends a stop event immediately. |
| FR-14 | Report to the operator when the target is reached or no route is left. |
| FR-15 | Log sensor readings, events, state changes and commands with timestamps to a file. |

**3. Exploration & Obstacle Sensing (Vivi)**

| ID | Requirement |
|---|---|
| FR-16 | Read the front, left and right ToF continuously, completing a full three-sensor cycle at the NFR-01 rate. |
| FR-17 | Discard invalid ToF readings and raise a ToF timeout fault if a sensor stops responding. |
| FR-18 | Trigger an emergency stop, independently of the mission state, when the ToF reading in the direction of motion is within the very-close distance (20 cm, *provisional*). Very-close < far limit. |
| FR-19 | Phase 1 DFS: at a junction pick an unexplored exit (front, left or right) and drive to the next marker; at a plain marker continue straight. |
| FR-20 | When the ToF reading in the direction of travel is within the far limit (40 cm, *provisional*), stop, mark the exit blocked, return to the junction and try the next exit. |
| FR-21 | When all exits of a junction are blocked or explored, backtrack along the DFS stack to the most recent junction with unexplored exits. |
| FR-22 | Report every exploration event (marker reached, exit explored, exit blocked, backtrack) to the laptop over MQTT. |
| FR-23 | Execute a segment command (turn to a heading, drive to the next marker) and report segment done or exit blocked; the same drive routine is used in both phases. |

**4. Mapping & Path Planning (Jeslyn)**

| ID | Requirement |
|---|---|
| FR-24 | Build a marker grid map (IDs, types, grid positions, connections, exit statuses) and a 2D occupancy map from valid ToF readings, calibrated sensor mounting positions and the estimated pose; marker fixes correct the pose at junctions. |
| FR-25 | Count markers by distinct ID and notify Mission Control when the count equals N. |
| FR-26 | Detect Phase 1 exhausted: no junction has unexplored exits but fewer than N markers found. |
| FR-27 | In Go-To Ready, compute the shortest route to the target with A\* over the grid map. |
| FR-28 | Send the route one segment at a time and wait for segment done before the next. |
| FR-29 | On a blocked exit, mark it and re-plan from the current marker with no retry limit; report "no route left" if none remains. |

**5. Vision Localisation (Jeremy)**

| ID | Requirement |
|---|---|
| FR-30 | The ESP32-S3 captures frames from the USB webcam and streams them to the laptop over WiFi. |
| FR-31 | The laptop detects ArUco markers and determines each marker's ID, distance and bearing. |
| FR-32 | Classify each ID as junction or plain using a fixed ID scheme and ignore IDs outside it. |
| FR-33 | Publish each detection (ID, type, distance, bearing, timestamp) to the rover over MQTT within the NFR-05 limit. |
| FR-34 | Each time a marker is reached, set the rover's grid position to that marker's position. |
| FR-35 | Raise a marker timeout fault if no marker is detected within the marker timeout (7 s per segment, *provisional*) while driving a Phase 2 segment. |
| FR-36 | If the camera stream stops and cannot be recovered, report a camera fault to Mission Control. |

**6. Motion Control & Heading (Zi Ying)**

| ID | Requirement |
|---|---|
| FR-37 | Read the GY-6500 gyro at the NFR-02 rate, zero its offset at start-up, and integrate yaw rate into a heading. |
| FR-38 | Convert movement commands (forward, sideways, turn) into PWM and direction signals for the four mecanum wheels through the two L298N drivers, using inverse kinematics. |
| FR-39 | Manual mode supports forward/back, diagonal, strafe, arc turn, rotate on the spot and parallel park. |
| FR-40 | Autonomous mode uses only forward/back and rotate on the spot; other moves from the autonomous logic are rejected, and manual commands are accepted only in Ready. |
| FR-41 | Hold a straight course with the gyro heading and stop a rotation within the heading tolerance (±3°, *provisional*). |
| FR-42 | Ramp PWM for normal moves; cut all motor PWM immediately on emergency stop, operator stop, or when no command arrives within the command timeout (300 ms, *provisional*). |

### 5.2 Non-functional requirements

| ID | Requirement |
|---|---|
| NFR-01 | All 3 ToF sensors complete one full read cycle at ≥10 Hz. |
| NFR-02 | IMU gyro sampled at 100 Hz for heading integration. |
| NFR-03 | On detecting an obstacle within the very-close distance, halt within 200 ms. |
| NFR-04 | The Sensor Pico relays ToF readings and heading to the Robo Pico over UART at ≥20 Hz, with STOP sent immediately. |
| NFR-05 | A detected ArUco marker fix is applied to the pose estimate within 500 ms. |

None of the NFR timings has been measured on hardware yet.

## 6. Interfaces and message formats

Agreed at Tribunal 1 (Change Requirement 1: Minimum-Bit Data Representation). All layouts
are **provisional** until the shared header is frozen (before Week 10).

**Encoding rules:** masks and shifts on a byte array (no C bit-fields), little-endian, one
shared C header plus a matching Python decoder. Laptop-only topics (map update, route
result) stay JSON. Each module validates its own readings; Buddy 5 owns the packing.

| Message | Fields (bits) | Total |
|---|---|---|
| `rover/marker` (laptop → rover) | ID 6 (0–49), type 1, distance 8 (0–300 cm, 2 cm steps), bearing 7 (±45° at 1°, stored +45), sequence 8 | 30 bits in a 32-bit word |
| `rover/segment` (laptop → rover) | target marker ID 6, last-segment flag 1, target heading 9 (1°), sequence 8 | 24 bits; `SEGMENT_DONE` echoes the sequence |
| Motor command (Robo Pico internal) | type 3, source 1, sequence 4, three speeds 8 each (−100 to +100 %) or target heading 9 (1°) | 32 bits |
| UART heading | heading 12 (0.1°) + sequence 4 | 2 bytes; the gyro running total stays full precision |
| ToF distance (UART) | 10 bits in mm (0–1000) + status (valid hit / beyond range / invalid or stale) | status encoding open; 11 bits if the cap rises to 2 m |
| Junction exits event | 4 exits × 2 bits, absolute grid direction | 8 bits |
| System state | 3 bits | |
| Faults | One fault bitmask (FreeRTOS event group): each detector sets its own bit, only Mission Control clears | bit positions open |
| STOP | **Not packed**: a direct task notification to Motor Control | |

**MQTT topics.** The firmware and dashboard currently use `robot/command`, `robot/status`,
`robot/telemetry` and `robot/heartbeat`. The design adds `rover/marker`, `rover/segment`,
`system/state`, `map/update` and `route/result`. The `robot/` vs `rover/` naming will be
fixed once, together with the shared header, before Week 10.

**UART.** Sensor Pico → Robo Pico: ToF + heading frames at 20 Hz plus an immediate STOP
frame. Robo Pico → Sensor Pico: movement direction (4-bit mask + 4-bit sequence) before
moving, on every change and every 100 ms; if none arrives for 300 ms, Collision Detection
checks all sensors. UART checksum is deferred.

**Timeouts (provisional):** heartbeat 500 ms, rover link timeout 1.5 s, dashboard watchdog
2 s, motor command timeout 300 ms, heading freshness 150 ms.

**Grid convention:** start marker = (0, 0), start heading = 0°, headings 0/90/180/270; exits
are front/left/right relative to the arrival heading. Marker arrival distance 30 cm;
example ID scheme 0–19 junction, 20–49 plain (*provisional*).

## 7. Current status (2026-10-23)

Overall readiness: **Yellow**. The parts exist separately; nothing is integrated or
measured on hardware yet.

| Area | Status |
|---|---|
| ToF (Sensor Pico) | 3 ToF read together with separate addresses (`tof.c`, XSHUT GP18/19/20). `obstacle.c` and `dfs_explorer.c` added to `main` on 2026-10-23. ToF bus still at 100 kHz in `VL53L1X_platform.h` |
| IMU | GY-6500 reader sketch in `IMU-Test`. `sensor_pico` still uses the old LSM303DLHC driver (`accelero.c`); the FreeRTOS gyro task is not in the repo yet |
| Motor Control | Module, unit tests and bench firmware in `Motor-Control-Test`; merged into `integrated-rover-ii` (PR #1). Bench loop time 11–12 µs, worst case 101 µs |
| Vision | ESP32-S3 streams MJPEG; laptop ArUco detection, camera calibration and distance/bearing estimation work in `ESP32-LogiCam-Test`. Not yet publishing `rover/marker` to the rover |
| Dashboard / Mission Control | LIDAR removed and tested against `fake_rover` (`37dd313`), link watchdog added, FR-08 link timeout written for the Robo Pico but **not yet built or tested**. All on branch `JiaHui`, not merged |
| Map + A\* | Not started (`Mapping-Path-Planning` is empty) |
| Old code still present | RPLIDAR driver (`rplidar.c`), `robot/lidar` topic, LIDAR-era `README.md` in `integrated-rover-ii` |

## 8. Open items before Week 10

From the Tribunal 1 Version C gap check:

| Item | Owner | Status |
|---|---|---|
| Freeze the shared header, MQTT topic list and Python decoder | Buddy 5 (all send field tables) | Open |
| End-to-end freshness of marker messages: the laptop drops frames >200 ms after receipt, but the rover cannot yet reject delayed detections | Buddies 3, 5, with 1 | Design decision open; NFR-05 validation pending |
| Segment delivery protocol: sequence wrap, restart, retry timeout, duplicates (a repeated segment must not move the rover again) | Buddies 1, 2, 5 | Design decision open |
| ToF status meanings and map behaviour (beyond-range must not mark a cell occupied; invalid must not imply clear) | Buddies 1, 2, 5 | Design decision open |
| STOP output states: L298N enable low = coast; active brake needs enable high with equal direction inputs. Measure halt time | Buddy 4, with 1 and 5 | Open before integrated movement tests |
| R7 PWM slice 5: GP10/GP26 share channel A, GP11/GP27 channel B. Verify arm pin functions; arm commands must not change wheel PWM | Buddy 4 | Open before integrated motor testing |
| Angle conventions (reference frame, sign, zero, scaling) for marker bearing vs gyro heading; heading correction from markers deferred | Buddies 3, 4 | Open |
| Position estimation between markers (A8): method and numerical accuracy target | Buddy 2, with 3 and 4 | Open |
| Rear obstacle coverage when reversing, or an agreed movement restriction | Buddies 1, 4 | Open before reverse/manual movement |
| Measure obstacle distances, marker timeout, heading tolerance, command and link timeouts | Owners per FR | Provisional values in use |

## 9. Risks and gotchas

- **Timing after packing:** re-measure STOP-to-halt (NFR-03), marker latency (NFR-05) and
  the Motor Control loop after the packed formats go in.
- **Shared packed data:** the Cortex-M0+ has no atomic read-modify-write. One writer per
  packed word; fault bits via an event group or critical section.
- **C vs Python decoding:** never rely on C bit-field layout; round-trip test every message.
- **Pin conflicts:** check any change against the full pin map. PWM slice 5 is shared by the
  back L298N enables and the arm pins; GP23/24/25/29 are reserved on the Pico W.
- **No encoders:** turns and straight lines rely on the gyro; L298N deadband and battery sag
  change speed.
- **Missed markers** show up late as "exploration exhausted" → Fault → restart.
- **Laptop dependence:** ArUco, map and A\* run on the laptop; a link loss stops the rover.
- **Credentials in git history:** rotate the Wi-Fi password and remove it from the repos.
- **Verification wording:** "working" means compiled, flashed and tested on hardware.

## 10. Build order

1. Freeze the shared header, topic list and decoder; update `sensor_config.h` (GY-6500 on
   I2C1) and raise the ToF bus to 400 kHz.
2. Port the GY-6500 to a FreeRTOS IMU task; add packed ToF + heading and the immediate STOP
   frame to the UART link.
3. Merge the `JiaHui` branch; build and test the FR-08 link timeout.
4. Build the safe skeleton on the Robo Pico and laptop (Boot, Self-Check, Ready, Fault,
   operator and emergency stop); remove the LIDAR code.
5. Connect vision: ESP32 stream → laptop ArUco → `rover/marker` → Robo Pico.
6. Integrate DFS exploration with the laptop map; test DFS and A\* on recorded runs first.
7. Build Phase 2: A\*, segment sending with the delivery protocol, reroute.
8. Finish the dashboard (N entry, controls, manual pad, map and route display).

## 11. Sources

- Week 6 Design Review, "updated proposal" tab (requirements, tasks, pin map, assumptions).
- Tribunal 1 report `P1-SE10_PostTri1` (Versions A–C: message formats, risks, open items).
- Repository contents on 2026-10-23.
