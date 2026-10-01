# INF2004 ESP – Rover II: Project Overview

*Autonomous Marker-Grid Exploration Rover Upgrade*
*Last updated: 2026-10-01 (aligned with system statechart v9, motor statechart v4, task interaction diagram v2 and the Week 6 Design Review FR-01 to FR-42)*

---

## 1. What This Project Is

Rover II is the upgrade of an existing mecanum-wheel rover, separate from
INF2004's default graded module project (the line-following car). The base
rover already drives manually with no orientation sensing. This project adds
two autonomous phases that run on a **grid of ArUco markers**:

- **Phase 1 – Explore & Map:** the rover explores the grid on its own by
  **depth-first search (DFS)**, using 3 ToF distance sensors (front / left /
  right) and the GY-6500 gyro for heading, replacing the current LIDAR + claw
  setup. Every marker's ID says whether it is a **junction** or a **plain
  (non-junction)** marker. At each junction the rover picks an unexplored exit;
  a blocked exit is marked and the next one tried; when every exit is blocked or
  explored it **backtracks along its DFS stack** to the most recent junction that
  still has unexplored exits. The laptop builds the grid map from the rover's
  events. Phase 1 ends **only when all N distinct markers have been found**.
- **Phase 2 – Go to Marker (manual target):** once the map is complete, the
  operator picks a target marker on the dashboard. The **laptop runs A\*** over
  the map and sends the route **one segment at a time** (turn to a heading,
  drive to the next marker). If an exit on the route is blocked it marks it and
  re-plans, with **no retry limit**; only "no route left" ends the attempt.
- **Manual driving:** in READY the operator can drive the rover from the
  dashboard, including the mecanum moves (strafe, diagonal, arc turn, rotate on
  the spot, parallel park).

**Use of the mecanum drivetrain (decided 2026-09-30/10-01).** Autonomous driving
uses **forward/back and rotate on the spot only**, which is the most reliable
option without wheel encoders. Strafe, diagonal, arc turn and parallel park are
**manual-mode only** for now. This replaces the older note that continuous
mecanum motion was a "big requirement" for exploration.

The rover is told only **how many markers exist (N)** and **what the two
marker types are**. It is **not** given the grid layout.

## 2. System Architecture (target design)

Two Raspberry Pi Pico boards (the Robo Pico is a **Pico W**) linked by UART,
plus an ESP32-S3 camera and a laptop.

| Part | Details | Role |
|---|---|---|
| **Sensor Pico** | Pico, FreeRTOS | Reads 3 ToF + IMU, runs Collision Detection (emergency stop), sends data and immediate STOP events to the Robo Pico over UART |
| **Robo Pico** | **Pico W**, FreeRTOS | WiFi/MQTT client, UART receive, **Exploration Task** (DFS), **Motor Control Task** |
| **3x VL53L1X ToF** | LEFT `0x30`, FRONT `0x31`, RIGHT `0x32`, XSHUT GP18/19/20 | Exit checks at junctions, far limit (exit blocked), very-close distance (emergency stop) |
| **GY-6500 IMU** | MPU6500: gyro + accelerometer, **no magnetometer**, I2C `0x68` | Heading from the integrated gyro, zeroed at start-up |
| **ESP32-S3 + Logitech webcam** | USB-host UVC, MJPEG stream over WiFi | Streams frames to the laptop (**no detection on the ESP32**) |
| **Drive** | **2x L298N** on a **12 V battery (motors only)**, **4 DC motors with mecanum wheels**; Picos on a separate charged battery (common ground needed) | Omni-directional motion |
| **Laptop** | Python + Mosquitto MQTT broker | Dashboard + mission controller, ArUco detection (OpenCV), grid map + A\*, logging |
| **Wheel encoders** | **None** | Distance is open-loop; position is corrected at each marker |

**Where each piece of software runs**

| Location | Software (owner) |
|---|---|
| Sensor Pico (FreeRTOS) | ToF Sensing Task (Vivi), Collision Detection Task – highest priority (Vivi), IMU Task (Zi Ying), Sensor Comms Task (Jia Hui) |
| Robo Pico W (FreeRTOS) | Robo Comms Task: UART RX + MQTT (Jia Hui), Exploration Task: DFS + drive to next marker (Vivi), Motor Control Task (Zi Ying) |
| ESP32-S3 | Camera Stream Task (Jeremy) |
| Laptop (Python) | Mission Controller + Dashboard + Logger (Jia Hui), Map Builder + A\* Planner (Jeslyn), ArUco Detector (Jeremy), Mosquitto broker |

**Existing code (what is in the repos today):**

- **`firmware/robo_pico/`** — motor control (mecanum drive), WiFi/MQTT, and the
  old LIDAR-based reactive wander `AUTO_*` state machine (to be replaced by
  the statechart in §6).
- **`firmware/sensor_pico/`** — IMU driver for the **LSM303DLHC** (to be replaced
  by the GY-6500), 3x VL53L1X ToF (`tof.c`), HC-SR04 ultrasonic drivers,
  RPLIDAR A1 code (hardware removed; code still to be deleted), SD card logging. Sends a packed struct over UART.
- **`ESP32-LogiCam-Test`** — ESP32-S3 USB-host UVC firmware serving `/stream`
  (MJPEG) and `/capture`, plus laptop ArUco scripts (`software/*.py`, OpenCV).
- **`IMU-Test`** — `gy6500_reader` sketch (MPU6500 at `0x68`, ±500 dps, ±4 g).
- **`3TOF-Test`** — standalone 3-ToF bring-up and `integration_test.c`.
- **`dashboard/robot_dashboard.py`** — Tkinter + MQTT, still LIDAR-oriented.

### 2.1 Sensor roles

| Sensor | Role |
|---|---|
| ToF FRONT / LEFT / RIGHT | The three candidate exits at a junction; obstacle detection while driving (far limit → exit blocked / reroute, very close → emergency stop) |
| GY-6500 gyro | Heading for rotate-to-heading, heading hold while driving, and grid direction |
| ESP32 camera + laptop ArUco | Marker ID (junction / plain), distinct-ID count, distance and bearing, position fix at each marker |
| Ultrasonic x2 (front GP26/27, back GP0/1) | Existing close-range sensing, wiring unchanged. **Not used by any FR yet** (role to be confirmed) |

### 2.2 Pin map (confirmed 2026-10-01)

| Board | Device | Signal | GPIO | Interface | Owner task |
|---|---|---|---|---|---|
| Sensor Pico | 3 x VL53L1X ToF | SDA / SCL | GP4 / GP5 | I2C0, 400 kHz | ToF Sensing Task |
| Sensor Pico | ToF LEFT | XSHUT | GP18 | GPIO out (address 0x30) | ToF Sensing Task |
| Sensor Pico | ToF FRONT | XSHUT | GP19 | GPIO out (address 0x31) | ToF Sensing Task |
| Sensor Pico | ToF RIGHT | XSHUT | GP20 | GPIO out (address 0x32) | ToF Sensing Task |
| Sensor Pico | GY-6500 IMU | SDA / SCL | GP6 / GP7 | I2C1 (address 0x68) | IMU Task |
| Sensor Pico | Ultrasonic FRONT (HC-SR04) | TRIG / ECHO | GP26 / GP27 | GPIO out / in | Existing, not used by current FRs |
| Sensor Pico | Ultrasonic BACK (HC-SR04) | TRIG / ECHO | GP0 / GP1 | GPIO out / in | Existing, not used by current FRs |
| Sensor Pico → Robo Pico | UART link | TX → RX | GP16 → GP13 | UART | Sensor Comms Task / Robo Comms Task |
| Robo Pico → Sensor Pico | UART return line | TX → RX | GP12 → GP17 | UART | Wired, not used (one-way link) |
| Sensor Pico | SD card (logging) | SPI1 (firmware: SCK GP10, MISO GP11, MOSI GP12, CS GP15) | GP10 – GP15 | SPI1 | Existing, kept for now (removal not decided) |
| Robo Pico | Front L298N | ENA / ENB (speed) | GP4 / GP5 | PWM, 10 kHz | Motor Control Task |
| Robo Pico | Front L298N | IN1 / IN2 / IN3 / IN4 (direction) | GP3 / GP2 / GP16 / GP17 | GPIO out | Motor Control Task |
| Robo Pico | Back L298N | ENA / ENB (speed) | GP27 / GP26 | PWM, 10 kHz | Motor Control Task |
| Robo Pico | Back L298N | IN1 / IN2 / IN3 / IN4 (direction) | GP28 / GP7 / GP0 / GP1 | GPIO out | Motor Control Task |
| Robo Pico | Arm motor | M1A (down) / M1B (up) | GP11 / GP10 | PWM / GPIO | Existing, not used by current FRs |
| Robo Pico | Claw motor | M1A (close) / M1B (open) | GP8 / GP9 | PWM / GPIO | Existing, not used by current FRs |
| Robo Pico | WiFi | – | onboard CYW43 | WiFi / MQTT | Robo Comms Task |
| ESP32-S3 | Logitech webcam | USB D− / D+, 5 V | USB-OTG port | USB host (UVC, MJPEG) | Camera Stream Task |
| ESP32-S3 | Laptop link | – | onboard WiFi | WiFi (MJPEG stream) | Camera Stream Task |

Power: the 12 V battery powers only the two L298N drivers; a separate battery powers the Picos; grounds connected.

Changes from the old `sensor_config.h`: the IMU moved to its own bus on GP6 / GP7
(previously the left ultrasonic), and the left and right ultrasonic sensors are
no longer used, and the RPLIDAR (UART1 GP8/GP9 and its VBUS power) is removed. Update `sensor_config.h` to match. The "GP1" in its UART comment
is outdated (Robo Pico RX is GP13).

## 3. Team and Workload Split

Subsystems are split **by responsibility**, then assigned so that each member
carries a similar difficulty: Jia Hui takes two lighter, closely linked
subsystems; everyone else takes one harder one.

| # | Subsystem | Owner | FRs (count) | Runs on | Laptop? | Difficulty |
|---|---|---|---|---|---|---|
| 1 | **Mission Control & Fault Handling** | Teo Jia Hui | FR-01 to FR-08 (8) | Laptop (mission state machine in Python) + Robo Pico (rover-side failsafe) | Yes | Medium |
| 2 | **Operator Interface & Communication** | Teo Jia Hui | FR-09 to FR-15 (7) | Laptop (dashboard, MQTT broker, logger) + Robo Pico (MQTT client, UART receive) + Sensor Pico (UART send) | Yes | Easy-Medium |
| 3 | **Exploration & Obstacle Sensing** | Genevivi Mia Er (Yu Ruien) | FR-16 to FR-23 (8) | Rover: Sensor Pico (ToF Sensing and Collision Detection Tasks) + Robo Pico (Exploration Task) | No | Hard |
| 4 | **Mapping & Path Planning** | Lee Ru Yan Jeslyn | FR-24 to FR-29 (6) | Laptop (Python) | Yes | Medium-Hard |
| 5 | **Vision Localisation** | Lee Zong Yang Jeremy | FR-30 to FR-36 (7) | ESP32-S3 (camera streaming) + Laptop (ArUco detection in Python/OpenCV) | Yes | Hard |
| 6 | **Motion Control & Heading** | Lim Zi Ying | FR-37 to FR-42 (6) | Rover: Robo Pico (Motor Control Task) + Sensor Pico (IMU Task) | No | Hard |

**What each subsystem does**

- **1. Mission Control & Fault Handling** — Decides what mode the whole system is in (Boot, Self-Check, Ready, Phase 1 Explore, Map Complete / Go-To Ready, Phase 2 Go to Marker, Fault / Safe Stop), handles operator stop, and sends the rover to a safe state when anything goes wrong. It follows the Section 5 system statechart.
- **2. Operator Interface & Communication** — Everything that moves information between the operator and the boards: the dashboard (inputs and displays), the MQTT link between laptop and rover, the UART link between the two Picos, result reporting and logging.
- **3. Exploration & Obstacle Sensing** — The rover-side exploration loop: read the ToF sensors, stop for anything too close, decide which exit to take at each junction (depth-first search), mark blocked exits, backtrack when a junction is used up, and drive marker to marker. Reports every event to the laptop so the map can be built.
- **4. Mapping & Path Planning** — The logical grid map of the room, built from the rover's events, and shortest-path planning over it. Counts distinct markers, detects when exploration is exhausted, plans Phase 2 routes with A*, sends them one segment at a time, and re-plans around blocked exits.
- **5. Vision Localisation** — Seeing the markers: stream the camera to the laptop, detect ArUco markers, work out each marker's ID, type (junction or plain), distance and bearing, and send that to the rover. Also corrects position drift at each marker and raises the Phase 2 marker timeout and camera faults.
- **6. Motion Control & Heading** — Turning movement commands into wheel motion: mecanum inverse kinematics, PWM through the L298N drivers, gyro heading tracking and heading hold, manual and autonomous move sets, ramping, and cutting power instantly on any stop.

**Hardware per subsystem**

| Subsystem | Hardware and role |
|---|---|
| 1. Mission Control & Fault Handling | Laptop: runs the mission state machine and fault logic<br>Robo Pico (Pico W): rover-side state, stops motors itself if the laptop link is lost<br>Sensor Pico: reports self-check results and sensor faults<br>UART link and WiFi/MQTT link: carry state, faults and stop events |
| 2. Operator Interface & Communication | Laptop: Tkinter dashboard, Mosquitto MQTT broker, log files<br>Robo Pico (Pico W): WiFi/MQTT client, UART receiver<br>Sensor Pico: UART sender (Sensor Comms Task), GP16 TX to Robo Pico GP13 RX<br>WiFi network: links laptop, Robo Pico and ESP32 |
| 3. Exploration & Obstacle Sensing | 3 x VL53L1X ToF (front, left, right) on the Sensor Pico I2C bus GP4/GP5, XSHUT on GP18/GP19/GP20 to give each sensor its own address<br>Sensor Pico: ToF Sensing Task and Collision Detection Task (emergency stop)<br>Robo Pico (Pico W): Exploration Task (DFS and the shared drive-to-next-marker routine) |
| 4. Mapping & Path Planning | Laptop: grid map data structure, A* planner, route sender<br>No rover hardware of its own: it uses events from Exploration (Subsystem 3) and marker data from Vision (Subsystem 5) |
| 5. Vision Localisation | Logitech webcam on the ESP32-S3 USB-OTG port (UVC, MJPEG, powered from the ESP32 USB 5 V)<br>ESP32-S3: captures frames and streams them over WiFi<br>Laptop: OpenCV ArUco detection, publishes detections over MQTT |
| 6. Motion Control & Heading | GY-6500 IMU (MPU6500, gyro + accelerometer, no magnetometer) on the Sensor Pico I2C GP6/GP7: heading from the gyro<br>Robo Pico (Pico W): Motor Control Task, PWM and direction pins<br>2 x L298N motor drivers powered by the 12 V battery (motors only)<br>4 DC motors with mecanum wheels<br>Picos powered from a separate battery; grounds must be common with the L298N logic |

**Why these boundaries**

- **Exploration + Obstacle Sensing together (rover):** the ToF checks and the
  exit decisions form one real-time loop, so one person owns it.
- **Mapping + Path Planning together (laptop):** A\* works directly on the grid
  map, so they share one data structure.
- **Mission Control + Operator Interface together:** both are the
  state machine and the links that every other subsystem plugs into.

**Week 6 document mapping**

| Doc section | Content | Owner |
|---|---|---|
| §5 System statechart | v9 (Mission Control) | Jia Hui |
| §6.1 | Mapping & Path Planning (replaces "Navigation Controller") | Jeslyn |
| §6.2 | Exploration & Obstacle Sensing (extend the ToF chart) | Vivi |
| §6.3 | Vision Localisation (update camera chart: detection moves to laptop) | Jeremy |
| §6.4 | Motion Control & Heading (v4, done) | Zi Ying |
| §6.5 | Operator Interface & Communication | Jia Hui |
| §7 | At least one assumption per buddy | Everyone |
| §8.1 | Each buddy's honest AI-usage rows | Everyone |

Each member has a briefing file (`Rover_II_briefing_<name>.md`) with the full
context for their AI sessions.

## 4. Requirements

### 4.1 Functional requirements (42)

### 1. Mission Control & Fault Handling — Teo Jia Hui (FR-01 to FR-08)

| ID | Requirement |
|---|---|
| FR-01 | On power-up, the system shall run Boot & Init (both Picos, UART, WiFi/MQTT, ESP32 camera stream) and then a Self-Check of the ToF sensors, the IMU, the UART link, the camera stream and the MQTT link. |
| FR-02 | If init or self-check fails, the system shall enter Fault / Safe Stop and show the operator which check failed. |
| FR-03 | The system shall enter Ready only after self-check passes, and shall not start Phase 1 until the operator has entered N and the camera detects a starting marker. |
| FR-04 | The system shall accept an operator stop at any time. A stop in Phase 1 shall return to Ready with the partial map kept so exploration can resume; a stop in Phase 2 shall return to Go-To Ready with the map kept. |
| FR-05 | The system shall enter Map Complete / Go-To Ready only when the distinct marker count equals N. |
| FR-06 | The system shall enter Fault / Safe Stop from any state on: emergency stop, ToF sensor timeout, lost UART or MQTT link, camera fault during Phase 1 or 2, Phase 2 marker timeout, or Phase 1 exploration exhausted with fewer than N markers found. |
| FR-07 | In Fault / Safe Stop, the rover shall stop all motion and stay stopped until the operator issues a fault reset, which returns the system to Self-Check. |
| FR-08 | The Robo Pico shall stop the motors on its own if no message from the laptop arrives within the link timeout (value set during testing, see Section 7). |

### 2. Operator Interface & Communication — Teo Jia Hui (FR-09 to FR-15)

| ID | Requirement |
|---|---|
| FR-09 | The dashboard shall let the operator enter N, start exploration, stop, select a target marker, reset after a fault, and reset the map (clear it and return to Ready). |
| FR-10 | The dashboard shall let the operator drive the rover manually (forward/back, diagonal, strafe, arc turn, rotate on the spot, parallel park) while the system is in Ready. |
| FR-11 | The dashboard shall display the system state, markers found out of N, the latest ToF readings, the heading, and the live map with explored and blocked exits and, in Phase 2, the planned route. |
| FR-12 | The laptop and the Robo Pico shall exchange commands, telemetry and events over WiFi/MQTT using agreed topics, and the Robo Pico shall reconnect automatically if the link drops. |
| FR-13 | The Sensor Pico shall send ToF readings and gyro data to the Robo Pico over UART at least 20 times per second (NFR-04), and shall send a stop event immediately rather than waiting for the next periodic frame. |
| FR-14 | The system shall report to the operator when the target marker is reached or when no route is left. |
| FR-15 | The system shall log sensor readings, events, state changes and commands with timestamps to a file for later review. |

### 3. Exploration & Obstacle Sensing — Genevivi Mia Er (Yu Ruien) (FR-16 to FR-23)

| ID | Requirement |
|---|---|
| FR-16 | The Sensor Pico shall read the front, left and right ToF sensors continuously while the system is running, completing a full three-sensor cycle at the NFR-01 rate. |
| FR-17 | The Sensor Pico shall discard invalid ToF readings (out of range or sensor error) and raise a ToF sensor timeout fault if a sensor stops responding. |
| FR-18 | The Sensor Pico shall trigger an emergency stop, independently of the mission state, when the ToF reading in the direction of motion falls within the very-close distance (value set during testing, see Section 7). The very-close distance shall be smaller than the far limit. |
| FR-19 | In Phase 1, the Robo Pico shall explore by depth-first search: at a junction marker it shall pick an unexplored exit (front, left or right) and drive to the next marker; at a plain marker it shall continue in the same direction. |
| FR-20 | When the ToF reading in the direction of travel falls within the far limit (value set during testing, see Section 7), the rover shall stop, mark that exit blocked, return to the junction and try the next unexplored exit. |
| FR-21 | When all exits of a junction are blocked or explored, the rover shall backtrack along its DFS stack to the most recent junction that still has unexplored exits. |
| FR-22 | The rover shall report every exploration event (marker reached, exit explored, exit blocked, backtrack) to the laptop over MQTT. |
| FR-23 | The rover shall execute a segment command from the laptop (turn to a heading, then drive to the next marker) and report segment done or exit blocked; the same drive routine is used in Phase 1 and Phase 2. |

### 4. Mapping & Path Planning — Lee Ru Yan Jeslyn (FR-24 to FR-29)

| ID | Requirement |
|---|---|
| FR-24 | The laptop shall build and store a grid map from the rover's events: each marker's ID, type and grid position, the connections between markers, and the status of each exit (unexplored, explored, blocked). |
| FR-25 | The laptop shall count markers by distinct ID, so revisited markers are not counted twice, and notify Mission Control when the count equals N. |
| FR-26 | The laptop shall detect when Phase 1 is exhausted: no junction has unexplored exits but fewer than N markers have been found. |
| FR-27 | In Go-To Ready, the laptop shall compute the shortest route from the current marker to the operator's target marker using A* over the grid map. |
| FR-28 | The laptop shall send the route to the rover one segment at a time and wait for segment done before sending the next segment. |
| FR-29 | If the rover reports an exit on the route blocked, the laptop shall mark it blocked and re-plan with A* from the current marker, with no retry limit, and report "no route left" if no route remains. |

### 5. Vision Localisation — Lee Zong Yang Jeremy (FR-30 to FR-36)

| ID | Requirement |
|---|---|
| FR-30 | The ESP32-S3 shall capture frames from the USB webcam and stream them to the laptop over WiFi. |
| FR-31 | The laptop shall detect ArUco markers in the stream and, for each marker, determine its ID and its distance and bearing from the rover. |
| FR-32 | The laptop shall classify each marker ID as junction or plain using a fixed ID scheme (listed in Section 7), and ignore IDs outside the scheme. |
| FR-33 | The laptop shall publish each marker detection (ID, type, distance, bearing, timestamp) to the rover over MQTT within the NFR-05 time limit. |
| FR-34 | Each time a marker is reached, the system shall set the rover's grid position to that marker's position, to correct drift. |
| FR-35 | If no marker is detected within the marker timeout (value set during testing, see Section 7) while driving a route segment in Phase 2, the system shall raise a marker timeout fault. |
| FR-36 | If the camera stream stops and cannot be recovered, the system shall report a camera fault to Mission Control. |

### 6. Motion Control & Heading — Lim Zi Ying (FR-37 to FR-42)

| ID | Requirement |
|---|---|
| FR-37 | The Sensor Pico shall read the GY-6500 gyro at the NFR-02 rate, zero its offset while the rover is still at start-up, and integrate the yaw rate into a heading. |
| FR-38 | The Robo Pico shall convert movement commands (forward speed, sideways speed, turn rate) into PWM and direction signals for the four mecanum wheels through the two L298N drivers, using inverse kinematics. |
| FR-39 | In manual mode, the rover shall support forward/back, diagonal, strafe left/right, arc turn, rotate on the spot and parallel park. |
| FR-40 | In autonomous mode, the rover shall use only forward/back and rotate on the spot; strafe, diagonal, arc turn and park commands from the autonomous logic shall be rejected, and manual commands shall be accepted only in Ready. |
| FR-41 | The rover shall use the gyro heading to hold a straight course while driving, and to stop a rotation within the heading tolerance (value set during testing, see Section 7) of the requested heading. |
| FR-42 | The rover shall ramp PWM up and down for normal moves, and cut all motor PWM immediately on an emergency stop or operator stop, or when no movement command arrives within the command timeout (value set during testing, see Section 7). |

### 4.2 Non-functional requirements

| ID | Requirement |
|---|---|
| NFR-01 | Each ToF sensor shall be sampled such that all 3 sensors complete one full read cycle at ≥10Hz |
| NFR-02 | The IMU shall be sampled at 100Hz to support the complementary-filter tilt/heading estimate |
| NFR-03 | On detecting an obstacle within minimum safe stopping distance, the rover shall halt within 200ms |
| NFR-04 | The Sensor Pico shall relay combined ToF + IMU + pose data to the Robo Pico over UART at ≥20 Hz |
| NFR-05 | Once an ArUco marker position fix is detected, it shall be applied to the pose estimate within 500 ms. |

Placeholder values (far limit, very-close distance, marker timeout, command
timeout, link timeout, heading tolerance) are set during testing and listed in
§7 of the Week 6 document.

## 5. Settled Decisions

| Decision | Detail |
|---|---|
| **LIDAR removal** | The RPLIDAR A1 has been removed entirely (weight + no longer needed for the ToF-based approach); its firmware code still needs deleting. |
| **Replacement sensor** | 3x VL53L1X ToF (LEFT / FRONT / RIGHT). |
| **Sensor addresses** | LEFT `0x30`, FRONT `0x31`, RIGHT `0x32` (default `0x29`, reassigned one at a time via XSHUT). |
| **XSHUT pins** | **GP18 (LEFT), GP19 (FRONT), GP20 (RIGHT)** — moved from GP6/7/8 because GP6/GP7 are the left ultrasonic pins on the real board. |
| **I2C bus speed** | ToFs need 400 kHz. The IMU is now on its own bus (GP6/GP7), so the old shared-bus conflict goes away once `sensor_config.h` is updated; `VL53L1X_platform.h` still sets `VL53L1X_I2C_BAUDRATE` to 100000 and must be raised. |
| **IMU** | **GY-6500 (MPU6500)** on the Sensor Pico. Gyro only for heading (no magnetometer); zeroed at start-up; heading sent to the Robo Pico over UART. |
| **Encoders** | None. Distances are open-loop; markers give the position fix. |
| **Drive** | 4 mecanum wheels, 2x L298N on a 12 V motor-only battery; Picos on a separate battery. |
| **Mapping approach** | 2D marker grid (the earlier 3D / vertical-scan idea is superseded; confirm dropped, §9). |
| **ArUco detection** | Runs **on the laptop**; the ESP32-S3 only streams video. |
| **Exploration** | DFS on the Robo Pico; backtracking along the DFS stack (no A\* in Phase 1). |
| **Map and A\*** | On the laptop. Phase 2 routes are sent one segment at a time. |
| **Autonomous motion** | Forward/back and rotate on the spot only; mecanum moves are manual-only. Manual driving only in READY. |
| **Marker count** | Distinct IDs only. |
| **Start condition** | The rover starts on a marker; Phase 1 starts only after N is entered and a start marker is seen. |
| **Operator stop** | Any time. Phase 1 → READY **with the partial map kept** (exploration can resume). Phase 2 → GO-TO READY (map kept). "Reset map" clears it. |
| **Obstacle thresholds** | **Far limit** (exit blocked / reroute) and **very-close distance** (emergency stop, independent of the state machine); very-close < far limit. Values to be measured. |
| **Emergency stop path** | Collision Detection (Sensor Pico) → immediate UART STOP frame (not queued behind data) → Motor Control cuts PWM at once (NFR-03, 200 ms). |
| **Marker timeout** | Phase 2 only. |
| **Faults** | Init fail, self-check fail, emergency stop, ToF/camera/link fault, Phase 2 marker timeout, Phase 1 exploration exhausted (fewer than N found). Recovery is an operator reset back to SELF-CHECK. The Robo Pico also stops by itself if the laptop link goes silent. |
| **Missed marker** | Cannot be detected when passed; appears as "exploration exhausted (fewer than N found)" → Fault → operator restarts. |
| **Org name** | Jeremy created "INF2004 ESP - Rover II" as the organisation for this work. |

## 6. Statecharts and Diagrams

### 6.1 System statechart (v9)

States: BOOT AND INIT (both Picos, UART, WiFi/MQTT, camera stream, gyro zeroed)
→ SELF-CHECK (ToF, IMU, UART, camera, MQTT) → READY (manual driving allowed,
N set, partial map kept) → **PHASE 1: EXPLORE AND MAP** (DFS on rover; AT
JUNCTION, DRIVE TO NEXT MARKER, PLAIN MARKER, EXIT BLOCKED, BACKTRACK) → MAP
COMPLETE / GO-TO READY → **PHASE 2: GO TO MARKER** (A\* on laptop; PATH
PLANNING, EXECUTE SEGMENT, AT MARKER, REROUTE) → back to GO-TO READY. FAULT /
SAFE STOP is reachable from any state.

| From | To | Trigger |
|---|---|---|
| Boot and Init | Self-check | init complete |
| Self-check | Ready | pass |
| Ready | Phase 1 (At Junction) | start / resume explore `[N set, start marker seen]` |
| At Junction | Drive to Next Marker | exit chosen |
| Drive to Next Marker | At Junction | junction marker reached |
| Drive to Next Marker | Plain Marker → Drive | plain marker reached (count new ID, continue) |
| Drive to Next Marker | Exit Blocked | obstacle within far limit |
| Exit Blocked | At Junction | back at junction |
| At Junction | Backtrack | all exits blocked / explored |
| Backtrack | At Junction | junction reached |
| Phase 1 | Map Complete / Go-To Ready | all N distinct markers found |
| Map Complete / Go-To Ready | Path Planning | target marker set |
| Path Planning | Execute Segment | segment sent |
| Execute Segment | At Marker | marker reached |
| At Marker | Execute Segment | next segment |
| Execute Segment | Reroute | exit blocked |
| Reroute | Path Planning | re-plan |
| Path Planning | Go-To Ready | no route left |
| At Marker | Go-To Ready | target reached (result reported) |
| Phase 1 | Ready | operator stop (map kept) |
| Phase 2 | Go-To Ready | operator stop |
| Go-To Ready | Ready | reset map (operator) |
| Boot / Self-check | Fault | init fail / check fail |
| Phase 1 | Fault | exploration exhausted (fewer than N) / emergency stop / ToF, camera or link fault |
| Phase 2 | Fault | marker timeout / emergency stop / ToF, camera or link fault |
| Fault | Self-check | operator reset |

### 6.2 Motor statechart (v4, Motion Control & Heading)

ZERO GYRO (Sensor Pico IMU Task) → IDLE → VALIDATE COMMAND (manual only in
READY; auto only forward/back, rotate, stop) → MANUAL MODE (forward/back,
diagonal, strafe, arc turn, rotate on spot, parallel park; switch without
stopping) or AUTO MODE (drive forward/back with heading hold, rotate to
heading) → RAMP DOWN → IDLE. Emergency stop / command timeout / link lost →
SAFE STOP (PWM 0 at once) → fault cleared + operator reset → IDLE. Operator
stop → IDLE with PWM 0 at once.

### 6.3 Task interaction diagram (v2)

ToF Sensing → ToF queue → Collision Detection and Sensor Comms; IMU Task →
heading → Sensor Comms; Collision Detection → **immediate STOP** → Sensor Comms
→ UART (20 Hz frames + STOP) → Robo Comms → STOP override → Motor Control.
Robo Comms → ToF + heading → Exploration Task → move commands → Motor Control
→ 2x L298N → 4 motors. Robo Comms ⇄ MQTT broker ⇄ Mission Controller /
Dashboard and Map + A\*. ESP32 Camera Stream → MJPEG over WiFi → laptop ArUco
Detector → marker detections → MQTT → rover.

**Diagram files** (add to the repo and link here): `system_statechart_v9.png`,
`motor_statechart_v4.png`, `task_interaction_v2.png`, with Lucid / Mermaid code
in `*_lucid.txt` and PlantUML in `*.puml`.

## 7. Interfaces (proposal, owned by Jia Hui – agree before coding)

**MQTT topics:** `rover/cmd` (laptop → rover: START_EXPLORE, STOP, RESET_FAULT,
RESET_MAP, MANUAL moves), `rover/segment` (laptop → rover: from, to,
heading), `rover/marker` (laptop → rover: id, type, distance, bearing,
timestamp), `rover/event` (rover → laptop: MARKER_REACHED, EXIT_EXPLORED,
EXIT_BLOCKED, BACKTRACK, SEGMENT_DONE, FAULT), `rover/telemetry` (rover →
laptop, 5–10 Hz), `system/state` (retained mission state), `system/heartbeat`
(both ways, 2 Hz), `camera/status`, `map/update`, `route/result`.

**UART frame (Sensor Pico → Robo Pico):** start byte `0xAA`, type (DATA 20 Hz /
STOP immediate / FAULT), length, payload (ToF L/F/R mm + validity bits,
heading, yaw rate), sequence number, checksum.

**Grid convention:** start marker = (0, 0), start heading = 0°, headings
0/90/180/270; exits front/left/right relative to the arrival heading.

Full payload definitions are in the member briefing files.

## 8. Current Status

| Item | Status |
|---|---|
| Standalone 3-ToF bring-up test (`3TOF-Test`, `read_3_tofs.c`) | **Working**, confirmed by Jeremy. Reference for the init/read pattern. |
| ToF in `sensor_pico` (`integrated-rover-ii`) | **Pushed** 2026-09-27 (`tof.c` XSHUT sequencing and addresses; `main.c` prints readings). **Not yet in the UART packet.** Hardware verification not recorded. `VL53L1X_platform.h` still at 100 kHz. |
| `integration_test.c` (`3TOF-Test/integration_test/`) | Written, not yet compiled or flashed. |
| IMU | **GY-6500 chosen.** `IMU-Test` has a reader sketch; `sensor_pico` still contains the LSM303DLHC driver. Gyro zeroing / heading integration in FreeRTOS not started. |
| ArUco detection | Laptop scripts exist and the ESP32 streams video. **Not connected to the rover**; ID scheme, distinct-ID counting and MQTT publishing not started. |
| Rover state machine | `robo_pico` still has the LIDAR `AUTO_*` wander. Mission states, faults, operator stop, Exploration Task and segment driving **not implemented**. |
| Motor control | Mecanum drive exists in `robo_pico`. Gyro heading hold, rotate-to-heading, command gating and command timeout not implemented. |
| Map + A\* (laptop) | Not started. |
| Dashboard | Still LIDAR-oriented. Needs N entry, start/stop/reset, target selection, manual pad, state and map display. |
| Design documents | Week 6 doc updated with FR-01 to FR-42, subsystem split, system statechart v9, motor statechart v4, task interaction v2, updated task table. §6.1, §6.2, §6.3 (camera chart) and §6.5 still need updating; pin table updated with all confirmed pins (§2.2). |

## 9. Open Questions / Decisions Still Needed

- Grid size, marker spacing and N.
- ArUco dictionary, marker size and placement, and the junction / plain ID scheme.
- Values: far limit, very-close distance, marker arrival distance, marker timeout, command timeout, link timeout, heading tolerance.
- Message contracts (§7): agree before coding.
- After a Fault reset, is the partial map kept?
- Phase 2: do blocked-exit marks expire, in case an obstacle was temporary?
- Which battery powers the ESP32-S3 + webcam.
- Vertical / 3D scanning: confirm it is dropped.
- Whether the claw is being removed.
- Whether Rover II must follow the INF2004 module's Week 4 process rules (public repo, pin-map sign-off, AI-usage documentation, Week 8 tribunal recording). Clarify with the instructor if unsure.

## 10. Risks / Gotchas to Track

- **I2C bus speed ordering** — easy to reintroduce silently if init order changes.
- **Pin conflicts** — check any reassignment against the full pin map (motors, ultrasonic, IMU, UART, SD SPI). GP23/24/25/29 are reserved on the Pico **W** for the wireless chip; drive the LED via `cyw43_arch_gpio_put()`.
- **Credential exposure**: `PINOUT.md` in the Senior Project repo has a plaintext WiFi SSID/password and broker IP. Rotate and remove before anything goes public.
- **Loop timing** on the Sensor Pico (ultrasonic reads + 3 ToF) — measure once integrated; NFR-01 and NFR-04 depend on it.
- **No encoders** — turns and straight lines rely on gyro heading; L298N deadband and battery sag change speed.
- **Heading at 20 Hz on the Robo Pico** — rotations can overshoot; slow down near the target.
- **Missed markers are found late** (exploration exhausted → Fault). Document in the report.
- **Marker counting** — a misread ID could end Phase 1 early; consider requiring a confirmed read.
- **Laptop dependence** — ArUco, map and A\* all run on the laptop; a link loss stops the rover (Fault).
- **ESP32-S3 USB-host streaming** at full speed may limit frame rate.
- **LIDAR still in the firmware** — remove it together with the `AUTO_*` wander.
- **Don't overstate verification status**: "working" means compiled, flashed and tested on hardware.

## 11. Suggested Build Order

1. Agree the message contracts (§7) and update `sensor_config.h` to the confirmed pin map (§2.2).
2. Verify ToF on hardware and add ToF + heading to the UART packet with an immediate STOP frame.
3. Port the GY-6500 to a FreeRTOS IMU Task (zeroing, heading); test each wheel and the kinematics.
4. Build the safe skeleton: Boot, Self-check, Ready, Fault, operator stop and emergency stop on `robo_pico` and the laptop mission controller; remove the `AUTO_*` wander.
5. Connect vision: ESP32 stream → laptop ArUco → MQTT → `robo_pico`, with the ID scheme.
6. Build the Exploration Task (DFS) and the laptop map; test DFS in simulation first.
7. Build Phase 2: A\*, segment sending, reroute.
8. Update the dashboard (N entry, controls, manual pad, map display).
9. Remove the LIDAR and its code once nothing depends on it.

---
*This overview keeps the structure of the original `overview.md`, updated with the decisions agreed up to 2026-10-01: system statechart v9, the six-subsystem split and the Week 6 FRs. Repo status was read from the org repositories on 2026-10-01.*
