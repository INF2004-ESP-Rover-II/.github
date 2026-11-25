# INF2004 ESP - Rover II: Project Overview

Embedded Systems Project (INF2004), group P1-SE10. Rover II upgrades an existing mecanum-wheel rover and is separate from the module's default line-following project. The rover explores a grid of ArUco markers on its own, builds a map of it, and can then drive to a chosen marker.

Code: [integrated-rover-ii](https://github.com/INF2004-ESP-Rover-II/integrated-rover-ii). Setup and run instructions are in that repository's README.

## What the rover does

- **Phase 1, explore and map:** after the operator sets the start node and sends Start DFS, the Robo Pico explores depth-first on its own. At each node it samples the exits with the three ToF sensors and picks the next exit in the order forward, right, left, back, backtracking when none is left. A junction is a node with 2 to 4 exits, worked out from the node's exits, not from a marker type. The mission is COMPLETE when every reachable exit has been explored.
- **Phase 2, go to a marker:** the Robo Pico plans the shortest route over explored, unblocked edges (1000 per hop plus 1 per 90 degree turn) and drives it one hop at a time. It reports no route or an unknown target.
- **Manual driving:** from the dashboard, only while the mission is READY.
- **Dashboard:** a laptop app that sends commands and shows what the rover reports (mission and DFS state, ToF readings, heading, camera observations, the map and a 2D occupancy view). It does not run the exploration.
- The rover has no wheel encoders, so distance is open-loop. Position is corrected at each marker.

## Architecture

| Part | Runs on | Job |
|---|---|---|
| Sensor Pico | Raspberry Pi Pico W | 3 VL53L1X ToF sensors and the GY-6500 gyro (100 Hz). Obstacle flags. Sends ToF and heading to the Robo Pico over UART at 20 Hz, and STOP at once |
| Robo Pico | Raspberry Pi Pico W | Mission control, onboard DFS and map, route planner, motion control, safety interlock, MQTT. A main loop with timers, no RTOS (FreeRTOS is used only by the motor bench) |
| ESP32-S3 vision | ESP32-S3 with a USB webcam | Detects DICT_6X6 ArUco markers (74 mm) and sends ID, range and bearing to the Robo Pico over one binary UART frame |
| Dashboard | Laptop (Python, Mosquitto) | Operator interface over WiFi and MQTT. JSON, with a binary command format selectable |

| Link | From to | Carries | Format and rate |
|---|---|---|---|
| UART0 GP16 to GP13 | Sensor Pico to Robo Pico | ToF distances, heading, obstacle flags, STOP | Binary frame, 20 Hz, STOP sent at once |
| UART1 GPIO17 to GP21 | ESP32-S3 to Robo Pico | Marker ID, range, bearing, quality, camera status | Binary 7 to 31 bytes, CRC-8, 115200 8N1, about 5 Hz |
| USB-OTG (UVC) | Webcam to ESP32-S3 | 640x480 MJPEG frames | Newest frame only |
| MQTT robot/command, robot/heartbeat | Dashboard to Robo Pico | Operator commands, link heartbeat | JSON (binary 1 to 2 bytes ready), heartbeat every 500 ms |
| MQTT robot/status, telemetry, camera, map | Robo Pico to dashboard | Mission and DFS state, events, sensor data, camera observations, map snapshot | JSON |
| I2C0 and I2C1 | Sensors to Sensor Pico | 3 ToF on I2C0, gyro on I2C1 | ToF about 16.7 Hz each, gyro 100 Hz |
| PWM and direction GPIO | Robo Pico to the two L298N drivers | Four mecanum wheel speeds and directions | 10 kHz PWM, 5% ramp per 20 ms |

Control and safety paths: (1) a front obstacle under 150 mm makes the Sensor Pico send STOP and the motors brake; (2) an operator E-STOP cuts the motors and latches FAULT until Reset Safety; (3) no laptop message for 1.5 s stops the rover and pauses the mission; (4) no motor command within 300 ms, or a heading older than 600 ms, stops the motors; (5) a stalled or lost camera, or an image that is too old, brakes the rover and, if it does not recover within 1.5 s, cancels the journey and pauses the mission.

## Repositories

| Repository | What it is |
|---|---|
| [integrated-rover-ii](https://github.com/INF2004-ESP-Rover-II/integrated-rover-ii) | The main codebase: both Pico firmwares, the ESP32-S3 vision firmware, the dashboard, vision tools and tests |
| [ESP32-LogiCam-Test](https://github.com/INF2004-ESP-Rover-II/ESP32-LogiCam-Test) | Early camera test code |
| [3TOF-Test](https://github.com/INF2004-ESP-Rover-II/3TOF-Test) | Early three-ToF sensor bring-up test |

## Subsystems and owners

| Subsystem | Owner | Requirements |
|---|---|---|
| 1: Mission Control & Fault Handling | Teo Jia Hui | FR-01 to FR-08 |
| 2: Operator Interface & Communication | Teo Jia Hui | FR-09 to FR-15 |
| 3: Exploration & Obstacle Sensing | Genevivi Mia Er | FR-16 to FR-23 |
| 4: Mapping & Path Planning | Lee Ru Yan Jeslyn | FR-24 to FR-29 |
| 5: Vision Localisation | Lee Zong Yang Jeremy | FR-30 to FR-36 |
| 6: Motion Control & Heading | Lim Zi Ying | FR-37 to FR-42 |

## Requirements

These are the current requirements from the Tribunal 2 pre-submission. FR-10 and FR-39 are worded for what the dashboard supports: arc turn and parallel park are available through the motor bench.

### 1. Mission Control & Fault Handling (owner: Teo Jia Hui)

| ID | Requirement |
| --- | --- |
| FR-01 | On power-up, the Robo Pico shall start in IDLE, bring up the Sensor Pico and camera UART links and WiFi/MQTT, and report its state to the dashboard. |
| FR-02 | The Robo Pico shall run the mission states IDLE, READY, EXPLORING, PAUSED, COMPLETE and FAULT, and publish every state change with its reason. |
| FR-03 | Exploration shall start only after the operator sets the start node and sends Start DFS and the Robo Pico accepts it; a refused start shall be reported with its reason. |
| FR-04 | The system shall accept an operator stop at any time: the rover stops, the mission goes to PAUSED, and the map and DFS progress are kept. |
| FR-05 | The mission shall enter COMPLETE when every reachable exit has been explored. |
| FR-06 | On link loss, a front-obstacle stop or loss of usable camera input, the rover shall stop and the mission shall go to PAUSED without latching, publishing the reason. |
| FR-07 | On an operator E-STOP, all motor output shall be cut and the mission shall latch in FAULT until the operator sends Reset Safety; Reset Mission shall return the mission to IDLE. |
| FR-08 | The Robo Pico shall stop the motors on its own if no message from the laptop arrives within 1.5 s; the dashboard sends a heartbeat every 500 ms. |

### 2. Operator Interface & Communication (owner: Teo Jia Hui)

| ID | Requirement |
| --- | --- |
| FR-09 | The dashboard shall let the operator set the start node, start DFS, select a go-to target, stop, E-STOP, reset safety, reset the mission, zero the heading and request the map. |
| FR-10 | The dashboard shall let the operator drive the rover manually (forward/back, diagonal, strafe, rotate on the spot) while the mission is in Ready. |
| FR-11 | The dashboard shall only display what the Robo Pico reports: mission and DFS state, ToF readings, heading, camera observations, and the map (explored and seen nodes, blocked exits), plus a 2D occupancy view. |
| FR-12 | The laptop and the Robo Pico shall exchange commands, heartbeat, telemetry and events over WiFi/MQTT (robot/command, heartbeat, status, telemetry, camera, map) as JSON; a binary encoding of commands (1 to 2 bytes) and heartbeat (1 byte) shall be selectable. The Robo Pico shall reconnect automatically. |
| FR-13 | The Sensor Pico shall send ToF readings and heading to the Robo Pico over UART at 20 Hz (NFR-04), and send a STOP immediately instead of waiting for the next frame. |
| FR-14 | The system shall report to the operator when DFS completes, when a go-to target is reached, or when no route is left. |
| FR-15 | The dashboard shall log commands, events, state changes and received reports with timestamps to a text file for each session. |

### 3. Exploration & Obstacle Sensing (owner: Genevivi Mia Er)

| ID | Requirement |
| --- | --- |
| FR-16 | The Sensor Pico shall range the front, left and right ToF sensors concurrently, each with a 50 ms timing budget and a 60 ms interval (about 16.7 Hz per sensor). |
| FR-17 | The Sensor Pico shall discard invalid ToF readings and raise a ToF fault if a sensor stops responding; a fault or no usable return shall leave an exit unknown, never open. |
| FR-18 | The Sensor Pico shall flag an urgent front obstacle when a valid front reading is under 150 mm (clearing at 200 mm) and send a STOP immediately, independently of the mission state. |
| FR-19 | After Start DFS, the Robo Pico shall explore by depth-first search, choosing unexplored exits in the order forward, right, left, back relative to its heading. A junction is a node with 2 to 4 exits. |
| FR-20 | At each node the Robo Pico shall sample every exit from two consistent sensor packets and mark it blocked under 500 mm (configurable at Start DFS), open at 500 mm or more, and unknown on a fault. |
| FR-21 | When a node has no eligible unexplored exit, the rover shall backtrack along its DFS stack to the most recent node that still has one. |
| FR-22 | The Robo Pico shall perform DFS route selection, backtracking and node-to-node movement onboard after an accepted Start DFS, with no movement segments from the laptop. |
| FR-23 | The Robo Pico shall publish exploration progress, confirmed arrivals, map updates, completion and failure reasons for the dashboard to display. |

### 4. Mapping & Path Planning (owner: Lee Ru Yan Jeslyn)

| ID | Requirement |
| --- | --- |
| FR-24 | The Robo Pico shall store the marker grid map (up to 64 nodes: marker ID, grid position, visited flag and N/E/S/W exit states) and publish it on robot/map. The laptop shall draw a 2D occupancy view from valid ToF readings and the rover pose, for display only. |
| FR-25 | The Robo Pico shall count each visited node once, so revisited markers are not counted twice. |
| FR-26 | The Robo Pico shall detect when no reachable node has an unexplored exit, end exploration and report the number of nodes visited. |
| FR-27 | On a go-to command, the Robo Pico shall compute the shortest route from the confirmed current node to the target over explored, unblocked edges (1000 per hop plus 1 per 90° turn, ties to the lowest marker IDs), or report no route or unknown target. |
| FR-28 | The Robo Pico shall drive the route one hop at a time, start the next hop only after the previous hop's result, and re-plan from the confirmed node after each arrival. |
| FR-29 | If a hop fails, the route shall stop and report why without guessing the rover's position. Edges marked blocked shall be routed around with no retry limit, and “no route left” reported when none remains. |

### 5. Vision Localisation (owner: Lee Zong Yang Jeremy)

| ID | Requirement |
| --- | --- |
| FR-30 | The ESP32-S3 shall capture 640x480 MJPEG frames from the USB webcam and process only the newest frame, dropping older frames. |
| FR-31 | The ESP32-S3 shall detect DICT_6X6 ArUco markers (74 mm) and estimate each marker's ID, range (mm) and bearing (0.01°, positive to the right) at ≥4.5 fps for markers 50 to 120 cm away, with range error ≤2% and bearing error ≤2° against a tape-measured layout. |
| FR-32 | The ESP32-S3 shall report only marker IDs 1 to 249 and drop any detection with a shape error over 0.25, a side under 20 px or a range over 1500 mm. |
| FR-33 | The ESP32-S3 shall send each result to the Robo Pico as one binary UART frame (115200 8N1: sync, sequence, status, image age, 0 to 4 markers of ID, range, bearing and quality, CRC-8). The Robo Pico shall discard frames with a bad CRC, an invalid status or count, marker ID 0, quality over 4 or a gap between bytes, and ignore repeated sequence numbers. |
| FR-34 | The Robo Pico shall confirm arrival at the target node only from stationary reference-marker observations at about 600 mm (3 frames within ±100 mm after a 300 ms settle) or a verified front-obstacle stop, and only then mark the node visited and set the rover's map position to it. |
| FR-35 | The Robo Pico shall limit forward driving while searching for a reference marker to 10 s in total and each node-to-node journey to 30 s; choosing a target at a node shall time out after 5 s. On a timeout it shall stop, pause exploration and publish the reason, without confirming arrival. |
| FR-36 | The ESP32-S3 shall report STALLED after 0.5 s without a processed frame and CAMERA_LOST after 5 s, repeated every 0.2 s. On STALLED, CAMERA_LOST or a stale image while driving, the Robo Pico shall brake and, unless 2 good frames arrive within 1.5 s, cancel the journey and pause exploration. |

### 6. Motion Control & Heading (owner: Lim Zi Ying)

| ID | Requirement |
| --- | --- |
| FR-37 | The Sensor Pico shall sample the GY-6500 gyro at 100 Hz, zero its offset while still at start-up, and integrate yaw into a heading (clockwise positive, 0 to 360°) that the operator can re-zero. |
| FR-38 | The Robo Pico shall convert movement commands (forward speed, sideways speed, turn rate) into PWM and direction signals for the four mecanum wheels through the two L298N drivers, using inverse kinematics. |
| FR-39 | In manual mode, the rover shall support forward/back, diagonal, strafe left/right and rotate on the spot from the dashboard. Arc turn and parallel park are available through the motor bench only; the operator parks by holding strafe. |
| FR-40 | In autonomous mode, the rover shall use only forward/back with heading hold and rotate on the spot; strafe, diagonal, arc turn and park shall be manual only. |
| FR-41 | The rover shall use the gyro heading to hold a straight course while driving, and shall brake a rotation once it is within ±3° of the requested heading. |
| FR-42 | The rover shall ramp drive PWM at 5% per 20 ms, and cut motor PWM immediately on an E-STOP, front-obstacle STOP or operator stop, when no command or lease refresh arrives within 300 ms, or when the heading is older than 600 ms during an autonomous move. |

### Non-functional requirements

| ID | Requirement |
| --- | --- |
| NFR-01 | Each ToF sensor shall produce a new reading at ≥10 Hz. |
| NFR-02 | The IMU gyro shall be sampled at 100 Hz to support heading integration. |
| NFR-03 | On detecting a front obstacle within 150 mm, the rover shall halt within 200 ms. |
| NFR-04 | The Sensor Pico shall relay ToF readings and heading to the Robo Pico over UART at ≥20 Hz, with STOP events sent immediately. |
| NFR-05 | A marker result shall reach the Robo Pico within 300 ms of being sent, and the image it came from shall be no older than 750 ms when used for driving (1500 ms for choosing a target); older results shall be ignored. |

## Status (25 November 2026)

A snapshot from the team's notes; update it as things change.

| Area | State |
|---|---|
| Exploration (DFS, map, journeys) | Runs on the Robo Pico. As of 6 Nov it had not completed a full map on the rover |
| Go-to planner | Runs on the Robo Pico and works in a board dry run. It is on a separate branch, not on `main`, and has not driven a real route |
| Motion control | Implemented and passing its host tests (96). Floor measurements of turn accuracy, drift and stopping time are still to be done |
| Sensor and camera links | The Sensor Pico packet rate (20 Hz target) and the 200 ms obstacle halt have not been measured on the rover |
| Dashboard and MQTT | JSON in use. The binary command format is ready but not tested on the real rover |

## Open items

1. Merge the go-to planner branch with the current `main` and drive a real route.
2. Complete a full Phase 1 run on the rover. On 6 Nov it stopped on a dead left ToF sensor, lost turn clearance and a lost position reference.
3. Firmware: separate the 10 s reference-search timer from the 30 s journey timer (FR-35).
4. Dashboard: stop it clearing an E-STOP by itself after 3 s, so FR-07 holds.
5. Measure the 20 Hz packet rate, the 10 Hz ToF rate, the 100 Hz gyro rate and the 200 ms obstacle halt.
6. Test the binary command format on the rover.
7. Decide what manual forward should do when the front ToF has failed or the Sensor Pico is silent.
8. Fix the Python dashboard tests that no longer match the dashboard.

## Risks

- Timing after bit-packing, and packed data shared between tasks.
- C and Python must decode the same packed formats.
- Pin conflicts and shared buses (I2C0 is shared by three ToF sensors).
- No wheel encoders, so movement is open-loop and depends on the gyro and marker corrections.
- Dependence on WiFi and the laptop: the rover stops if the link is lost for 1.5 s.
- WiFi credentials appear in some firmware files. Rotate the password and keep credentials out of the repository.

## Code style

The motor code follows BARR-C:2018 as the team applies it (checked by hand and with a script, not a BARR-C tool). Known deviations are explained in the [repo README](https://github.com/INF2004-ESP-Rover-II/integrated-rover-ii): plain `unsigned` bit-fields in the private Motion Control state (`mc_t`), and four headers and config files that keep a hand layout instead of clang-format.

## Team

| Buddy | Name | Student ID | Subsystem |
|---|---|---|---|
| 1 | Genevivi Mia Er | 2502517 | 3 |
| 2 | Lee Ru Yan Jeslyn | 2503391 | 4 |
| 3 | Lee Zong Yang Jeremy | 2500330 | 5 |
| 4 | Lim Zi Ying | 2503397 | 6 |
| 5 | Teo Jia Hui | 2502328 | 1 and 2 |
