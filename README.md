# PHELAN BUTTERFLY - Bionic RC Ornithopter System

[![C++17](https://img.shields.io/badge/Language-C%2B%2B17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![Hardware](https://img.shields.io/badge/MCU-ESP32--C3%20%7C%20ESP32--C6-green.svg)](https://www.espressif.com/)
[![License: Proprietary](https://img.shields.io/badge/License-Proprietary-red.svg)](LICENSE)
[![Control Protocol](https://img.shields.io/badge/Control-CRSF%20%7C%20SBUS%20%7C%20WiFi%20Web-orange.svg)]()
[![Web GCS](https://img.shields.io/badge/Web%20Cockpit-WebSocket%2050Hz-cyan.svg)]()
[![Author](https://img.shields.io/badge/Author-PhelanDev-red.svg)](https://github.com/PhelanDev)

**PHELAN BUTTERFLY** is a high-performance bionic ornithopter flight control system designed for **ESP32-C3** and **ESP32-C6** microcontrollers. It features bio-inspired flapping wing kinematics, real-time dynamic amplitude adjustment, and versatile control options via traditional RC transmitters or direct web browser Ground Control Station (GCS).

---

## Key Features

- **Dynamic Amplitude Kinematics (`dynamicAmp`)**: Automatically contracts flapping amplitude from **55.0°** (at idle throttle 1050 µs) down to **35.0°** (at full throttle 2000 µs), generating massive lift for take-off while eliminating destructive inertial vibrations and thermal overload at high flap rates.
- **300Hz Servo Control Loop**: High-frequency PWM servo updates (`LOOP_DT_US = 3333 µs`) with exponential moving average (EMA) smoothing for sub-millimeter wing flap synchronization.
- **3-in-1 Unified Control Protocols**:
  - **CRSF (ExpressLRS)**: High-speed 420,000 baud receiver decoding.
  - **SBUS**: 100,000 baud serial packet parser.
  - **Wi-Fi Web GCS**: Binary WebSocket control loop operating at 50Hz.
- **Pure Cockpit Web Ground Control Station**: Ultra-lightweight (~43.7 KB) tactical interface embedded directly into the ESP32 flash memory. Includes:
  - Orthogonal joystick kinematics with zero-throttle default baseline and **Sticky Throttle Hold**.
  - **Pre-Flight Arming Interlock**: Prevents accidental arming unless throttle is strictly at 0%.
  - Embedded Base64 Phelan calligraphy logo for 100% offline flight field operation.
- **Bionic Flight Modes**:
  - **Normal Flap**: Triangle or Sine wave flapping mode.
  - **Perch Mode**: V-dihedral wing lock (+30°) for perching or gliding.
  - **Autonomous Failsafe Dihedral (+15°)**: Automatically activates if the control link is lost for >500ms to ensure gentle glide landings.
- **Hardware Protection**: Low-level constructor (`disable_bod_early`) suppressing brownout resets during heavy servo spikes.

---

## Firmware Matrix (12 Available Firmware Options)

The repository provides **12 ready-to-flash firmware options** categorized into 6 functional branches. Each branch includes both a **Modular version** (for structured development) and a **Single-file All-in-One version** (for instant 1-click flashing in Arduino IDE):

| # | Firmware Branch | MCU | Control Mode | Modular Firmware | Single-File All-in-One |
| :-: | :--- | :--- | :--- | :--- | :--- |
| **1** | **Sine Wave (Pure RC)** | ESP32-C3 | Pure RC (CRSF/ELRS or SBUS) | [`butterfly_esp32c3mini_Phelan_sin`](butterfly_esp32c3mini_Phelan_sin/) | [`butterfly_esp32c3mini_Phelan_single_sin`](butterfly_esp32c3mini_Phelan_single_sin/) |
| **2** | **Triangle Wave (Pure RC)** | ESP32-C3 | Pure RC (CRSF/ELRS or SBUS) | [`butterfly_esp32c3mini_Phelan_tria`](butterfly_esp32c3mini_Phelan_tria/) | [`butterfly_esp32c3mini_Phelan_single_tria`](butterfly_esp32c3mini_Phelan_single_tria/) |
| **3** | **C3 3-in-1 (V1 - SoftAP)** | ESP32-C3 | Standalone AP (`192.168.4.1`) + CRSF + SBUS | [`code_3in1/c3mini3in1`](code_3in1/c3mini3in1/) | [`single_code_3in1/c3mini3in1`](code_3in1/single_code_3in1/c3mini3in1/) |
| **4** | **C3 3-in-1 (V2 - Station STA)** | ESP32-C3 | Wi-Fi Router/Hotspot (STA) + Bridge + CRSF + SBUS | [`code_3in1/c3mini3in1_2`](code_3in1/c3mini3in1_2/) | [`single_code_3in1/c3mini3in1_2`](code_3in1/single_code_3in1/c3mini3in1_2/) |
| **5** | **C6 3-in-1** | ESP32-C6 | Standalone AP + CRSF + SBUS for ESP32-C6 | [`code_3in1/c6mini`](code_3in1/c6mini/) | [`single_code_3in1/c6mini`](code_3in1/single_code_3in1/c6mini/) |
| **6** | **C3 Wi-Fi STA (Pure Wi-Fi)** | ESP32-C3 | **Dedicated Wi-Fi Station** *(Hardware RC code removed)* | [`c3wifi_sta/c3wifi_sta`](c3wifi_sta/c3wifi_sta/) | [`c3wifi_sta/single_c3wifi_sta`](c3wifi_sta/single_c3wifi_sta/) |

---

## Hardware Wiring & Pinout

| Pin (ESP32-C3) | Connection | Description |
| :--- | :--- | :--- |
| **GPIO 5** | Left Wing Servo Signal | 300Hz PWM output for Left Wing |
| **GPIO 6** | Right Wing Servo Signal | 300Hz PWM output for Right Wing |
| **GPIO 3** | RC Receiver Signal (RX) | CRSF / ExpressLRS or SBUS serial input *(Not needed for `c3wifi_sta`)* |
| **GPIO 8** | Status LED | Blinking pattern indicator (Wi-Fi status, Arm state) |
| **5V / VIN** | Power Input (+) | Regulated 5V BEC (stepped down from 2S LiPo 7.4V - 8.4V) |
| **GND** | Ground (-) | Common ground for 2S Battery, MCU, and Servos |

> **Power Supply & Battery Requirements**:
> - **Required Battery**: **2S LiPo (7.4V - 8.4V)** to provide sufficient torque and high-frequency 300Hz flapping power for both servos.
> - **MCU Power**: Use a 5V BEC or DC-DC step-down converter from the 2S battery to the `5V / VIN` pin (never connect 8.4V directly to the ESP32 5V rail).
> - **Servo Power**: Connect servo positive to the appropriate power rail (direct 2S for HV servos, or a dedicated BEC rail) and **always share a common GND** with the ESP32.
> - For ESP32-C6 (`c6mini`), the status LED is connected to **GPIO 14**.

---

## Quick Start Guide

### 1. Arduino IDE Setup
1. Install **Arduino IDE** (v2.x recommended).
2. Install the **ESP32** board package by Espressif (`v3.0.x` or later).
3. Required external libraries:
   - `ESP32Servo`
   - `ESPAsyncWebServer` & `AsyncTCP` *(Only needed for modular compilation; single-file sketches compile directly)*.
4. Select Board: `ESP32C3 Dev Module` (or `ESP32C6 Dev Module`).
   - **Flash Frequency**: `80MHz`
   - **Flash Size**: `4MB`
   - **USB CDC On Boot**: `Enabled` (for USB Type-C upload & logging).

### 2. Flash and Fly
- **Flying via Standalone SoftAP Wi-Fi** (Direct phone connection):
  - Flash `single_code_3in1/c3mini3in1/c3mini3in1.ino`.
  - Connect your smartphone to Wi-Fi SSID `PHELAN BUTTERFLY` (Password: `12345678`).
  - Open browser: `http://192.168.4.1`.
- **Flying via Outdoor Station Router / Phone Hotspot**:
  - Flash `c3wifi_sta/single_c3wifi_sta/single_c3wifi_sta.ino`.
  - Configure `ROUTER_SSID` and `ROUTER_PASS` to match your local network or phone hotspot.
  - Open browser: `http://phelan-102.local` (or the IP assigned by the router).
- **Flying with Traditional RC Transmitter**:
  - Flash `butterfly_esp32c3mini_Phelan_single_tria.ino` or `single_code_3in1/c3mini3in1.ino`.
  - Connect your ELRS/SBUS receiver to **GPIO 3**. System auto-detects protocol.

---

## Control Range Expectations

| Protocol / Setup | Hardware Environment | Typical Effective Range |
| :--- | :--- | :---: |
| **Direct Wi-Fi SoftAP** | Direct connection to ESP32 onboard antenna | **30m – 50m** |
| **Wi-Fi Station (Hotspot)** | Connected to phone 4G/5G mobile hotspot | **40m – 60m** |
| **Wi-Fi Station (Outdoor AP)** | High-power outdoor Wi-Fi router placed at flying field | **80m – 150m** |
| **ExpressLRS (ELRS 2.4GHz)** | External receiver connected to GPIO 3 | **500m – 2+ km** |

---

## License

This project is proprietary software (**Proprietary License**). All rights reserved by **PhelanDev**. Unauthorized copying, modification, reverse engineering, redistribution, or commercial use is strictly prohibited. See the [LICENSE](LICENSE) file for details.

Developed with passion by **[PhelanDev](https://github.com/PhelanDev)**.
