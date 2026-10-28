# 🚁 Mini Drone (Quadcopter Flight Controller)

> A small brushed-motor quadcopter able to lift a 20 g payload, with a self-written flight controller on an Arduino and an MPU-6050 IMU.

![badge](https://img.shields.io/badge/Arduino-00878F?logo=arduino&logoColor=white&style=flat) ![badge](https://img.shields.io/badge/C%2B%2B-00599C?logo=cplusplus&logoColor=white&style=flat) ![badge](https://img.shields.io/badge/PID%20Control-222222?style=flat)
![tests](https://github.com/MelbinManoj/mini-drone/actions/workflows/tests.yml/badge.svg)

> 📝 **About this code:** I built this project during my B.Tech, but the original source code was lost. The code here is a **rebuilt version (2026)** based on the original design. It compiles for the Arduino Uno, and its logic is checked by automated tests that run on simulated sensor data. It has not yet been re-tested on the physical hardware.

## 📖 Overview

This project is a compact quadcopter built from the ground up. Its flight controller is written from scratch instead of using ready-made firmware. An **MPU-6050** measures rotation and acceleration, a **complementary filter** estimates roll and pitch, and **PID controllers** adjust four coreless motors through MOSFETs to keep the drone level. You control it over serial from a PC or a Bluetooth app.

## ✨ Features

- Raw MPU-6050 driver over I²C (no external libraries) with gyro bias calibration at startup
- **Complementary filter** (98 % gyro / 2 % accelerometer) for drift-free roll and pitch
- Roll, pitch and yaw-rate **PID** with integral anti-windup, in a 250 Hz control loop
- **Quad-X motor mixer** with output clamping to the PWM range
- Safety: arms only at zero throttle, **cuts motors above 45° tilt**, cuts motors if the IMU fails, and **lands slowly if commands stop** for 1 s
- Simple text commands: `A` arm, `D` disarm, `T<0-255>` throttle, `R`/`P` roll/pitch target

**States:** `disarmed` → `A` (throttle 0) → `ARMED` → crash / link loss / `D` → `disarmed`

## 🧰 Components

| Component | Qty |
|---|:---:|
| Arduino Nano or Uno | 1 |
| MPU-6050 IMU | 1 |
| 8520 coreless motors + 65 mm props (2 CW, 2 CCW) | 4 |
| SI2302 MOSFETs + flyback diodes | 4 |
| 1S LiPo (~500 mAh) + 5 V boost converter | 1 |
| Lightweight frame | 1 |
| HC-05 Bluetooth (optional) | 1 |

## 🔌 Wiring

| Connection | Arduino pin |
|---|---|
| MPU-6050 SDA / SCL | A4 / A5 |
| Front-left motor MOSFET | D3 |
| Front-right motor MOSFET | D9 |
| Rear-left motor MOSFET | D10 |
| Rear-right motor MOSFET | D11 |
| HC-05 TX / RX | D0 / D1 (RX through a divider) |

## 🚀 Getting Started

1. Open [`mini_drone/mini_drone.ino`](./mini_drone/mini_drone.ino) in the Arduino IDE. The `flight_controller.h` file in the same folder opens with it.
2. Select **Arduino Uno** (or Nano) and upload. No extra libraries are needed.
3. Open the Serial Monitor to watch the live readings.
4. **Calibrate:** ⚠️ Remove the propellers for first tests. Check that tilting the drone right shows positive `roll`, then tune the PID gains on a tethered test rig before free flight.

## 🧪 Tests

All decision logic lives in [`flight_controller.h`](./mini_drone/flight_controller.h), with no Arduino dependencies, so it can be tested on a PC with simulated sensor data:

```bash
cd test
g++ -std=c++11 -I../mini_drone test_flight_controller.cpp -o test && ./test
```

What's tested:

- ✅ Attitude estimate converges to the true tilt (30° test)
- ✅ Gyro drift is corrected by the accelerometer
- ✅ Mixer sends more thrust to the correct motors, clamped to 0–255
- ✅ **Closed-loop physics simulation**: a drone tilted 15° is levelled to < 0.5° within 5 s
- ✅ Arming rules, 45° crash cutoff and slow landing on link loss

These tests run automatically on every push with GitHub Actions.

## 📂 Project Structure

```text
mini-drone/
├── mini_drone/
│   ├── mini_drone.ino      # hardware layer: pins, sensors, outputs
│   └── flight_controller.h   # decision logic (unit-tested)
├── test/
│   └── test_flight_controller.cpp
└── README.md
```

## 👤 Author

**Melbin Manoj** — [GitHub](https://github.com/MelbinManoj) · [LinkedIn](https://www.linkedin.com/in/melbin-manoj-8ba512269/)
