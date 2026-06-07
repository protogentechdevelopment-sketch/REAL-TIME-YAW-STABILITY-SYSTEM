# 🚗 REAL-TIME-YAW-STABILITY-SYSTEM

> A fully embedded, hardware-validated yaw stability control system for Electric Vehicles using ESP32, MPU6050, L298N Motor Driver, LM2596 Buck Converter, and BO Motors — with PID, Selective Braking  & Torque vectoring, and LQR strategies.

---

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Hardware Components](#hardware-components)
- [Software Tools](#software-tools)
- [Circuit & Pin Connections](#circuit--pin-connections)
- [How It Works](#how-it-works)
- [Communication Protocols](#communication-protocols)
- [Results](#results)
- [Future Scope](#future-scope)

---

## Overview

Yaw stability in Electric Vehicles is a critical safety challenge — especially in independently driven multi-motor configurations where wheel torque can be precisely controlled but complex dynamics arise during cornering, lane changes, and high-speed maneuvers. Traditional solutions rely on expensive MATLAB/dSPACE platforms or supervisory PCs that are impractical for compact or low-cost deployment.

This project presents a **fully embedded, hardware-validated yaw stability control system** implemented entirely on an **ESP32 microcontroller**. The system reads real-time yaw rate from an **MPU6050 IMU sensor** via I2C, computes corrective torque vectoring commands using onboard **PID, selective braking, and LQR control algorithms**, and applies differential torque to two independently driven **BO motors** through an **L298N motor driver** — all without reliance on any supervisory computer or simulation platform.

---

## Features

- ✅ Three embedded control strategies — PID, Role/Pitch/Yaw and LQR
- ✅ Real-time yaw rate measurement via MPU6050 gyroscope over I2C at 400 kHz
- ✅ Auto-calibration of gyroscope bias using 1000-sample averaging at startup
- ✅ Continuous idle auto-correction (low-pass gyro drift compensation)
- ✅ Adaptive PID gains for straight-line forward motion control
- ✅ Closed-loop yaw angle control within ±0.5° accuracy
- ✅ Differential torque vectoring via independently driven left and right BO motors
- ✅ Fallback selective braking for extreme maneuver conditions
- ✅ IMU watchdog that halts motors within 100 ms on sensor failure
- ✅ Full UART serial monitor output for real-time debugging at 115200 baud
- ✅ Low-cost, compact, and scalable — suitable for student research and EV prototypes

---

## System Architecture

The system is divided into three functional layers:

```
┌─────────────────────────────────────────────────────────────┐
│  SENSING LAYER       MPU6050 (6-axis IMU: Gyro + Accel)     │
│                      Z-axis Yaw Rate → Bias Calibration     │
├─────────────────────────────────────────────────────────────┤
│  CONTROL LAYER       ESP32 (Dual-Core Xtensa LX6 @ 240 MHz) │
│                      PID / SMC / LQR Control Algorithms     │
│                      Yaw Angle Integration (Trapezoidal)    │
├─────────────────────────────────────────────────────────────┤
│  ACTUATION LAYER     L298N Motor Driver (H-Bridge)          │
│                      BO Motor Left + BO Motor Right         │
│                      PWM @ 5 kHz, 8-bit Resolution          │
└─────────────────────────────────────────────────────────────┘
```

### Block Diagram

```
LM2596 Buck Converter (5V Regulated)
        │
        ▼
MPU6050 (I2C) ──► ESP32 Main Microcontroller ──► L298N Motor Driver ──► Motor 1 (Left)
                        │                                               └──► Motor 2 (Right)
                        │
                  UART Serial Monitor (115200 baud)
                  └── Real-time Yaw, Error, Correction, Motor PWM

Power Supply (Li-ion 7.4V–12.6V) ──► LM2596 (5V) ──► ESP32 + MPU6050
                                  └──────────────────► L298N (Direct Battery Rail)
```

---

## Hardware Components

| Component | Model | Interface | Purpose |
|---|---|---|---|
| Microcontroller | ESP32 (Dual-Core Xtensa LX6) | — | Central processing, control computation |
| IMU Sensor | MPU6050 | I2C (GPIO21/GPIO22) | 6-axis gyroscope + accelerometer |
| Motor Driver | L298N Dual H-Bridge | GPIO (PWM + Direction) | Independent left/right motor control |
| DC Motors | BO Geared DC Motor × 2 | L298N Output Terminals | Differential torque actuation |
| Voltage Regulator | LM2596 Step-Down Buck Converter | — | Regulated 5V supply for ESP32 + sensor |
| Power Source | Li-ion Battery Pack (7.4V–12.6V) | — | System power |

---

## Software Tools

| Tool | Purpose |
|---|---|
| **Arduino IDE** | Firmware development, compilation, and flashing |
| **Espressif Arduino Core** | ESP32 board support (LEDC PWM, I2C, UART, GPIO) |
| **Wire.h Library** | I2C communication with MPU6050 |
| **Arduino Serial Monitor** | Real-time UART debugging and performance visualization |

---

## Circuit & Pin Connections

### MPU6050 → ESP32 (I2C)

| MPU6050 Pin | ESP32 Pin | Description |
|---|---|---|
| VCC | 3.3V | Power supply |
| GND | GND | Common ground |
| SCL | GPIO22 | I2C Clock |
| SDA | GPIO21 | I2C Data |
| AD0 | GND | I2C Address = 0x68 |

### L298N Motor Driver → ESP32 (PWM + Direction)

| L298N Pin | ESP32 Pin | Description |
|---|---|---|
| ENA (Left Motor PWM) | GPIO25 | LEDC PWM speed control — Left motor |
| ENB (Right Motor PWM) | GPIO14 | LEDC PWM speed control — Right motor |
| IN1 | GPIO26 | Left motor direction 1 |
| IN2 | GPIO27 | Left motor direction 2 |
| IN3 | GPIO12 | Right motor direction 1 |
| IN4 | GPIO13 | Right motor direction 2 |
| VCC | Battery Direct (7.4–12.6V) | Motor power rail |
| GND | GND | Common ground |

### LM2596 Buck Converter → Power Rail

| LM2596 Terminal | Connected To | Description |
|---|---|---|
| VIN+ | Battery (+) | Input from Li-ion pack |
| VIN− | Battery (−) | Input ground |
| VOUT+ | ESP32 VIN / MPU6050 VCC | Regulated 5V output |
| VOUT− | GND | Output ground |

---

## How It Works

1. **Boot & Init** — ESP32 initializes I2C, UART, GPIO direction pins, and LEDC PWM channels at 5 kHz / 8-bit resolution.
2. **Gyro Calibration** — 1000 Z-axis MPU6050 readings sampled at 2 ms intervals; mean bias offset computed and stored as `gyroOffsetZ`.
3. **Control Loop** — MPU6050 Z-axis raw value read, offset-subtracted, converted to °/s via sensitivity factor 131.0 LSB/(°/s).
4. **Auto-Drift Correction** — If stationary for >2000 ms, gyro offset slowly tracks current reading via low-pass filter (blend = 0.01).
5. **Deadband Filter** — Readings within ±0.3°/s (rotation) or ±1.0°/s (forward mode) suppressed to prevent noise integration.
6. **Yaw Angle Integration** — Trapezoidal discrete-time integration accumulates yaw angle from calibrated angular velocity × elapsed Δt.
7. **Control Computation** — Error = Target Yaw − Measured Yaw; corrective torque computed via selected strategy (PID / SMC / LQR).
8. **Motor Actuation** — PWM duty cycles for left and right motors adjusted differentially to generate corrective yaw moment.
9. **Selective Braking Fallback** — Engaged via L298N direction reversal when torque vectoring alone cannot correct extreme conditions.
10. **IMU Watchdog** — `checkIMU()` halts all motors if no sensor update received within 100 ms.

### Control Algorithm Summary

```
PID Controller (Baseline):
  error = targetYaw - measuredYaw
  P = Kp × error
  I = Ki × ∫error dt   (clamped ±50 to prevent windup)
  D = Kd × Δerror / Δt
  correction = P + I + D

SMC (Sliding Mode Controller):
  s = error + λ × d(error)/dt
  u = K × sign(s) with boundary layer smoothing

LQR (Linear Quadratic Regulator):
  u = −K × [yaw_angle; yaw_rate]
  K gains pre-computed offline for optimal response
```

### Demo Sequence

| Demo | Description | Duration |
|---|---|---|
| Demo 1 — Open Loop | Motors OFF; robot pushed manually to visualize uncontrolled drift | 15 s |
| Demo 2 — PID Hold | Closed-loop PID maintains 0° heading against manual disturbances | 20 s |
| Demo 3 — Forward Rotation | PID drives +90° → +180° → +270° → +360° with ±0.5° accuracy | Sequential |
| Demo 4 — Backward Rotation | PID drives −90° → −180° → −270° → −360° with ±0.5° accuracy | Sequential |
| Demo 5 — Forward PID | Adaptive PID straight-line driving for 5 seconds with yaw correction | 5 s |

### Key Parameters

| Parameter | Value |
|---|---|
| Rotation PID — Kp / Ki / Kd | 2.0 / 0.1 / 1.0 |
| Forward PID — Kp / Ki / Kd | 0.8 / 0.02 / 0.3 |
| Motor PWM Max / Min | 220 / 100 |
| Base Forward Speed | 130 PWM counts |
| Max PID Correction Per Side | 30 PWM counts |
| Gyro Sensitivity Factor | 131.0 LSB/(°/s) |
| PWM Frequency | 5000 Hz |
| PWM Resolution | 8-bit (256 levels) |
| Angle Tolerance | ±0.5° |
| UART Baud Rate | 115200 bps |

---

## Communication Protocols

### I2C (MPU6050 Sensor Interface)
- Two-wire synchronous protocol: SDA (GPIO21) + SCL (GPIO22)
- Operating speed: **400 kHz Fast Mode**
- MPU6050 device address: `0x68` (AD0 tied to GND)
- ESP32 acts as I2C master; MPU6050 is the slave
- Managed via Arduino `Wire.h` library

**Key I2C Operations:**
```
Register 0x6B ← 0x00        // Wake MPU6050 from sleep
Register 0x47–0x48 → Read   // Z-axis gyroscope raw data (16-bit signed)
```

### UART (Serial Debug Output)
- Asynchronous serial at **115200 bps**
- Used for startup banner, calibration offset display, real-time yaw/error/correction/motor logs
- Debug output throttled to every 300 ms to avoid control loop interference

**Real-time UART Output Format:**
```
[FWD PID] t=1.2s heading=-359.52° err=-0.04° corr=0.0 Kp=0.800 Kd=0.300 L=130 R=130
```

### LEDC PWM (Motor Speed Control)
- ESP32 built-in LED Control peripheral
- 2 independent channels at 5 kHz, 8-bit resolution
- `ledcAttach(pin, freq, resolution)` → `ledcWrite(pin, duty)`

---

## Results

The system was successfully implemented and validated on a bench rig and scaled EV prototype. Key outcomes:

- **Gyro Calibration** — `gyroOffsetZ` consistently within ±0.5°/s across multiple startup trials, confirming stable MPU6050 I2C initialization.
- **Deadband Suppression** — ±1.0°/s threshold effectively prevented false angle accumulation during stationary periods.
- **Rotation Accuracy** — All target angles (+90°, +180°, +270°, +360° and their negative counterparts) achieved within **±0.5° tolerance** using closed-loop PID.
- **PID Heading Hold** — Successfully maintained 0° heading against manual disturbances for 20 seconds; maximum hold error was 9.63°.
- **Forward Motion** — Adaptive PID kept heading error below 29.15° maximum during a 5-second straight drive with active yaw correction.
- **Motor Control** — PWM duty cycle of 220/255 at 5 kHz delivered stable rotation within L298N current ratings.
- **Multi-strategy Comparison** — PID served as reliable baseline; SMC provided robust nonlinear handling; LQR achieved optimal response — all running on the same ESP32 platform without external computing.

---

## Future Scope

- **Wi-Fi Data Logging** — Transmit real-time yaw, error, and motor data to ThingSpeak or a custom web dashboard using ESP32's built-in Wi-Fi.
- **Extended Kalman Filter** — Fuse gyroscope + accelerometer + GPS for more accurate yaw rate, side-slip angle, and lateral velocity estimation.
- **Steering Angle Feedback** — Compute reference yaw rate from desired path for true closed-loop trajectory tracking.
- **BLDC Hub Motors** — Replace BO motors with brushless hub motors and VESC controllers for full-scale EV application.
- **CAN Bus Integration** — Interface with commercial EV powertrain and BMS components via CAN bus.
- **Regenerative Braking** — Combine selective braking with energy recovery for efficiency-optimized yaw correction.
- **ML-based Control** — Replace threshold/gain logic with a trained neural network or reinforcement learning policy for adaptive stability management.
- **STM32H7 Co-processor** — Offload computationally intensive LQR matrix operations to a dedicated automotive-grade MCU while ESP32 handles IoT and HMI.

---

> *Built to demonstrate that effective yaw stability control for Electric Vehicles is achievable on compact, low-cost embedded hardware — no MATLAB, no dSPACE, no supervisory PC required.*
