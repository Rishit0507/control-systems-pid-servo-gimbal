# Camera Gimbal Stabilization Using PID Control

A 2-axis camera gimbal that keeps a camera level despite external disturbances, built with Arduino UNO and an MPU6050 sensor. Uses independent PD controllers for pitch and roll with real-time angle estimation and MATLAB-based simulation validation.

## Project Overview

This project implements a closed-loop stabilization system demonstrating feedback control, PID tuning, sensor fusion, and disturbance rejection. The system is modeled in MATLAB/Simulink and validated against real hardware data — showing where simulation matches reality and where it doesn't.

## Key Features

- **2-DOF Stabilization** - Independent pitch and roll control with axis-decoupled gain tuning
- **Complementary Filter** - Fuses gyroscope and accelerometer data for stable angle estimation
- **Asymmetric PD Gains** - Different Kp/Kd per axis based on mechanical load differences
- **MATLAB Validation** - Transfer function model compared against real hardware step response
- **Serial Data Logging** - Live CSV output at 115200 baud for MATLAB analysis

## Hardware Components

- **Arduino UNO** - Main microcontroller running PD control loop at 100Hz
- **MPU6050** - 6-axis IMU providing gyroscope and accelerometer data over I2C
- **2x SG90 Servo** - Pitch (D9) and roll (D10) axes
- **Buck Converter** - External 5V supply for servos
- **Mechanical Frame** - Cardboard / 3D printed gimbal platform

## How It Works

### Angle Estimation
Raw gyro and accelerometer data are fused using a complementary filter. The gyro is fast but drifts over time — the accelerometer is noisy but stable long term. Blending both gives a clean, drift-free angle estimate.

```
angle = 0.96 * (angle + gyro_rate * dt) + 0.04 * acc_angle
```

### PD Control
Two independent PD controllers run in parallel — one for pitch, one for roll. Each computes a servo correction based on the error between desired angle (0°) and measured angle.

```
u(t) = Kp * e(t) + Kd * de(t)/dt
```

Pitch and roll use different gains because the roll axis carries more mechanical load:

| Axis  | Kp  | Kd  | Servo Range  |
|-------|-----|-----|--------------|
| Pitch | 1.8 | 1.2 | 45° to 135°  |
| Roll  | 1.2 | 1.8 | 60° to 120°  |

Ki is set to 0 — integral action causes windup on a continuously moving gimbal and makes stabilization worse.

## Technical Implementation

**Control Loop Rate:** 100Hz using millis() timer (no blocking delays)  
**Serial Output:** CSV format — time, pitch, roll, pitchCmd, rollCmd  
**Baud Rate:** 115200 for clean data logging to MATLAB  
**Filter Constant:** α = 0.96 (tuned for balance between noise and drift)

### MATLAB Model
The servo is approximated as a first-order transfer function:
```
G(s) = 1 / (0.1s + 1)
```
Simulation generates step response, Bode plot, root locus, and PD vs PID comparison. Hardware data is overlaid on simulation results to quantify the sim-to-real gap.

## Skills Demonstrated

- **Control Systems** - PID design, tuning, and stability analysis
- **Sensor Fusion** - Complementary filter for IMU data
- **Embedded C++** - Non-blocking real-time loop on Arduino
- **MATLAB/Simulink** - Transfer function modeling and frequency domain analysis
- **Hardware Debugging** - Power isolation, servo calibration, axis decoupling
- **Data Analysis** - Live serial logging and MSE-based performance comparison

## Quick Setup

1. Install Arduino IDE and required libraries — MPU6050 (Electronic Cats), PID_v1 (Brett Beauregard)
2. Wire MPU6050 to A4/A5, servos to D9/D10, power servos from external 5V
3. Connect buck converter GND to Arduino GND
4. Upload `gimbal_main.ino` and open Serial Plotter to verify pitch and roll angles
5. Run `matlab/simulation.m` for transfer function graphs
6. Run `matlab/data_logging.m` to capture live hardware data

## Troubleshooting

| Problem | Cause | Fix |
|---|---|---|
| Servo spinning non-stop | 360 degree continuous servo | Replace with standard positional SG90 |
| Roll axis jittery | Servos from different production batches | Match servos by internal pot color |
| Garbage angle values | Buck converter GND floating | Connect all GNDs together |
| System unstable on startup | MPU6050 not settled | Keep gimbal still for 2 seconds after power on |

---

**Built for:** Control Systems Engineering Project  
**Status:** Functional Prototype  
**References:** IEEE IC4 — Camera Gimbal Stabilization Using Conventional PID Controller
