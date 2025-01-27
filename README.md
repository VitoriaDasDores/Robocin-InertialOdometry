# Robocin-InertialOdometry
# MPU6050 Data Acquisition and Processing for Inertial Odometry

This project acquires and processes data from an MPU6050 gyroscope and accelerometer for inertial odometry on a robot RoboCup's Small Size League.

## Overview

The firmware reads raw MPU6050 data, calibrates the gyroscope, converts the data to rad/s and radians, filters it, and calculates angular velocity and displacement.  Accelerometer calibration is included but not the primary focus.

## Hardware & Software

- **Board:** NUCLEO-F767ZI
- **API:** Mbed OS (latest)
- **IDE:** Visual Studio Code with PlatformIO
- **Sensor:** MPU6050 (I2C)

## Implementation

1. **Acquisition:**  Uses `multiByteRead` for efficient register reading.

2. **Calibration:** Gyroscope calibrated using sample mean and outlier removal (standard deviation analysis). Accelerometer calibration implemented.

3. **Conversion:** Raw data converted to rad/s and radians.

4. **Constraint:** `constrainAngle` keeps angle within -π to π.

5. **Filtering:**  Digital low-pass filter for noise reduction.

6. **Configuration:**
    - Sample Rate: 500Hz (fast movements)
    - Gyro Scale: ±1000°/s
    - Accel Scale: ±8g

## Development

Mbed OS was chosen for its hardware abstraction and team compatibility, after initial work with STM32CubeIDE.  A custom MPU6050 library (adapted from the manufacturer's) was used.

A Google Sheet was employed to simulate data processing and validate the math in the absence of the physical sensor.

## Dependencies

- Mbed OS
- MPU6050 Library (included in `lib`)
