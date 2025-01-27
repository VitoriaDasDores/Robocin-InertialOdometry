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
  
## Validation
https://docs.google.com/spreadsheets/d/15UAhusqmTekeYFRcJhxQUaSOYU4pnZHtHsBnsit3GCg/edit?usp=sharing

![image](https://github.com/user-attachments/assets/b594a9bb-f42f-4637-97de-07268021af4a)

![image](https://github.com/user-attachments/assets/847c9f12-ddbc-4b8f-a471-b26979c1e35e)

![image](https://github.com/user-attachments/assets/29846a3c-2f96-409b-a500-5d9ebb190ef9)


## Development

Mbed OS was chosen for its hardware abstraction and team compatibility, after initial work with STM32CubeIDE.  A custom MPU6050 library (https://os.mbed.com/users/213468891/code/MPU6050/docs/tip/classMPU6050.html) was used.

## References

https://os.mbed.com/users/213468891/code/MPU6050/docs/tip/classMPU6050.html	
https://embarcados.com.br/calibracao-de-sensores-na-pratica/
https://www2.decom.ufop.br/imobilis/sensores-imu-uma-abordagem-completa-parte-2/		
https://calculareconverter.com.br/graus-para-radianos/	
https://github.com/robocin/ssl-firmware/tree/ce19eda3d749f0ade35c57139934eda18b57cb55	
https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf	
https://docs.platformio.org/en/latest/frameworks/mbed.html	
https://www.st.com/en/microcontrollers-microprocessors/stm32f767zi.html	


A Google Sheet was employed to simulate data processing and validate the math in the absence of the physical sensor.

## Dependencies

- Mbed OS
- MPU6050 Library (included in `lib`)
