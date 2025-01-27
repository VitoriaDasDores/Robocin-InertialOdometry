#ifndef SENSOR_UTILS_H
#define SENSOR_UTILS_H

#include "MPU6050.h"
#include "gyro_accel_data.h"

void configure_sensor(MPU6050 &mpu);
void calibrate_gyro(MPU6050 &mpu, int16_t *gyro_offsets);
void calibrate_accel(MPU6050 &mpu, int16_t *accel_offsets);
void read_sensor_data(MPU6050& mpu, char reg_start, int16_t* raw_data, const int16_t* offsets);
void process_sensor_data(GyroData &gyro, AccelData &accel, double *filtered_angle, double alpha, double delta_t);
void print_sensor_data(const GyroData &gyro, const AccelData &accel, const double *filtered_angle);

#endif // SENSOR_UTILS_H