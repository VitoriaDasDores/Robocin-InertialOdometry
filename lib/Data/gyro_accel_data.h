#ifndef GYRO_ACCEL_DATA_H
#define GYRO_ACCEL_DATA_H

#include <cstdint>

// Estruturas para armazenar dados do giroscópio e acelerômetro
struct GyroData {
    int16_t raw[3] = {0, 0, 0};
    double rad_per_s[3] = {0.0, 0.0, 0.0};
    double angle_rad[3] = {0.0, 0.0, 0.0}; 
};

struct AccelData {
    int16_t raw[3] = {0, 0, 0};
    double g[3] = {0.0, 0.0, 0.0};
    double angle_rad[3] = {0.0, 0.0, 0.0}; 
};

#endif