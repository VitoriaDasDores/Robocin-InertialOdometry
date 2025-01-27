#include <stdio.h>
#include <mbed.h>
#include <cmath>
#include <cstdint>
#include "MPU6050.h"
#include "sensor.h"
#include "math_utils.h"
#include "gyro_accel_data.h"

#define SDA_PIN PB_9 // Pino SDA do I2C
#define SCL_PIN PB_8 // Pino SCL do I2C

int main() {
    I2C i2c(SDA_PIN, SCL_PIN);// Inicializa o objeto I2C com os pinos 
    MPU6050 mpu(i2c);// Inicializa o objeto MPU6050 com o objeto I2C

    // Verifica a comunicação com o sensor lendo o registrador WHO_AM_I
    uint8_t WhoAmI = mpu.getWhoAmI();
    if (WhoAmI != 0x68) {
        printf("Erro: MPU6050 não encontrado (WhoAmI = 0x%02X).\n", WhoAmI);
        return 1; 
    } else {
        printf("MPU6050 encontrado (WhoAmI = 0x%02X).\n", WhoAmI);
    }

    // Configuração do sensor
    configure_sensor(mpu);

    // Inicializa as estruturas
    GyroData gyro = {};
    AccelData accel = {};

    // Arrays para armazenar os offsets
    int16_t gyro_offsets[3] = {0, 0, 0};
    int16_t accel_offsets[3] = {0, 0, 0};

    //chamando as funções para calibração
    calibrate_gyro(mpu, gyro_offsets);
    calibrate_accel(mpu, accel_offsets);

    double delta_t = 1.0 / 500.0; // Intervalo de tempo (2ms para 500Hz)
    double alpha = 0.98; // Coeficiente de suavização para o filtro passa-baixa
    double filtered_rad_per_s[3] = {0.0, 0.0, 0.0}; // Velocidades angulares filtradas (em rad/s)

    while(true) {
        // Lê os dados brutos do giroscópio e do acelerômetro
        read_sensor_data(mpu, GYRO_XOUT_H_REG, gyro.raw, gyro_offsets);
        read_sensor_data(mpu, ACCEL_XOUT_H_REG, accel.raw, accel_offsets);

        // Processa os dados do sensor, convertendo e aplicando os filtros
        process_sensor_data(gyro, accel, filtered_rad_per_s, alpha, delta_t);

        // Imprime os dados no console
        print_sensor_data(gyro, accel, filtered_rad_per_s);

        ThisThread::sleep_for(2ms); // Aguarda 2ms - 500Hz
        
    }  
    
    return 0;
}
