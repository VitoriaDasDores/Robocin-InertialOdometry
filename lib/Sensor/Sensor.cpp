#include "sensor.h"
#include "math_utils.h"
#include <stdio.h>

// Configurações do sensor
void configure_sensor(MPU6050 &mpu) {
     // Configura a taxa de amostragem do sensor
    mpu.setSampleRate(1); // 500Hz
    mpu.setPowerCtl_1(0x00, 0x00, 0x00, 0x00, PLL_GYROX_REF); // Configura o registrador de controle de energia 1
    mpu.setGyroConfig(GYRO_ST_OFF, GFS_1000dps);  // Configura o giroscópio - escala de ±1000°/s
    mpu.setAccelConfig(ACC_ST_OFF, AFS_8g); // Configura o acelerômetro - escala de ±8g
}

// Função para calibrar o giroscópio com análise de desvio padrão
void calibrate_gyro(MPU6050& mpu, int16_t* gyro_offsets) {
    const int samples = 200;            // Número de amostras para calibração
    const double threshold = 2.0;       // Limite de desvio padrão (em σ)
    int16_t gyro_readings[samples][3];  // Armazena as leituras brutas
    int valid_samples[3] = {0, 0, 0};   // Contagem de amostras válidas
    double sum[3] = {0.0, 0.0, 0.0};   // Soma acumulada para cada eixo
    double mean[3] = {0.0, 0.0, 0.0};   // Média para cada eixo
    double stddev[3] = {0.0, 0.0, 0.0}; // Desvio padrão para cada eixo

    printf("Calibrando Giroscópio... Mantenha o robô parado.\n");

    // Coleta das leituras
    for (int i = 0; i < samples; i++) {
        int16_t gyro_raw[3]; // Leituras brutas para os três eixos
        mpu.readGyroRaw(gyro_raw);

        for (int j = 0; j < 3; j++) {
            gyro_readings[i][j] = gyro_raw[j];
            sum[j] += gyro_raw[j];
        }
        ThisThread::sleep_for(2ms);
    }

    // Cálculo inicial das médias
    for (int j = 0; j < 3; j++) {
        mean[j] = sum[j] / samples;
    }

    // Cálculo do desvio padrão
    for (int i = 0; i < samples; i++) {
        for (int j = 0; j < 3; j++) {
            stddev[j] += (gyro_readings[i][j] - mean[j]) * (gyro_readings[i][j] - mean[j]);
        }
    }
    for (int j = 0; j < 3; j++) {
        stddev[j] = sqrt(stddev[j] / samples);
    }

    // Recalcula a média, descartando outliers
    for (int i = 0; i < samples; i++) {
        for (int j = 0; j < 3; j++) {
            if (fabs(gyro_readings[i][j] - mean[j]) <= threshold * stddev[j]) {
                sum[j] += gyro_readings[i][j];
                valid_samples[j]++;
            }
        }
    }

    for (int j = 0; j < 3; j++) {
        gyro_offsets[j] = valid_samples[j] > 0 ? sum[j] / valid_samples[j] : 0; // Calcula a média ajustada
    }

    printf("Calibração do Giroscópio concluída.\n");
    printf("Offsets: X=%d, Y=%d, Z=%d\n", gyro_offsets[0], gyro_offsets[1], gyro_offsets[2]);
}

// Função para calibrar o acelerômetro
void calibrate_accel(MPU6050& mpu, int16_t* accel_offsets) {
    int16_t accel_total[3] = {0, 0, 0}; // Inicializa um array para acumular as leituras do acelerômetro de cada eixo

    printf("Calibrando Acelerômetro... Mantenha o robô parado.\n");

    for (int i = 0; i < 200; i++) {
        int16_t accel_raw[3];

        char accel_buffer[6]; // Buffer para armazenar os bytes lidos do acelerômetro

        mpu.multiByteRead(ACCEL_XOUT_H_REG, accel_buffer, 6); // Lê os 6 bytes - 3 eixos * 2 bytes/eixo

        for (int i = 0; i < 3; i++) {
            // Combina os bytes LSB e MSB para formar um valor de 16 bits
            accel_raw[i] = (int16_t)(accel_buffer[i * 2] << 8 | accel_buffer[i * 2 + 1]);
        }

        for (int j = 0; j < 3; j++) {
            accel_total[j] += accel_raw[j]; // Soma a leitura atual do eixo j ao total acumulado
        }
        ThisThread::sleep_for(5ms);
    }

    for (int i = 0; i < 3; i++) {
        accel_offsets[i] = accel_total[i] / 200;
    }

   
    accel_offsets[2] -= 4096; // Ajusta o offset do eixo Z para considerar 1g na escala de ±8g 

    printf("Calibração do Acelerômetro concluída. Offsets: %d, %d, %d\n", accel_offsets[0], accel_offsets[1], accel_offsets[2]);
}

// Função auxiliar para ler e combinar bytes, aplicando o offset
void read_sensor_data(MPU6050& mpu, char reg_start, int16_t* raw_data, const int16_t* offsets) {
    char buffer[6];
    mpu.multiByteRead(reg_start, buffer, 6);  // Lê 6 bytes 

    for (int i = 0; i < 3; i++) {// Loop para os 3 eixos (X, Y, Z)
        // Combina os bytes LSB e MSB e subtrai o offset de calibração
        raw_data[i] = (int16_t)(buffer[i * 2] << 8 | buffer[i * 2 + 1]) - offsets[i];
    }
}

void process_sensor_data(GyroData &gyro, AccelData &accel, double *filtered_rad_per_s, double alpha, double delta_t) {
    for (int i = 0; i < 3; i++) {
            // Converte a leitura bruta do giroscópio para rad/s (velocidade angular)
            gyro.rad_per_s[i] = (gyro.raw[i] / 32.8) * M_PI / 180.0; // Escala de ±1000°/s
    
            // Aplica o filtro passa-baixa
            filtered_rad_per_s[i] = filter_low_pass(gyro.rad_per_s[i], filtered_rad_per_s[i], alpha);
    
            // Integra a velocidade angular filtrada para obter o ângulo
            gyro.angle_rad[i] += filtered_rad_per_s[i] * delta_t;
            gyro.angle_rad[i] = constrainAngle(gyro.angle_rad[i]); // Limita o ângulo entre -π e π
    
            // Converte a leitura bruta do acelerômetro para g's
            accel.g[i] = accel.raw[i] / 4096.0; // Escala de ±8g
            accel.angle_rad[i] = atan2(accel.g[(i + 1) % 3], accel.g[(i + 2) % 3]); // Inclinação
    }
}

void print_sensor_data(const GyroData &gyro, const AccelData &accel, const double *filtered_rad_per_s) {
    
    printf("Giroscópio (rad/s): X=%.4f, Y=%.4f, Z=%.4f\n", gyro.rad_per_s[0], gyro.rad_per_s[1], gyro.rad_per_s[2]);
    printf("Ângulo do Giroscópio (rad): X=%.4f, Y=%.4f, Z=%.4f\n", gyro.angle_rad[0], gyro.angle_rad[1], gyro.angle_rad[2]);
    printf("Velocidade Angular Filtrada (rad/s): X=%.4f, Y=%.4f, Z=%.4f\n", filtered_rad_per_s[0], filtered_rad_per_s[1], filtered_rad_per_s[2]);
    printf("Acelerômetro (g): X=%.4f, Y=%.4f, Z=%.4f\n", accel.g[0], accel.g[1], accel.g[2]);
    printf("Ângulo do Acelerômetro (rad): X=%.4f, Y=%.4f, Z=%.4f\n", accel.angle_rad[0], accel.angle_rad[1], accel.angle_rad[2]);
}