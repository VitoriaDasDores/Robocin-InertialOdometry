#include <stdio.h>
#include <mbed.h>
#include "MPU6050.h"
#include <cmath>

#define SDA_PIN PB_9 // Pino SDA do I2C
#define SCL_PIN PB_8 // Pino SCL do I2C

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

// Função auxiliar para ler e combinar bytes, aplicando o offset
void read_sensor_data(MPU6050& mpu, char reg_start, int16_t* raw_data, const int16_t* offsets) {
    char buffer[6];
    mpu.multiByteRead(reg_start, buffer, 6);  // Lê 6 bytes 

    for (int i = 0; i < 3; i++) {// Loop para os 3 eixos (X, Y, Z)
        // Combina os bytes LSB e MSB e subtrai o offset de calibração
        raw_data[i] = (int16_t)(buffer[i * 2] << 8 | buffer[i * 2 + 1]) - offsets[i];
    }
}

// Função para calibrar o giroscópio
void calibrate_gyro(MPU6050& mpu, int16_t* gyro_offsets) {
    int16_t gyro_total[3] = {0, 0, 0}; // Inicializa um array para acumular as leituras do giroscópio de cada eixo

    printf("Calibrando Giroscópio... Mantenha o robô parado.\n");

    for (int i = 0; i < 200; i++) { // Coleta 200 amostras
        int16_t gyro_raw[3]; // Array para armazenar as leituras brutas do giroscópio em cada eixo

        mpu.readGyroRaw(gyro_raw);

        for (int j = 0; j < 3; j++) { // Loop para cada eixo (X, Y e Z)
            gyro_total[j] += gyro_raw[j]; // Soma a leitura atual do eixo j ao total acumulado
        }
        ThisThread::sleep_for(2ms);
    }

    for (int i = 0; i < 3; i++) { // Loop para calcular o offset de cada eixo
        gyro_offsets[i] = gyro_total[i] / 200;// Calcula a média das 200 leituras para o eixo i - offset
    }

    printf("Calibração do Giroscópio concluída. Offsets: %d, %d, %d\n", gyro_offsets[0], gyro_offsets[1], gyro_offsets[2]);
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

// Função para limitar um ângulo entre -PI e +PI
double constrainAngle(double angle) {
    if (angle > M_PI) {
        angle -= 2.0 * M_PI;
    } else if (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}


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

    // Configurações do sensor
    const int sample_rate = 1; // 500Hz
     // Configura a taxa de amostragem do sensor
    mpu.setSampleRate(sample_rate); // 500Hz
    mpu.setPowerCtl_1(0x00, 0x00, 0x00, 0x00, PLL_GYROX_REF); // Configura o registrador de controle de energia 1
    mpu.setGyroConfig(GYRO_ST_OFF, GFS_1000dps);  // Configura o giroscópio - escala de ±1000°/s
    mpu.setAccelConfig(ACC_ST_OFF, AFS_8g); // Configura o acelerômetro - escala de ±8g

    // Inicializa as estruturas
    GyroData gyro = {};
    AccelData accel = {};

    // Arrays para armazenar os offsets
    int16_t gyro_offsets[3] = {0, 0, 0};
    int16_t accel_offsets[3] = {0, 0, 0};

    //chamando as funções para calibração
    calibrate_gyro(mpu, gyro_offsets);
    calibrate_accel(mpu, accel_offsets);

    double delta_t = 1.0 / (1000.0/(1+sample_rate)); // Intervalo de tempo (2ms para 500Hz) - Ele se ajusta se mudar a taxa de amostragem
    double alpha = 0.98; // Constante do filtro complementar - da mais peso ao giroscopio
    double filtered_angle[3] = {0.0, 0.0, 0.0}; // Ângulos filtrados (em radianos)

    while(true) {

        // Buffers para armazenar os dados brutos
        char gyro_buffer[6];
        char accel_buffer[6];

        // Lê os dados brutos do giroscópio e do acelerômetro
        read_sensor_data(mpu, GYRO_XOUT_H_REG, gyro.raw, gyro_offsets);
        read_sensor_data(mpu, ACCEL_XOUT_H_REG, accel.raw, accel_offsets);

        for (int i = 0; i < 3; i++) {
            // Converte a leitura bruta do giroscópio para rad/s (velocidade angular)
            gyro.rad_per_s[i] = (gyro.raw[i] / 32.8) * M_PI / 180.0; // Escala de 1000°/s
            gyro.angle_rad[i] += gyro.rad_per_s[i] * delta_t; // Integra a velocidade angular para obter o ângulo
            gyro.angle_rad[i] = constrainAngle(gyro.angle_rad[i]); // Limita o ângulo entre -PI e +PI
            accel.g[i] = accel.raw[i] /  4096; // // Converte a leitura bruta do acelerômetro para g's - Escala de 8g
            accel.angle_rad[i] = atan2(accel.g[(i + 1) % 3], accel.g[(i + 2) % 3]); // Calcula o ângulo de inclinação com atan2

            // Filtro complementar  
            filtered_angle[i] = alpha * (filtered_angle[i] + gyro.rad_per_s[i] * delta_t) + (1 - alpha) * accel.angle_rad[i];
        }

        // Imprime os dados no console
        printf("Giroscópio (rad/s): X=%.4f, Y=%.4f, Z=%.4f\n", gyro.rad_per_s[0], gyro.rad_per_s[1], gyro.rad_per_s[2]);
        printf("Acelerômetro (g): X=%.4f, Y=%.4f, Z=%.4f\n", accel.g[0], accel.g[1], accel.g[2]);
        printf("Ângulo do Giroscópio (rad): X=%.4f, Y=%.4f, Z=%.4f\n", gyro.angle_rad[0], gyro.angle_rad[1], gyro.angle_rad[2]);
        printf("Ângulo do Acelerômetro (rad): X=%.4f, Y=%.4f, Z=%.4f\n", accel.angle_rad[0], accel.angle_rad[1], accel.angle_rad[2]);
        printf("Ângulo Filtrado (rad): X=%.4f, Y=%.4f, Z=%.4f\n\n", filtered_angle[0], filtered_angle[1], filtered_angle[2]);

        ThisThread::sleep_for(2ms); // Aguarda 2ms - 500Hz
        
    }  
    
    return 0;
}
