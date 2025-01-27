#include "math_utils.h"

// Função para limitar um ângulo entre -PI e +PI
double constrainAngle(double angle) {
    if (angle > M_PI) {
        angle -= 2.0 * M_PI;
    } else if (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

// Função para aplicar filtro passa-baixa
double filter_low_pass(double current_value, double previous_value, double alpha) {
    return alpha * current_value + (1 - alpha) * previous_value;
}

// Função para calcular o desvio padrão
double calculate_stddev(const int16_t* data, int size, double mean) {
    double variance = 0.0;
    for (int i = 0; i < size; i++) {
        variance += (data[i] - mean) * (data[i] - mean);
    }
    return sqrt(variance / size);
}
