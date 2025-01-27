#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <cmath>
#include <cstdint>

double constrainAngle(double angle);
double filter_low_pass(double current_value, double previous_value, double alpha);
double calculate_stddev(const int16_t* data, int size, double mean);

#endif 