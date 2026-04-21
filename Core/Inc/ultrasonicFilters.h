#ifndef ULTRASONIC_FILTERS_H
#define ULTRASONIC_FILTERS_H

#include "math.h"
#include <string.h>

void init_ultrasonic_filters();
float apply_filters(uint8_t sensor, float value);
float reject_outlier(float d, float d_prev, float threshold);
float median_filter(const float *d_buffer);
float low_pass(float d_prev, float d, float alpha);
float corrected_height(float distance_mm, float roll_rad, float pitch_rad);
float deg_to_rad(float deg);

#endif