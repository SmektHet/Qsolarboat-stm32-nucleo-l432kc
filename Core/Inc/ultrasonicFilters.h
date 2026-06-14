#ifndef ULTRASONIC_FILTERS_H
#define ULTRASONIC_FILTERS_H

#include "math.h"
#include <string.h>
#include <stdint.h>
#include "config.h"

void init_ultrasonic_filters();
float apply_filters(int sensor, float value);
float reject_outlier(float d, float d_prev, float threshold);
float median_filter(const float *d_buffer);
float low_pass(float d_prev, float d, float alpha);
float corrected_height(float distance_mm, float roll, float pitch);
float deg_to_rad(float deg);
float roll_ultra(float hL, float hR, float sensor_distance);

#endif