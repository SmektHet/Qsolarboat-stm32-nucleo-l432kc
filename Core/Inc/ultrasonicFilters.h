#ifndef ULTRASONIC_FILTERS_H
#define ULTRASONIC_FILTERS_H

#include "math.h"
#include <string.h>

float reject_outlier(float d, float d_prev, float threshold);
float median_filter(const float *d_buffer);
float low_pass(float d_prev, float d, float alpha);

#endif