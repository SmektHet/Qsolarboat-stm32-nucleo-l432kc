#include "ultrasonicFilters.h"

#define MEDIAN_SIZE 5 // Buffer for median filter keep it odd for a clear median


// Simple outlier rejection filter. If the new value deviates from the previous by more than a threshold, reject it.
float reject_outlier(float d, float d_prev, float threshold) {
    if (fabs(d - d_prev) > threshold) {
        return d_prev;
    }
    return d;
}

// Simple median filter. Maintains a buffer of the last N values and returns the median.
float median_filter(const float *d_buffer) {
    float sorted[MEDIAN_SIZE];
    memcpy(sorted, d_buffer, MEDIAN_SIZE * sizeof(float));

    // bubble sort the sorted array to find the median
    for (int i = 0; i < MEDIAN_SIZE-1; i++) {
        for (int j = 0; j < MEDIAN_SIZE-i-1; j++) {
            if (sorted[j] > sorted[j+1]) {
                float swap_value  = sorted[j];
                sorted[j] = sorted[j+1];
                sorted[j+1] = swap_value ;
            }
        }
    }
    return sorted[MEDIAN_SIZE/2];
}

// Simple low-pass filter, for smoothing out noise. Alpha is the smoothing factor (0 < alpha < 1).
// alpha  0.2 - 0.3 is a common choice for moderate smoothing.
float low_pass(float d, float d_prev, float alpha) {
    return alpha * d + (1.0f - alpha) * d_prev;
}

float tilt_correction(){}


float deg_to_rad(float deg) {
    return deg * (M_PI / 180.0f);
}

float corrected_height(float distance_mm, float roll_rad, float pitch_rad)
{
    float cz = cosf(pitch_rad) * cosf(roll_rad);
    return (distance_mm * cz) / 10.0f;
}