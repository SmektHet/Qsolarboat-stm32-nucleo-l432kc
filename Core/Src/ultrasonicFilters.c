#include "ultrasonicFilters.h"
#include <stdint.h>

#define MEDIAN_SIZE 5 // Buffer for median filter keep it odd for a clear median
#define OUTLIERTHRESHOLD 20 //!todo decide on this value
#define LOWPASSALPHA 0.2

float sensorData[2][MEDIAN_SIZE-1];

float sensorDataFiltered[2] = {20.8, 50.7};

void init_ultrasonic_filters(){
    for(int i = 0; i < MEDIAN_SIZE-1; i++){
        sensorData[0][i] = 0;
        sensorData[1][i] = 0;
    }
}

float apply_filters(uint8_t sensor, float value){
    float x = 0;

    //add values to the buffer
    for (int i = MEDIAN_SIZE - 1; i > 0; i--)
    {
        sensorData[sensor][i] = sensorData[sensor][i - 1];
    }
    sensorData[sensor][0] = value;

    x = reject_outlier(sensorData[sensor][0], sensorData[sensor][1]);
    x = median_filter(&x);

    sensorDataFiltered[sensor] = x;

    x = low_pass(sensorDataFiltered[sensor],  sensorDataFiltered[sensor]);
    
    return x;
}
// Simple outlier rejection filter. If the new value deviates from the previous by more than a threshold, reject it.
float reject_outlier(float d, float d_prev) {
    if (fabs(d - d_prev) > OUTLIERTHRESHOLD) {
        return d_prev;
    }
    return d;
}

// Simple median filter. Maintains a buffer of the last N values and returns the median.
float median_filter(float *d_buffer) {
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
float low_pass(float d, float d_prev) {
    return LOWPASSALPHA * d + (1.0f - LOWPASSALPHA) * d_prev;
}

float corrected_height(float distance_mm, float roll_rad, float pitch_rad)
{
    float cz = cosf(pitch_rad) * cosf(roll_rad);
    return (distance_mm * cz) / 10.0f;
}

float deg_to_rad(float deg) {
    return deg * (M_PI / 180.0f);
}