#ifndef MADGWICK_AHRS_H
#define MADGWICK_AHRS_H

#include <math.h>

typedef struct {
    float beta;
    float q0, q1, q2, q3;
    float invSampleFreq;
    float roll, pitch, yaw;
    char anglesComputed;
} Madgwick;

// Init
void Madgwick_init(Madgwick *m, float sampleFrequency);

// Update
void Madgwick_update(Madgwick *m, float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz);

void Madgwick_updateIMU(Madgwick *m, float gx, float gy, float gz,
                        float ax, float ay, float az);

// Get angles (degrees)
float Madgwick_getRoll(Madgwick *m);
float Madgwick_getPitch(Madgwick *m);
float Madgwick_getYaw(Madgwick *m);

#endif