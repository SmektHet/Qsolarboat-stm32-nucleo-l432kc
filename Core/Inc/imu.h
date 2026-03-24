#ifndef IMU_H
#define IMU_H

#include <stdint.h>

typedef struct {
    float acc[3];
    float gyro[3];
    float mag[3];
    float angle[3];
    float distance;
} IMU_Data;

int IMU_Parse(uint8_t *rx, uint16_t len, IMU_Data *out);

float deg_to_rad(float deg);
float corrected_height(float distance_mm, float roll_rad, float pitch_rad);
float foil_angle(float h_cm);

#endif