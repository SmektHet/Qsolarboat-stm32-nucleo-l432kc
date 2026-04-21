#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <math.h>

typedef struct {
    float acc[3];
    float gyro[3];
    float mag[3];
    float angle[3];
} IMU_Data;

int IMU_Parse(uint8_t *rx, uint16_t len, IMU_Data *out);
float foil_angle(float h_cm);

#endif