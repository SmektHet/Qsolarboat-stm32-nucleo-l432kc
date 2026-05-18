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
typedef struct {
    float global_yaw;
    float global_pitch;
    float global_roll;

    float foil_left_angle;   // degrees tilt on left side
    float foil_right_angle;  // degrees tilt on right side
} FusedOrientation;

int IMU_Parse(uint8_t *rx, uint16_t len, IMU_Data *out);
float foil_angle(float h_cm);
void fuse_orientation (IMU_Data *imu_fl, IMU_Data *imu_mid, IMU_Data *imu_fr, FusedOrientation *out);
float fuse_roll(float roll_imu, float roll_ultra); 

#endif