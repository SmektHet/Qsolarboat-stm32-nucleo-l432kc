#include "imu.h"
#include <math.h>

int IMU_Parse(uint8_t *rx, uint16_t len, IMU_Data *out)
{
    if(len < 29) return -1;
    if(rx[0] != 0x50 || rx[1] != 0x03) return -2;
    if(rx[2] != 24) return -3;

    int16_t raw[12];
    uint8_t *data = &rx[3];

    for(int i = 0; i < 12; i++)
    {
        raw[i] = (int16_t)((data[2*i] << 8) | data[2*i + 1]);
    }

    for(int i = 0; i < 3; i++)
    {
        out->acc[i]   = raw[i]     / 32768.0f * 16.0f;
        out->gyro[i]  = raw[i+3]   / 32768.0f * 2000.0f;
        out->mag[i]   = raw[i+6];
        out->angle[i] = raw[i+9]   / 32768.0f * 180.0f;
    }

    return 0;
}

float deg_to_rad(float deg) {
    return deg * (M_PI / 180.0f);
}

float corrected_height(float distance_mm, float roll_rad, float pitch_rad)
{
    float cz = cosf(pitch_rad) * cosf(roll_rad);
    return (distance_mm * cz) / 10.0f;
}

float foil_angle(float h_cm)
{
    h_cm = h_cm / 9.0f;

    if (h_cm <= 0.0f) return 0.0f;
    if (h_cm >= 68.0f) return 10.0f;

    return h_cm;
}