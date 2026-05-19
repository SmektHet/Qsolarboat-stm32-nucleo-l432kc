#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// #define DEBUG_PRINTS // Comment out to disable debug prints over UART2
// #define USE_SINGLE_IMU // Comment out to use all three IMUs, otherwise only the middle one will be used

#define SENSOR_COUNT 5
static const uint8_t sensor_addresses[SENSOR_COUNT] = {
    0x50,  // IMU main
    0x01,  // Ultrasonic left
    0x51,  // IMU left
    0x02,  // Ultrasonic right
    0x52   // IMU right
};

// typedef struct {
//     uint8_t imu_addr;
//     uint8_t us_addr;
// } SensorPair;

// #define PAIR_COUNT 3
// static const SensorPair pairs[] = {
//     { 0x50, 0x01 },  // IMU1 + US1
//     { 0x51, 0x02 },  // IMU2 + US2
//     { 0x52, 0x00 },  // IMU3 alone
// };


#define IMU_TIMEOUT_MS 20
#define ULTRASONIC_TIMEOUT_MS 50

#define UART_RX_BUFFER_SIZE 128
#define ACCUM_BUFFER_SIZE 256

#endif