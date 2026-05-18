#include "ultrasonicFilters.h"

#define MEDIAN_SIZE 5 // Buffer for median filter keep it odd for a clear median
#define OUTLIERTHRESHOLD 20 //!todo decide on this value
#define LOWPASSALPHA 0.2 //!todo decide on this value

float sensorData[2][MEDIAN_SIZE-1];
float sensorDataFiltered[2];

/**
 * \brief Initializes the ultrasonic filters by setting sensor data buffers to zero.
 */
void init_ultrasonic_filters(){
    for(int i = 0; i < MEDIAN_SIZE-1; i++){
        sensorData[0][i] = 0;
        sensorData[1][i] = 0;
    }
}

/**
* \brief Applies a series of filters (outlier rejection, median, low-pass) to the raw sensor value.
 * \param sensor The index of the sensor (0 or 1).
 * \param value The raw distance measurement from the ultrasonic sensor.
 * \return The filtered distance value.
*/
float apply_filters(int sensor, float value){
    float x = 0;

    for (int i = MEDIAN_SIZE - 1; i > 0; i--)
    {
        sensorData[sensor][i] = sensorData[sensor][i - 1];
    }
    sensorData[sensor][0] = value;

    x = reject_outlier(sensorData[sensor][0], sensorData[sensor][1], OUTLIERTHRESHOLD);
    x = median_filter(&x);

    sensorDataFiltered[sensor] = x;

    x = low_pass(sensorDataFiltered[sensor],  sensorDataFiltered[sensor], LOWPASSALPHA);
    
    return x;
}

/**
* \brief Rejects outliers in the distance measurements by comparing the new value to the previous one.
 * \param d The new distance measurement.
 * \param d_prev The previous distance measurement.
 * \param threshold The maximum allowed deviation before rejecting the new value.
 * \return The accepted distance value (either the new one or the previous one if rejected).
*/
float reject_outlier(float d, float d_prev, float threshold) {
    if (fabs(d - d_prev) > threshold) {
        return d_prev;
    }
    return d;
}

/**
 * \brief Applies a median filter to smooth out noise by returning the median of the last N values.
 * \param d_buffer Pointer to the buffer of distance measurements.
 * \return The median value of the buffer.
 */
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

/**
 * \brief Applies a low-pass filter to smooth out high-frequency noise in the distance measurements.
 * \param d The current distance measurement.
 * \param d_prev The previous filtered distance measurement.
 * \param alpha The smoothing factor (0 < alpha < 1), typically 0.2-0.3.
 * \return The filtered distance value.
 */
float low_pass(float d, float d_prev, float alpha) {
    return alpha * d + (1.0f - alpha) * d_prev;
}

/**
 * \brief Corrects the raw distance measurement from the ultrasonic sensor based on the roll and pitch angles of the IMU sensor.
 * \param distance_mm The raw distance measurement in millimeters.
 * \param roll_rad The roll angle in radians.
 * \param pitch_rad The pitch angle in radians.
 * \return The corrected height in centimeters.
 */
float corrected_height(float distance_mm, float roll_rad, float pitch_rad)
{
    float cz = cosf(pitch_rad) * cosf(roll_rad);
    return (distance_mm * cz) / 10.0f;
}

/**
 * \brief Converts an angle from degrees to radians.
 * \param deg The angle in degrees.
 * \return The angle in radians.
 */
float deg_to_rad(float deg) {
    return deg * (M_PI / 180.0f);
}

/**
 * \brief Calculates the roll angle from the heights measured by left and right ultrasonic sensors.
 * \param hL The height measured by the left sensor.
 * \param hR The height measured by the right sensor.
 * \param sensor_distance The distance between the two sensors.
 * \return The roll angle in radians.
 */
float roll_ultra(float hL, float hR, float sensor_distance) {
    return atan2f((hR - hL), sensor_distance);
}