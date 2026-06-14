#include "ultrasonicFilters.h"

float sensorData[2][MEDIAN_SIZE];
uint8_t sensorInitialized[2] = {0};
float sensorDataFiltered[2];

/**
 * \brief Initializes the ultrasonic filters by setting sensor data buffers to zero.
 */
void init_ultrasonic_filters()
{
    for(int s = 0; s < 2; s++)
    {
        for(int i = 0; i < MEDIAN_SIZE; i++)
        {
            sensorData[s][i] = 0.0f;
        }

        sensorDataFiltered[s] = 0.0f;
    }
}

/**
* \brief Applies a series of filters (outlier rejection, median, low-pass) to the raw sensor value.
 * \param sensor The index of the sensor (0 or 1).
 * \param value The raw distance measurement from the ultrasonic sensor.
 * \return The filtered distance value.
*/
float apply_filters(int sensor, float value)
{
    float x;

    // First sample initialization
    if (!sensorInitialized[sensor])
    {
        for (int i = 0; i < MEDIAN_SIZE; i++)
        {
            sensorData[sensor][i] = value;
        }

        sensorDataFiltered[sensor] = value;

        sensorInitialized[sensor] = 1;

        return value;
    }

    // Shift old samples
    for (int i = MEDIAN_SIZE - 1; i > 0; i--)
    {
        sensorData[sensor][i] =
            sensorData[sensor][i - 1];
    }

    // Insert newest sample
    sensorData[sensor][0] = value;

    // Reject outliers
    x = reject_outlier(sensorData[sensor][0],
                       sensorData[sensor][1],
                       OUTLIERTHRESHOLD);

    sensorData[sensor][0] = x;

    // Median filter
    x = median_filter(sensorData[sensor]);

    // Low-pass filter
    x = low_pass(x,
                 sensorDataFiltered[sensor],
                 LOWPASSALPHA);

    sensorDataFiltered[sensor] = x;

    return x;
}

/**
* \brief Rejects outliers in the distance measurements by comparing the new value to the previous one.
 * \param d The new distance measurement.
 * \param d_prev The previous distance measurement.
 * \param threshold The maximum allowed deviation before rejecting the new value.
 * \return The accepted distance value (either the new one or the previous one if rejected).
*/
float reject_outlier(float d,
                     float d_prev,
                     float threshold)
{
    if (fabsf(d - d_prev) > threshold)
    {
        return d_prev;
    }

    return d;
}

/**
 * \brief Applies a median filter to smooth out noise by returning the median of the last N values.
 * \param d_buffer Pointer to the buffer of distance measurements.
 * \return The median value of the buffer.
 */
float median_filter(const float *d_buffer)
{
    float sorted[MEDIAN_SIZE];

    memcpy(sorted,
           d_buffer,
           sizeof(sorted));

    // Bubble sort
    for (int i = 0; i < MEDIAN_SIZE - 1; i++)
    {
        for (int j = 0; j < MEDIAN_SIZE - i - 1; j++)
        {
            if (sorted[j] > sorted[j + 1])
            {
                float swap_value = sorted[j];

                sorted[j]     = sorted[j + 1];
                sorted[j + 1] = swap_value;
            }
        }
    }

    // Return middle value
    return sorted[MEDIAN_SIZE / 2];
}

/**
 * \brief Applies a low-pass filter to smooth out high-frequency noise in the distance measurements.
 * \param d The current distance measurement.
 * \param d_prev The previous filtered distance measurement.
 * \param alpha The smoothing factor (0 < alpha < 1), typically 0.2-0.3.
 * \return The filtered distance value.
 */
float low_pass(float d, float d_prev, float alpha)
{
    return alpha * d + (1.0f - alpha) * d_prev;
}

/**
 * \brief Corrects the raw distance measurement from the ultrasonic sensor based on the roll and pitch angles of the IMU sensor.
 * \param distance_mm The raw distance measurement in millimeters.
 * \param roll The roll angle in radians.
 * \param pitch The pitch angle in radians.
 * \return The corrected height in centimeters.
 */
float corrected_height(float distance_mm, float roll, float pitch)
{
    float cz = cosf(deg_to_rad(pitch)) * cosf(deg_to_rad(roll));
    return (distance_mm * cz) / 10.0f;
}

/**
 * \brief Converts an angle from degrees to radians.
 * \param deg The angle in degrees.
 * \return The angle in radians.
 */
float deg_to_rad(float deg)
{
    return deg * (M_PI / 180.0f);
}

/**
 * \brief Calculates the roll angle from the heights measured by left and right ultrasonic sensors.
 * \param hL The height measured by the left sensor.
 * \param hR The height measured by the right sensor.
 * \param sensor_distance The distance between the two sensors.
 * \return The roll angle in radians.
 */
float roll_ultra(float hL, float hR, float sensor_distance)
{
    float result = atan2f((hR - hL), sensor_distance);
    return result * (180.0f / M_PI);
}