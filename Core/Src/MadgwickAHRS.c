#include "MadgwickAHRS.h"

#define sampleFreqDef   400.0f
#define betaDef         0.4f

// Internal functions
static float invSqrt(float x);
static void computeAngles(Madgwick *m);

//============================================================================================
// Init

void Madgwick_init(Madgwick *m, float sampleFrequency) {
    m->beta = betaDef;
    m->q0 = 1.0f;
    m->q1 = 0.0f;
    m->q2 = 0.0f;
    m->q3 = 0.0f;
    m->invSampleFreq = 1.0f / sampleFrequency;
    m->anglesComputed = 0;
}

//============================================================================================
// Update (full AHRS)

void Madgwick_update(Madgwick *m, float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz) {

    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx;
    float _2bx, _2bz, _4bx, _4bz;
    float _2q0, _2q1, _2q2, _2q3;
    float _2q0q2, _2q2q3;
    float q0q0, q0q1, q0q2, q0q3;
    float q1q1, q1q2, q1q3;
    float q2q2, q2q3, q3q3;

    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        Madgwick_updateIMU(m, gx, gy, gz, ax, ay, az);
        return;
    }

    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    qDot1 = 0.5f * (-m->q1 * gx - m->q2 * gy - m->q3 * gz);
    qDot2 = 0.5f * (m->q0 * gx + m->q2 * gz - m->q3 * gy);
    qDot3 = 0.5f * (m->q0 * gy - m->q1 * gz + m->q3 * gx);
    qDot4 = 0.5f * (m->q0 * gz + m->q1 * gy - m->q2 * gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        _2q0mx = 2.0f * m->q0 * mx;
        _2q0my = 2.0f * m->q0 * my;
        _2q0mz = 2.0f * m->q0 * mz;
        _2q1mx = 2.0f * m->q1 * mx;
        _2q0 = 2.0f * m->q0;
        _2q1 = 2.0f * m->q1;
        _2q2 = 2.0f * m->q2;
        _2q3 = 2.0f * m->q3;
        _2q0q2 = 2.0f * m->q0 * m->q2;
        _2q2q3 = 2.0f * m->q2 * m->q3;

        q0q0 = m->q0 * m->q0;
        q0q1 = m->q0 * m->q1;
        q0q2 = m->q0 * m->q2;
        q0q3 = m->q0 * m->q3;
        q1q1 = m->q1 * m->q1;
        q1q2 = m->q1 * m->q2;
        q1q3 = m->q1 * m->q3;
        q2q2 = m->q2 * m->q2;
        q2q3 = m->q2 * m->q3;
        q3q3 = m->q3 * m->q3;

        hx = mx * q0q0 - _2q0my * m->q3 + _2q0mz * m->q2 + mx * q1q1 +
             _2q1 * my * m->q2 + _2q1 * mz * m->q3 -
             mx * q2q2 - mx * q3q3;

        hy = _2q0mx * m->q3 + my * q0q0 - _2q0mz * m->q1 +
             _2q1mx * m->q2 - my * q1q1 + my * q2q2 +
             _2q2 * mz * m->q3 - my * q3q3;

        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * m->q2 + _2q0my * m->q1 + mz * q0q0 +
               _2q1mx * m->q3 - mz * q1q1 +
               _2q2 * my * m->q3 - mz * q2q2 + mz * q3q3;

        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) +
             _2q1 * (2.0f * q0q1 + _2q2q3 - ay) -
             _2bz * m->q2 * (_2bx * (0.5f - q2q2 - q3q3) +
             _2bz * (q1q3 - q0q2) - mx) +
             (-_2bx * m->q3 + _2bz * m->q1) *
             (_2bx * (q1q2 - q0q3) +
             _2bz * (q0q1 + q2q3) - my) +
             _2bx * m->q2 *
             (_2bx * (q0q2 + q1q3) +
             _2bz * (0.5f - q1q1 - q2q2) - mz);

        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) +
             _2q0 * (2.0f * q0q1 + _2q2q3 - ay) -
             4.0f * m->q1 *
             (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
             _2bz * m->q3 *
             (_2bx * (0.5f - q2q2 - q3q3) +
             _2bz * (q1q3 - q0q2) - mx) +
             (_2bx * m->q2 + _2bz * m->q0) *
             (_2bx * (q1q2 - q0q3) +
             _2bz * (q0q1 + q2q3) - my) +
             (_2bx * m->q3 - _4bz * m->q1) *
             (_2bx * (q0q2 + q1q3) +
             _2bz * (0.5f - q1q1 - q2q2) - mz);

        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) +
             _2q3 * (2.0f * q0q1 + _2q2q3 - ay) -
             4.0f * m->q2 *
             (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
             (-_4bx * m->q2 - _2bz * m->q0) *
             (_2bx * (0.5f - q2q2 - q3q3) +
             _2bz * (q1q3 - q0q2) - mx) +
             (_2bx * m->q1 + _2bz * m->q3) *
             (_2bx * (q1q2 - q0q3) +
             _2bz * (q0q1 + q2q3) - my) +
             (_2bx * m->q0 - _4bz * m->q2) *
             (_2bx * (q0q2 + q1q3) +
             _2bz * (0.5f - q1q1 - q2q2) - mz);

        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) +
             _2q2 * (2.0f * q0q1 + _2q2q3 - ay) +
             (-_4bx * m->q3 + _2bz * m->q1) *
             (_2bx * (0.5f - q2q2 - q3q3) +
             _2bz * (q1q3 - q0q2) - mx) +
             (-_2bx * m->q0 + _2bz * m->q2) *
             (_2bx * (q1q2 - q0q3) +
             _2bz * (q0q1 + q2q3) - my) +
             _2bx * m->q1 *
             (_2bx * (q0q2 + q1q3) +
             _2bz * (0.5f - q1q1 - q2q2) - mz);

        recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        qDot1 -= m->beta * s0;
        qDot2 -= m->beta * s1;
        qDot3 -= m->beta * s2;
        qDot4 -= m->beta * s3;
    }

    m->q0 += qDot1 * m->invSampleFreq;
    m->q1 += qDot2 * m->invSampleFreq;
    m->q2 += qDot3 * m->invSampleFreq;
    m->q3 += qDot4 * m->invSampleFreq;

    recipNorm = invSqrt(m->q0*m->q0 + m->q1*m->q1 +
                        m->q2*m->q2 + m->q3*m->q3);

    m->q0 *= recipNorm;
    m->q1 *= recipNorm;
    m->q2 *= recipNorm;
    m->q3 *= recipNorm;

    m->anglesComputed = 0;
}