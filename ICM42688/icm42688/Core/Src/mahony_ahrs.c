#include "mahony_ahrs.h"
#include <math.h>
#include <stdint.h>

/* Defaults */
#define DEFAULT_SAMPLE_FREQ 512.0f
#define twoKpDef    (30.0f)   /* 2 * proportional gain */
#define twoKiDef    (2.0f)   /* 2 * integral gain */

/* Initialize Mahony structure with defaults */
void Mahony_init(Mahony *m)
{
    if (m == NULL) return;

    m->twoKp = twoKpDef;
    m->twoKi = twoKiDef;
    m->q0 = 1.0f;
    m->q1 = 0.0f;
    m->q2 = 0.0f;
    m->q3 = 0.0f;
    m->integralFBx = 0.0f;
    m->integralFBy = 0.0f;
    m->integralFBz = 0.0f;
    m->invSampleFreq = 1.0f / DEFAULT_SAMPLE_FREQ;
    m->roll = 0.0f;
    m->pitch = 0.0f;
    m->yaw = 0.0f;
}

/* Fast inverse square-root (Quake III style) */
float Mahony_invSqrt(float x)
{
    if (x <= 0.0f) return 0.0f;

    float halfx = 0.5f * x;
    union {
        float f;
        uint32_t i;
    } u;

    u.f = x;
    /* magic number and bit-level hack */
    u.i = 0x5f3759df - (u.i >> 1);
    float y = u.f;
    /* two Newton-Raphson iterations for accuracy */
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/* IMU-only update (no magnetometer) */
/* gx,gy,gz expected in degrees/sec (function converts to rad/sec) */
void Mahony_updateIMU(Mahony *m, float gx, float gy, float gz,
                      float ax, float ay, float az)
{
    if (m == NULL) return;

    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    /* convert deg/s to rad/s */
    const float DEG_TO_RAD = 0.01745329251994329576923690768489f; /* more accurate */
    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;

    /* Compute feedback only if accelerometer measurement valid
       (avoids NaN in accelerometer normalisation) */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        /* Normalise accelerometer measurement */
        recipNorm = Mahony_invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* Estimated direction of gravity (half vector) */
        halfvx = m->q1 * m->q3 - m->q0 * m->q2;
        halfvy = m->q0 * m->q1 + m->q2 * m->q3;
        halfvz = m->q0 * m->q0 - 0.5f + m->q3 * m->q3;

        /* Error is sum of cross product between estimated
           and measured direction of gravity */
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        /* Compute and apply integral feedback if enabled */
        if (m->twoKi > 0.0f) {
            m->integralFBx += m->twoKi * halfex * m->invSampleFreq;
            m->integralFBy += m->twoKi * halfey * m->invSampleFreq;
            m->integralFBz += m->twoKi * halfez * m->invSampleFreq;
            gx += m->integralFBx; /* apply integral feedback */
            gy += m->integralFBy;
            gz += m->integralFBz;
        } else {
            m->integralFBx = 0.0f; /* prevent integral windup */
            m->integralFBy = 0.0f;
            m->integralFBz = 0.0f;
        }

        /* Apply proportional feedback */
        gx += m->twoKp * halfex;
        gy += m->twoKp * halfey;
        gz += m->twoKp * halfez;
    }

    /* Integrate rate of change of quaternion */
    gx *= (0.5f * m->invSampleFreq);
    gy *= (0.5f * m->invSampleFreq);
    gz *= (0.5f * m->invSampleFreq);

    qa = m->q0;
    qb = m->q1;
    qc = m->q2;

    m->q0 += (-qb * gx - qc * gy - m->q3 * gz);
    m->q1 += (qa * gx + qc * gz - m->q3 * gy);
    m->q2 += (qa * gy - qb * gz + m->q3 * gx);
    m->q3 += (qa * gz + qb * gy - qc * gx);

    /* Normalise quaternion */
    recipNorm = Mahony_invSqrt(m->q0 * m->q0 + m->q1 * m->q1 +
                              m->q2 * m->q2 + m->q3 * m->q3);
    m->q0 *= recipNorm;
    m->q1 *= recipNorm;
    m->q2 *= recipNorm;
    m->q3 *= recipNorm;

    /* Compute Euler angles (radians) */
    m->roll  = atan2f(m->q0 * m->q1 + m->q2 * m->q3,
                      0.5f - m->q1 * m->q1 - m->q2 * m->q2);
    m->pitch = asinf(-2.0f * (m->q1 * m->q3 - m->q0 * m->q2));
    m->yaw   = atan2f(m->q1 * m->q2 + m->q0 * m->q3,
                      0.5f - m->q2 * m->q2 - m->q3 * m->q3);

    /* Convert to degrees */
    const float RAD_TO_DEG = 57.29577951308232f;
    m->roll  *= RAD_TO_DEG;
    m->pitch *= RAD_TO_DEG;
    m->yaw   *= RAD_TO_DEG;
}
