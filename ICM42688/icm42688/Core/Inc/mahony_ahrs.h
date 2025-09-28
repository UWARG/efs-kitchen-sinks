#ifndef MAHONY_AHRS_H_
#define MAHONY_AHRS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Mahony AHRS state */
typedef struct {
    /* tunable params */
    float twoKp;          /* 2 * proportional gain (Kp) */
    float twoKi;          /* 2 * integral gain (Ki) */

    /* quaternion of sensor frame relative to auxiliary frame */
    float q0, q1, q2, q3;

    /* integral feedback terms */
    float integralFBx, integralFBy, integralFBz;

    /* inverse sample frequency (1 / sample_freq) */
    float invSampleFreq;

    /* Euler angles (radians) */
    float roll;
    float pitch;
    float yaw;
} Mahony;

/* initialize Mahony struct to defaults */
void Mahony_init(Mahony *m);

/* IMU-only update (gyros in deg/s, acc in same units) */
void Mahony_updateIMU(Mahony *m, float gx, float gy, float gz,
                      float ax, float ay, float az);

/* fast inverse square-root */
float Mahony_invSqrt(float x);

#ifdef __cplusplus
}
#endif

#endif /* MAHONY_AHRS_H_ */
