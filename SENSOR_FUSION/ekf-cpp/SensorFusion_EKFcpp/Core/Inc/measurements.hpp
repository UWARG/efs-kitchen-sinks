/*
 * Measurements.hpp
 *
 *  Created on: May 18, 2026
 *      Author: aahan
 */

#ifndef MEASUREMENTS_HPP_
#define MEASUREMENTS_HPP_

#include "arm_math.h"
#include <cstdint>

class Measurements {
public:
    static constexpr uint32_t VECTOR_SIZE = 3;

    Measurements(
        const float32_t* gyro_initial = nullptr,
        const float32_t* accel_initial = nullptr,
        const float32_t* mag_initial = nullptr
    );

    void UpdateGyro(const float32_t* gyro_new_in);
    void UpdateAccel(const float32_t* accel_new_in);
    void UpdateMag(const float32_t* mag_new_in);

    void UpdateBiases(
        const float32_t* gyro_bias_new,
        const float32_t* accel_bias_new,
        const float32_t* mag_bias_new
    );

    void UpdateGyroBar();
    void UpdateAccelBar();
    void UpdateMagBar();
    void UpdateAllBars();

    float32_t gyro_prev[VECTOR_SIZE];
    float32_t gyro_new[VECTOR_SIZE];
    float32_t gyro_bar[VECTOR_SIZE];

    float32_t accel_prev[VECTOR_SIZE];
    float32_t accel_new[VECTOR_SIZE];
    float32_t accel_bar[VECTOR_SIZE];

    float32_t mag_prev[VECTOR_SIZE];
    float32_t mag_new[VECTOR_SIZE];
    float32_t mag_bar[VECTOR_SIZE];

    float32_t gyro_bias_accumulated[VECTOR_SIZE];
    float32_t accel_bias_accumulated[VECTOR_SIZE];
    float32_t mag_bias_accumulated[VECTOR_SIZE];

private:
    void CopyVector3(const float32_t* v_in, float32_t* v_out);
    void ZeroVector3(float32_t* v_out);
    void AverageVector3(const float32_t* a, const float32_t* b, float32_t* out);
};

#endif  // MEASUREMENTS_HPP_
