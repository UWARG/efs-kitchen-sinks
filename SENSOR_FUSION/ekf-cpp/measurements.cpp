/*
 * measurements.cpp
 *
 *  Created on: May 18, 2026
 *      Author: aahan
 */

#include "measurements.hpp"

#include "arm_math.h"

Measurements::Measurements(
    const float32_t* gyro_initial,
    const float32_t* accel_initial,
    const float32_t* mag_initial
) {
	//Populate measurements
    if (gyro_initial != nullptr) {
        arm_copy_f32(gyro_initial, gyro_prev, VECTOR_SIZE);
        arm_copy_f32(gyro_initial, gyro_new, VECTOR_SIZE);
    } else {
        arm_fill_f32(0.0f, gyro_prev, VECTOR_SIZE);
        arm_fill_f32(0.0f, gyro_new, VECTOR_SIZE);
    }

    if (accel_initial != nullptr) {
        arm_copy_f32(accel_initial, accel_prev, VECTOR_SIZE);
        arm_copy_f32(accel_initial, accel_new, VECTOR_SIZE);
    } else {
        arm_fill_f32(0.0f, accel_prev, VECTOR_SIZE);
        arm_fill_f32(0.0f, accel_new, VECTOR_SIZE);
    }

    if (mag_initial != nullptr) {
        arm_copy_f32(mag_initial, mag_prev, VECTOR_SIZE);
        arm_copy_f32(mag_initial, mag_new, VECTOR_SIZE);
    } else {
        arm_fill_f32(0.0f, mag_prev, VECTOR_SIZE);
        arm_fill_f32(0.0f, mag_new, VECTOR_SIZE);
    }

    arm_fill_f32(0.0f, gyro_bias_accumulated, VECTOR_SIZE);
    arm_fill_f32(0.0f, accel_bias_accumulated, VECTOR_SIZE);
    arm_fill_f32(0.0f, mag_bias_accumulated, VECTOR_SIZE);

    UpdateAllBars();
}

void Measurements::UpdateGyro(const float32_t* gyro_new_in) {
    if (gyro_new_in == nullptr) {
        return;
    }


    arm_copy_f32(gyro_new, gyro_prev, VECTOR_SIZE);
    arm_sub_f32(gyro_new_in, gyro_bias_accumulated, gyro_new, VECTOR_SIZE);

    UpdateGyroBar();
}

void Measurements::UpdateAccel(const float32_t* accel_new_in) {
    if (accel_new_in == nullptr) {
        return;
    }


    arm_copy_f32(accel_new, accel_prev, VECTOR_SIZE);
    arm_sub_f32(accel_new_in, accel_bias_accumulated, accel_new, VECTOR_SIZE);

    UpdateAccelBar();
}

void Measurements::UpdateMag(const float32_t* mag_new_in) {
    if (mag_new_in == nullptr) {
        return;
    }

    arm_copy_f32(mag_new, mag_prev, VECTOR_SIZE);
    arm_sub_f32(mag_new_in, mag_bias_accumulated, mag_new, VECTOR_SIZE);

    UpdateMagBar();
}

void Measurements::UpdateBiases(
    const float32_t* gyro_bias_new,
    const float32_t* accel_bias_new,
    const float32_t* mag_bias_new
) {


    if (gyro_bias_new != nullptr) {
        arm_add_f32(
            gyro_bias_accumulated,
            gyro_bias_new,
            gyro_bias_accumulated,
            VECTOR_SIZE
        );
    }

    if (accel_bias_new != nullptr) {
        arm_add_f32(
            accel_bias_accumulated,
            accel_bias_new,
            accel_bias_accumulated,
            VECTOR_SIZE
        );
    }

    if (mag_bias_new != nullptr) {
        arm_add_f32(
            mag_bias_accumulated,
            mag_bias_new,
            mag_bias_accumulated,
            VECTOR_SIZE
        );
    }
}

void Measurements::UpdateGyroBar() {
    AverageVector3(gyro_prev, gyro_new, gyro_bar);
}

void Measurements::UpdateAccelBar() {
    AverageVector3(accel_prev, accel_new, accel_bar);
}

void Measurements::UpdateMagBar() {
    AverageVector3(mag_prev, mag_new, mag_bar);
}

void Measurements::UpdateAllBars() {
    UpdateGyroBar();
    UpdateAccelBar();
    UpdateMagBar();
}

void Measurements::AverageVector3(
    const float32_t* a,
    const float32_t* b,
    float32_t* out
) {
    float32_t temp[VECTOR_SIZE];

    arm_add_f32(a, b, temp, VECTOR_SIZE);
    arm_scale_f32(temp, 0.5f, out, VECTOR_SIZE);
}


