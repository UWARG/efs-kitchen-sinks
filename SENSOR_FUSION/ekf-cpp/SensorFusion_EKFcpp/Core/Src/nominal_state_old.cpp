 /*
  * nominal_state.cpp
  *
  *  Created on: Oct 8, 2025
  *      Author: aahan
  */

#include <nominal_state_old.hpp>
#include <utils.hpp>
#include "arm_math.h"
#include <cstring>
#include <cstdio>

#include "dsp/matrix_functions.h"
#include "dsp/matrix_functions_f16.h"


NominalState::NominalState() {
    // Initialize CMSIS matrix instances (each as column vector)
    arm_mat_init_f32(&prevDisplacement,    3, 1, displacementData);
    arm_mat_init_f32(&prevVelocity,        3, 1, velocityData);
    arm_mat_init_f32(&prevQuaternion,      4, 1, quaternionData);
    arm_mat_init_f32(&prevGyroMeasurement, 3, 1, gyroPrevData);
    arm_mat_init_f32(&prevAccelMeasurement,3, 1, accelPrevData);
    arm_mat_init_f32(&gravityInertialMat,  3, 1, gravityData);

    // Zero initialize vectors
    arm_fill_f32(0.0f, displacementData, 3);
    arm_fill_f32(0.0f, velocityData, 3);
    arm_fill_f32(0.0f, gyroPrevData, 3);
    arm_fill_f32(0.0f, accelPrevData, 3);

    // Identity quaternion
    quaternionData[0] = 1.0f;   // w
    quaternionData[1] = 0.0f;   // x
    quaternionData[2] = 0.0f;   // y
    quaternionData[3] = 0.0f;   // z

    //Need to normalize the quaternion

    // Gravity in inertial frame
    gravityData[0] = 0.0f;
    gravityData[1] = 0.0f;
    gravityData[2] = 9.81f;
}

void NominalState::Update(){
	 //set up matrix for gyro and accel _measurement

		 newQuaternion = UpdateQuaternion(gyro_measurement, dt);
		 newVelocity = UpdateVelocity(quaternion_new, accel_measurement, dt);
		 newDisplacement = UpdateDisplacement(velocity_new, dt);

		 prevGyroMeasuremenr = gyroMeasurement
		 prevAccelMeasurement = accelMeasurement;
		 prevQuaternion = newQuaternion;
		 prevVelocity = newVelocity;
		 prevDisplacement = newDisplacement;
	    }


void NominalState::UpdateQuaternion(float32_t *gyroMeasurement){

		float32_t gyroBarData[3];
		arm_matrix_instance_f32 gyroBar;
		arm_mat_init_f32 (&gyroBar, 3, 1, gyroBarData);

		arm_add_f32(&gyroMeasurement, prevGyroMeasurement.pData, gyroBarData, 3);
		arm_scale_f32(gyroBarData, 0.5f, gyroBarData, 3);

		float32_t omegaMatrixOut[16];
		ExpOmegaMatrix(gyroBarData, dt, omegaMatrixOut);



	};

void NominalState::ExpOmegaMatrix(float32_t *gyroBar, float32_t dt, float32_t *omegaMatrixOut){

	   float32_t normGyro;
	   normalizeVector(gyroBar, normGyro);
	   float32_t normSigma = 0.5 * dt * normGyro;

	   int gx = gyroBar[0];
	   int gy = gyroBar[1];
	   int gz = gyroBar[2];

	   float32_t gyroMultMatrix[16] = {
			0.0f, -gx, -gy, -gz,
			gx, 0.0f, gz, -gy,
			gy, -gz, 0.0f, gx,
			gz, gy, -gx, 0.0f
	   };

	   const float32_t iMatrix[16] = {
	       1,0,0,0,
	       0,1,0,0,
	       0,0,1,0,
	       0,0,0,1
	   };

	    // Compute cosine and sine terms
	    float32_t cosTerm = arm_cos_f32(sigma);
	    float32_t sinTerm = arm_sin_f32(sigma);

	    // Scale and combine
	    float32_t temp1[16];
	    float32_t temp2[16];

	    // temp1 = cos(σ)*I
	    arm_scale_f32(I4, cosTerm, temp1, 16);

	    // temp2 = (sin(σ)/|ω|)*Ω
	    arm_scale_f32(gyroMultMatrix, sinTerm / gyroNorm, temp2, 16);

	    // omegaMatrixOut = temp1 + temp2
	    arm_add_f32(temp1, temp2, omegaMatrixOut, 16);
	};

void NominalState::UpdateVelocity(const float32_t* quaternionNew, const float32_t* accelBodyNew, float32_t dt, float32_t* velocityOut){

		float32_t newQuaternionRotData[9];
		float32_t oldQuaternionRotData[9];

	    BToIFrameRotMatrix(quaternionNew, newQuaternionRotData);
	    BToIFrameRotMatrix(prevQuaternion, oldQuaternionRotData);

	    arm_matrix_instance_f32 newQuaternionRot, prevQuaternionRot;
	    arm_mat_init_f32(&newQuaternionRot, 3, 3, newQuaternionRotData);
	    arm_mat_init_f32(&prevQuaternionRot, 3, 3, oldQuaternionRotData);


	    arm_matrix_instance_f32 accelBodyNewMat, accelPrevMat;
	    arm_mat_init_f32(&accelBodyNewMat, 3, 1, (float32_t*)accelBodyNew);
	  	arm_mat_init_f32(&accelPrevMat, 3, 1, prevAccelMeasurement.pData);

	    arm_matrix_instance_f32 accelInertialNew, accelInertialOld;
	    float32_t accelInertialNewData[3];
	    float32_t accelInertialOldData[3];
	    arm_mat_init_f32(&accelInertialNew, 3, 1, accelInertialNewData);
	    arm_mat_init_f32(&accelInertialOld, 3, 1, accelInertialOldData);

	  	arm_mat_mult_f32(&newQuaternionRot, &accelBodyNewMat, &accelInertialNew);
	  	arm_mat_mult_f32(&newQuaternionRot, &accelBodyNewMat, &accelInertialNew);

	  	float32_t accelAvg[3];
	  	arm_add_f32(accelInertialNewData, accelInertialOldData, accelAvg)
	  	arm_scale_f32(accelAvg, 0.5f, accelAvg, 3);

	  	// --- Step 4: Add gravity (elementwise) ---
	  	float32_t accelPlusGrav[3];
	  	arm_add_f32(accelAvgData, gravityInertialMat.pData, accelPlusGrav, 3);

	  	// --- Step 5: Multiply by dt ---
	  	float32_t accelDt[3];
	  	arm_scale_f32(accelPlusGrav, dt, accelDt, 3);

	  	// --- Step 6: Get final velocity by adding to prev velocity
	  	arm_add_f32(accelDt, prevVelocity.pData, velocityOut, 3);

	};

void NominalState:: UpdateDisplacement(const float32_t* velocityNew, float32_t dt, float32_t* displacementOut){

		float32_t tempSum[3];
		float32_t velocityAvg[3];
		arm_add_f32(velocityNew, prevVelocity.pData, tempSum, 3)
		arm_scale_f32(tempSum, 0.5f, velocityAvg, 3);

		float32_t displacement[3];
		arm_scale_f32(velocityAvg, dt, displacement, 3);

		arm_add_f32(displacement, prevDisplacement.pData, displacementOut, 3);

	};
