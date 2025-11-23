/*
 * test.cpp
 *
 *  Created on: Nov 6, 2025
 *      Author: abhatt
 */

#include <iostream>
#include <cstring>
#include <cstdio>

#include "arm_math.h"

#include "utils.h"

void Test1(){

	float32_t initialPosition = {0.0, 0.0, 0.0};
	float32_t initalVelocity = {0.0, 0.0, 0.0};
	float32_t initialQuaternion = {1.0, 0.0, 0.0, 0.0};

	float32_t dt = 0.1;
	int numSimulationSteps = 10;
	float32_t totalTime = numSimulationSteps * dt;

	// may have to initialize these as matrices
	float32_t exampleOmegaBody = {0.0, 0.0, 1.0};
	float32_t exampleAccelBody = {0.0, 0.0, -10.0};

	std::cout << "Initial Nominal State: \n";
	std::cout << nominalState << "\n";

	for (int i = 0; i < numSimulationSteps; i++) {
		nominalState.update(exampleOmegaBody, exampleAccelBody, dt);
	}

	float32_t t = numSimulationSteps * dt;

	std::cout << "State after" << numSimulationSteps << " steps and" << t << "seconds is: \n";
	std::cout << nominalState << "\n";

	// not sure if this is supposed to be t or total time
	float32_t expectedPosition = initialPosition + 0.5 * (exampleAccelBody + GRAVITY_INERTIAL) * t^2;
	float32_t expectedVelocity = initialVelocity + (exampleAccelBody + GRAVITY_INERTIAL) * t;
	//float32_t expectedQ = ;

	std::cout << "Expected final position: " << expctedPosition << "\n";
	std::cout << "Expected final velocity: " << expctedVelocity << "\n";
	std::cout << "Expected final Z rotation: " << expctedQ << "\n";

	std::cout << "\nSimulation complete.";
}



