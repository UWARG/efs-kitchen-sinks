#ifndef IMU_DATATYPES_HPP
#define IMU_DATATYPES_HPP

#include <cstdint>

typedef struct {
    float xacc;
    float yacc;
    float zacc;
    float xgyro;
    float ygyro;
    float zgyro;
} IMUData_t;

#endif
