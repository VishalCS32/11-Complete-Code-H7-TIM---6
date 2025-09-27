#ifndef YAW_CALCULATION_H
#define YAW_CALCULATION_H

#include <stdint.h>

// Initialize complementary filter for yaw
void Yaw_Init(float alpha);

// Update yaw using magnetometer and quaternion
float Yaw_Update(float mx, float my, float mz,
                 float q0, float q1, float q2, float q3);

#endif
