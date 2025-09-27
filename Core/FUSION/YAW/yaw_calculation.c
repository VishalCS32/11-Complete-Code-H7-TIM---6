#include "yaw_calculation.h"
#include <math.h>

static float alpha_cf = 0.98f;   // Complementary filter gain
static float yaw_filtered = 0.0f;

// Initialize complementary filter
void Yaw_Init(float alpha)
{
    alpha_cf = alpha;
    yaw_filtered = 0.0f;
}

// Quaternion-based yaw calculation
float Yaw_Update(float mx, float my, float mz,
                 float q0, float q1, float q2, float q3)
{
    // Rotate magnetometer vector to Earth frame using quaternion
    float hx = 2.0f * (mx * (0.5f - q2 * q2 - q3 * q3) +
                       my * (q1 * q2 - q0 * q3) +
                       mz * (q1 * q3 + q0 * q2));

    float hy = 2.0f * (mx * (q1 * q2 + q0 * q3) +
                       my * (0.5f - q1 * q1 - q3 * q3) +
                       mz * (q2 * q3 - q0 * q1));

    // Raw yaw in [0, 360)
    float yaw_mag = atan2f(hy, hx) * (180.0f / M_PI);
    if (yaw_mag < 0.0f)
        yaw_mag += 360.0f;

    // ---- ANGLE UNWRAPPING ----
    float delta = yaw_mag - yaw_filtered;
    if (delta > 180.0f) delta -= 360.0f;
    else if (delta < -180.0f) delta += 360.0f;

    float yaw_continuous = yaw_filtered + delta;

    // Complementary filter
    yaw_filtered = alpha_cf * yaw_filtered + (1.0f - alpha_cf) * yaw_continuous;

    // Normalize back to [0, 360)
    if (yaw_filtered < 0.0f)
        yaw_filtered += 360.0f;
    else if (yaw_filtered >= 360.0f)
        yaw_filtered -= 360.0f;

    return yaw_filtered;
}
