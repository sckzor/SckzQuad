#include <math.h>

/*
 * Code that allows accelerometer, magnetometer and gyroscope readings to be turned into heading
 * for the air craft (AHRS)
 *
 * Modified from the following arduino library:
 *
 * https://github.com/kriswiner/MPU9250/blob/master/quaternionFilters.ino
 */

#ifndef _MADGWICK_H
#define _MADGWICK_H

static const double GYRO_MEAS_ERROR = M_PI * (60.0f / 180.0f); /* Estimated error of the gyroscope */
static const double BETA = sqrt(3.0f / 4.0f) * GYRO_MEAS_ERROR; /* 2 times the proportional gain */

struct quaternion {
    double q1;
    double q2;
    double q3;
    double q4;
};

struct vec3 get_angle(struct vec3, struct vec3, struct vec3, double);

static struct vec3 to_euler(struct quaternion);

#endif
