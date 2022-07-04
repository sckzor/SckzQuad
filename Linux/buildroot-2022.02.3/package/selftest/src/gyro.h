/*
 * gyro.h
 *
 * Header that includes functions for gyroscope, accelerometer and magnetometer opperation
 */


#ifndef _GYRO_H
#define _GYRO_H

struct vec3 {
	double x;
	double y;
	double z;
};

struct gyro_state {
	struct vec3 a;
	struct vec3 w;
	double temp;
};

int setup_gyro(int);
int setup_mag(int);
struct gyro_state get_gyro_state(int);
struct vec3 get_mag_state(int);
int instantiate_device(int, int);

#endif
