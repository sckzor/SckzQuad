#include <linux/types.h>

/*
 * Code to operate the gyroscope, accelerometer and magnetometer over the I2C bus
 *
 * Translated from Python code available at:
 *
 * https://makersportal.com/blog/2019/11/11/raspberry-pi-python-accelerometer-gyroscope-magnetometer
 *
 */

#ifndef _GYRO_H
#define _GYRO_H

static const int MAG_ADDRESS  = 0x0c; /* I2C address of the magnetometer module */
static const int GYRO_ADDRESS = 0x68; /* I2C address of the gyro/acclerometer module */

/* 
 * See the register map available at:
 * https://3cfeqx1hf82y3xcoull08ihx-wpengine.netdna-ssl.com/wp-content/uploads/2017/11/RM-MPU-9250A-00-v1.6.pdf
 */
static const __u8 AK8963_CNTL  = 0x0A;
static const __u8 HXH          = 0x04;
static const __u8 HYH          = 0x06;
static const __u8 HZH          = 0x08;
static const __u8 AK8963_ST2   = 0x09;
static const __u8 SMPLRT_DIV   = 0x19;
static const __u8 PWR_MGMT_1   = 0x6B;
static const __u8 CONFIG       = 0x1A;
static const __u8 GYRO_CONFIG  = 0x1B;
static const __u8 ACCEL_CONFIG = 0x1C;
static const __u8 INT_ENABLE   = 0x38;
static const __u8 ACCEL_XOUT_H = 0x3B;
static const __u8 ACCEL_YOUT_H = 0x3D;
static const __u8 ACCEL_ZOUT_H = 0x3F;
static const __u8 TEMP_OUT_H   = 0x41;
static const __u8 GYRO_XOUT_H  = 0x43;
static const __u8 GYRO_YOUT_H  = 0x45;
static const __u8 GYRO_ZOUT_H  = 0x47;

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

static int read_raw_gyro(int, __u8);
static int read_raw_mag(int, __u8);

int setup_gyro(int);
int setup_mag(int);
struct gyro_state get_gyro_state(int);
struct vec3 get_mag_state(int);

#endif
