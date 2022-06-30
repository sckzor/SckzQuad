#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "gyro.h"
#include "smbus.h"

/*
 * Code to operate the gyroscope, accelerometer and magnetometer over the I2C bus
 *
 * Translated from Python code available at:
 *
 * https://makersportal.com/blog/2019/11/11/raspberry-pi-python-accelerometer-gyroscope-magnetometer
 *
 */

static int instantiate_device(int, int);
static int read_raw_gyro(int, __u8);
static int read_raw_mag(int, __u8);

int setup_mag(int adapter_nr) {
	/*
	 * Setup the magnetometer unite on the MPU-92/65
	 */

	const int MAG_ADDRESS = 0x0c;

	/*
	 * Split into two nibbles, first nibble (0001) defines 16 bit
	 * percision second one defines a sample rate of 100 Hz (0110).
	 */
	const __u8 AK8963_MODE = 0b00010110;

	/* 
	 * See the register map available at https://bit.ly/3ndiyOo
	 */
	const __u8 AK8963_CNTL = 0x0A;

	int mag;
	__s32 res;

	mag = instantiate_device(adapter_nr, MAG_ADDRESS);

	res = i2c_smbus_write_byte_data(mag, AK8963_CNTL, 0x00); /* Zero the control settings */
	if (res != 0) {
		printf("There was an error zeroing the control setttings of the magnetometer\r\n");
		exit(1);
	}
	// usleep(100);

	res = i2c_smbus_write_byte_data(mag, AK8963_CNTL, AK8963_MODE); /* Set the control settings */
	if (res != 0) {
		printf("There was an error setting the control setttings of the magnetometer\r\n");
		exit(1);
	}
	// usleep(100);

	return mag;
}

int read_raw_mag(int file, __u8 address) {
	/*
	 * Read raw data from the magnetometer
	 */

	__s32 res, res_low, res_high;

	res_low = i2c_smbus_read_byte_data(file, address-1);
	res_high = i2c_smbus_read_byte_data(file, address);

	if (res_low < 0 | res_high < 0) {
		printf("There was an error reading raw magnetometer data\r\n");
		exit(1);
	}
	
	res = ((res_high << 8) | res_low);

	if (res > 32768) {
		res -= 65536;
	}

	return res;
}

struct vec3 get_mag_state(int file) {
	/*
	 * Get the full state of the magnetometer
	 */

	const double MAG_SENS = 4900.0;
	const double TWO_POW_FIFTEEN = 32768;

	/* 
	 * See the register map available at https://bit.ly/3ndiyOo
	 */
	const __u8 HXH        = 0x04;
	const __u8 HYH        = 0x06;
	const __u8 HZH        = 0x08;
	const __u8 AK8963_ST2 = 0x09;
	
	int loop_count;

	struct vec3 raw;
	struct vec3 m;
	
	
	while (1) {
		/* Read the data for x, y and z */
		raw.x = read_raw_mag(file, HXH);
		raw.y = read_raw_mag(file, HYH);
		raw.z = read_raw_mag(file, HZH);

		/* wait until the ST2 register has tells us the value is correct */
		if (i2c_smbus_read_byte_data(file, AK8963_ST2) == 0b10000) {
			break;
		}
	}
	      
	m.x = (raw.x / TWO_POW_FIFTEEN)*MAG_SENS;
	m.y = (raw.y / TWO_POW_FIFTEEN)*MAG_SENS;
	m.z = (raw.z / TWO_POW_FIFTEEN)*MAG_SENS;
	
	return m;
}


int setup_gyro(int adapter_nr) {
	/*
	 * Setup the gyroscope and acclerometer unit on the MPU-92/65
	 */

	const int GYRO_ADDRESS = 0x68; /* I2C address of the gyro/acclerometer module */

	const __u8 SAMPLE_DIV = 0; /* sample rate = 8 kHz/(1+sample_div) */
	
	/* 
	 * See the register map available at https://bit.ly/3ndiyOo
	 */
	const __u8 SMPLRT_DIV   = 0x19;
	const __u8 PWR_MGMT_1   = 0x6B;
	const __u8 CONFIG       = 0x1A;
	const __u8 GYRO_CONFIG  = 0x1B;
	const __u8 ACCEL_CONFIG = 0x1C;
	const __u8 INT_ENABLE   = 0x38;


	int gyro;
	__s32 res;

	gyro = instantiate_device(adapter_nr, GYRO_ADDRESS);
	// usleep(100);

	res = i2c_smbus_write_byte_data(gyro, SMPLRT_DIV, SAMPLE_DIV); /* Set the propper clock frequency */
	if (res != 0) {
		printf("There was an error configuring the sample rate of the gyro\r\n");
		exit(1);
	}
	// usleep(100);

	res = i2c_smbus_write_byte_data(gyro, PWR_MGMT_1, 0x00); /* Force a reset on the chip */
	if (res != 0) {
		printf("There was an error forceing a power cycle of the chip\r\n");
		exit(1);
	}
	// usleep(100);

	res = i2c_smbus_write_byte_data(gyro, PWR_MGMT_1, 0x01); /* Configure the clock to use best signal */
	if (res != 0) {
		printf("There was an error configuring the clock signal on the chip\r\n");
		exit(1);
	}
	// usleep(100);

	res = i2c_smbus_write_byte_data(gyro, CONFIG, 0x00); /* Zero the general configuration */
	if (res != 0) {
		printf("There was an error zeroing the general config\r\n");
		exit(1);
	}
	// usleep(100);

	/*
	 * Degrees per second values:
	 * 0b00000, 0b010000, 0b10000, 0b11000
	 * 250.0,   500.0,    1000.0,  2000.0
	 */

	res = i2c_smbus_write_byte_data(gyro, GYRO_CONFIG, 0b00000); /* Configure the gyroscope */
	if (res != 0) {
		printf("There was an error configuring the gyroscope\r\n");
		exit(1);
	}
	// usleep(100);

	/*
	 * Meters per second squared values:
	 * 0b00000, 0b01000, 0b10000, 0b11000
	 * 2.0,     4.0,     8.0,     16.0
	 */
	
	res = i2c_smbus_write_byte_data(gyro, CONFIG, 0b00000); /* Configure the accelerometer */
	if (res != 0) {
		printf("There was an error configuring the accelerometer\r\n");
		exit(1);
	}
	// usleep(100);

	
	res = i2c_smbus_write_byte_data(gyro, INT_ENABLE, 0x01); /* Enable data output */
	if (res != 0) {
		printf("There was an error enabling the chip\r\n");
		exit(1);
	}
	// usleep(100);

	return gyro;
}

int read_raw_gyro(int file, __u8 address) {
	__s32 res, res_high, res_low;

	res_high = i2c_smbus_read_byte_data(file, address);
	res_low = i2c_smbus_read_byte_data(file, address + 1);

	if (res_high < 0 | res_low < 0) {
		printf("There was an error reading the data at address 0x%x", address);
		exit(1);
	}

	res = ((res_high << 8) | res_low);

	if (res > 32768) {
		res -= 65536;
	}

	return res;
}

struct gyro_state get_gyro_state(int file) {
	struct gyro_state g_state;

	struct vec3 raw_a;
	struct vec3 raw_w;
	int raw_temp;

	/* 
	 * See the register map available at https://bit.ly/3ndiyOo
	 */
	const __u8 ACCEL_XOUT_H = 0x3B;
	const __u8 ACCEL_YOUT_H = 0x3D;
	const __u8 ACCEL_ZOUT_H = 0x3F;
	const __u8 TEMP_OUT_H   = 0x41;
	const __u8 GYRO_XOUT_H  = 0x43;
	const __u8 GYRO_YOUT_H  = 0x45;
	const __u8 GYRO_ZOUT_H  = 0x47;

	const double TWO_POW_FIFTEEN = 32768;

	/* Read the data from the accelerometer */
	raw_a.x = read_raw_gyro(file, ACCEL_XOUT_H);
	raw_a.y = read_raw_gyro(file, ACCEL_YOUT_H);
	raw_a.z = read_raw_gyro(file, ACCEL_ZOUT_H);

	/* Read the data from the gyroscope */
	raw_w.x = read_raw_gyro(file, GYRO_XOUT_H);
	raw_w.y = read_raw_gyro(file, GYRO_YOUT_H);
	raw_w.z = read_raw_gyro(file, GYRO_ZOUT_H);

	/* Read data from the thermometer */
	raw_temp = read_raw_gyro(file, TEMP_OUT_H);

	/*
	 * Magic "2.0" number comes from above table about meters per second squared.
	 */
	g_state.a.x = (raw_a.x / TWO_POW_FIFTEEN) * 2.0;
	g_state.a.y = (raw_a.y / TWO_POW_FIFTEEN) * 2.0;
	g_state.a.z = (raw_a.z / TWO_POW_FIFTEEN) * 2.0;

	/*
	 * Magic "250.0" number comes from above table about degrees per second.
	 */
	g_state.w.x = (raw_w.x / TWO_POW_FIFTEEN) * 250.0;
	g_state.w.y = (raw_w.y / TWO_POW_FIFTEEN) * 250.0;
	g_state.w.z = (raw_w.z / TWO_POW_FIFTEEN) * 250.0;

	g_state.temp = ((raw_temp) / 333.87) + 21.0;

	return g_state;
}

int instantiate_device(int adapter_nr, int address) {

	/*
	 * Instantiate the device with the specified address on the specified adapter.
	 */

	int file;
	char filename[20];

	snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);

	file = open(filename, O_RDWR);
	if (file < 0) {
		printf("There was an error (#%d) opening the device file %s\r\n", file, filename);
		exit(1);
	}


	if (ioctl(file, I2C_SLAVE, address) < 0) {
		printf("There was selecting the device 0x%x\r\n", address);
		exit(1);
	}

	return file;
}
