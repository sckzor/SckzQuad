#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "gyro.h"
#include "smbus.h"
#include "i2c.h"

/*
 * Setup the magnetometer unite on the MPU-92/65
 */
int setup_mag(int adapter_nr) {
	int mag;
	__s32 res;

	/*
	 * Split into two nibbles, first nibble (0001) defines 16 bit
	 * percision second one defines a sample rate of 100 Hz (0110).
	 */
	const __u8 AK8963_MODE = 0b00010110;

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

/*
 * Read raw data from the magnetometer
 */
int read_raw_mag(int file, __u8 address) {
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

/*
 * Get the full state of the magnetometer
 */
struct vec3 get_mag_state(int file) {
	struct vec3 raw;
	struct vec3 m;
	
	const double MAG_SENS = 4900.0;
	const double TWO_POW_FIFTEEN = 32768;
	
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


/*
 * Setup the gyroscope and acclerometer unit on the MPU-92/65
 */
int setup_gyro(int adapter_nr) {
	int gyro;
	__s32 res;
	
	const __u8 SAMPLE_DIV = 0; /* sample rate = 8 kHz/(1+sample_div) */

	gyro = instantiate_device(adapter_nr, GYRO_ADDRESS);

	res = i2c_smbus_write_byte_data(gyro, SMPLRT_DIV, SAMPLE_DIV); /* Set the propper clock frequency */
	if (res != 0) {
		printf("There was an error configuring the sample rate of the gyro\r\n");
		exit(1);
	}

	res = i2c_smbus_write_byte_data(gyro, PWR_MGMT_1, 0x00); /* Force a reset on the chip */
	if (res != 0) {
		printf("There was an error forceing a power cycle of the chip\r\n");
		exit(1);
	}

	res = i2c_smbus_write_byte_data(gyro, PWR_MGMT_1, 0x01); /* Configure the clock to use best signal */
	if (res != 0) {
		printf("There was an error configuring the clock signal on the chip\r\n");
		exit(1);
	}

	res = i2c_smbus_write_byte_data(gyro, CONFIG, 0x00); /* Zero the general configuration */
	if (res != 0) {
		printf("There was an error zeroing the general config\r\n");
		exit(1);
	}

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

	
	res = i2c_smbus_write_byte_data(gyro, INT_ENABLE, 0x01); /* Enable data output */
	if (res != 0) {
		printf("There was an error enabling the chip\r\n");
		exit(1);
	}

	return gyro;
}

/*
 * Read raw data from the gyroscope and accelerometer
 */
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

/*
 * Get the full state of the gyroscope
 */
struct gyro_state get_gyro_state(int file) {
	struct gyro_state g_state;

	struct vec3 raw_a;
	struct vec3 raw_w;
	int raw_temp;

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
