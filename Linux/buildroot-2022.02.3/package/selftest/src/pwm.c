#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "smbus.h"
#include "pwm.h"
#include "gyro.h"

int setup_pwm(int adapter_nr) {

	/*
	 * Translated from https://github.com/adafruit/Adafruit_Python_PCA9685
	 */

	const int PWM_ADDRESS = 0x40;

	int pwm;
	__s32 res;
	__u8 mode1;

	pwm = instantiate_device(adapter_nr, PWM_ADDRESS);
	set_all_pwm(pwm, 0, 0);

	res = i2c_smbus_write_byte_data(pwm, MODE2, OUTDRV);
	if(res != 0) {
		printf("Failed to set MODE1 value\r\n");
		exit(1);
	}

	res = i2c_smbus_write_byte_data(pwm, MODE1, ALLCALL);
	if(res != 0) {
		printf("Failed to set MODE2 value\r\n");
		exit(1);
	}

	mode1 = i2c_smbus_read_byte_data(pwm, MODE1);
	mode1 = mode1 & ~SLEEP; // Reset the sleep to wake the chip up
	
	res = i2c_smbus_write_byte_data(pwm, MODE1, mode1);
	if(res != 0) {
		printf("Failed to take the chip out of sleep\r\n");
		exit(1);
	}
	
	return pwm;
}

void set_pwm_frequency(int file, double hz) {
	double pre_scale_val;
	__s32 res;
	__u8 pre_scale, old_mode, new_mode;

	pre_scale_val = 25000000.0; // 25 Mhz
	pre_scale_val /= 4096; // 12 bit
	pre_scale_val /= hz;
	pre_scale_val -= 1.0;

	pre_scale = (__u8)(floor(pre_scale_val + 0.5));
	old_mode = i2c_smbus_read_byte_data(file, MODE1);

	new_mode = (old_mode & 0x7F) | SLEEP; // Make processor go to sleep
	
	res = i2c_smbus_write_byte_data(file, MODE1, new_mode);
	if(res != 0) {
		printf("Failed to make processor sleep %d\r\n", res);
		exit(1);
	}

	res = i2c_smbus_write_byte_data(file, PRESCALE, pre_scale);
	if(res != 0) {
		printf("Failed to change prescale %d\r\n", res);
		exit(1);
	}

	res = i2c_smbus_write_byte_data(file, MODE1, old_mode);
	if(res != 0) {
		printf("Failed to take chip out of sleep %d\r\n", res);
		exit(1);
	}

	res = i2c_smbus_write_byte_data(file, MODE1, old_mode | RESTART);
	if(res != 0) {
		printf("Failed to restart the chip %d\r\n", res);
		exit(1);
	}
}

void set_pwm(int file, int channel, int on, int off) {
	__s32 res;
	if(channel < 16 && channel > -1) {
		res = i2c_smbus_write_byte_data(file, LED0_ON_L + (4 * channel), (__u8)(on & 0xFF));
		if(res != 0) {
			printf("Failed to set pwm 1 %d\r\n", res);
			exit(1);
		}
		res = i2c_smbus_write_byte_data(file, LED0_ON_H + (4 * channel), (__u8)(on >> 8));
		if(res != 0) {
			printf("Failed to set pwm 2 %d\r\n", res);
			exit(1);
		}
		res = i2c_smbus_write_byte_data(file, LED0_OFF_L + (4 * channel), (__u8)(off & 0xFF));
		if(res != 0) {
			printf("Failed to set pwm 3 %d\r\n", res);
			exit(1);
		}
		res = i2c_smbus_write_byte_data(file, LED0_OFF_H + (4 * channel), (__u8)(off >> 8));
		if(res != 0) {
			printf("Failed to set pwm 4 %d\r\n", res);
			exit(1);
		}
	}
}

void set_all_pwm(int file, int on, int off) {
	__s32 res;
	res = i2c_smbus_write_byte_data(file, ALL_LED_ON_L, (__u8)(on & 0xFF));
	if(res != 0) {
		printf("Failed to set all pwm 1 %d\r\n", res);
		exit(1);
	}
	res = i2c_smbus_write_byte_data(file, ALL_LED_ON_H, (__u8)(on >> 8));
	if(res != 0) {
		printf("Failed to set all pwm 2 %d\r\n", res);
		exit(1);
	}
	res = i2c_smbus_write_byte_data(file, ALL_LED_OFF_L, (__u8)(off & 0xFF));
	if(res != 0) {
		printf("Failed to set all pwm 3 %d\r\n", res);
		exit(1);
	}
	res = i2c_smbus_write_byte_data(file, ALL_LED_OFF_H, (__u8)(off >> 8));
	if(res != 0) {
		printf("Failed to set all pwm 4 %d\r\n", res);
		exit(1);
	}
}


