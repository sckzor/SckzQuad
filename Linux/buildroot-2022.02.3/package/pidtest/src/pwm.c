#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "smbus.h"
#include "pwm.h"
#include "i2c.h"

/*
 * Setup the PWM controller over the I2C bus
 */
int setup_pwm(int adapter_nr) {

	int pwm;
	__s32 res;
	__u8 mode1;

	pwm = instantiate_device(adapter_nr, PWM_ADDRESS); /* Instantiate the PWM device */
	set_all_pwm(pwm, 0, 0);

	res = i2c_smbus_write_byte_data(pwm, MODE2, OUTDRV); /* Configure for totem pole structure so pull up is not necesary */
	if(res != 0) {
		printf("Failed to set MODE1 value\r\n");
		exit(1);
	}

	res = i2c_smbus_write_byte_data(pwm, MODE1, ALLCALL); /* Allow all channels to be treated as a group */
	if(res != 0) {
		printf("Failed to set MODE2 value\r\n");
		exit(1);
	}

	mode1 = i2c_smbus_read_byte_data(pwm, MODE1); /* Read current value of MODE1 */
	mode1 = mode1 & ~SLEEP; /* Reset the sleep to wake the chip up */
	
	res = i2c_smbus_write_byte_data(pwm, MODE1, mode1); /* Take the chip out of sleep */
	if(res != 0) {
		printf("Failed to take the chip out of sleep\r\n");
		exit(1);
	}
	
	return pwm;
}

/*
 * Set the frequency of a period for the PWM signals
 */
void set_pwm_frequency(int file, double hz) {
	double pre_scale_val;
	__s32 res;
	__u8 pre_scale, old_mode, new_mode;

	pre_scale_val = 25000000.0; // 25 Mhz
	pre_scale_val /= 4096; // 12 bit
	pre_scale_val /= hz;
	pre_scale_val -= 1.0;

	pre_scale = (__u8)(floor(pre_scale_val + 0.5));
	old_mode = i2c_smbus_read_byte_data(file, MODE1); /* Read the MODE1 value */

	new_mode = (old_mode & 0x7F) | SLEEP; /* Configure address to go to sleep */
	
	res = i2c_smbus_write_byte_data(file, MODE1, new_mode); /* Put the chip in sleep mode */
	if(res != 0) {
		printf("Failed to make the chip sleep %d\r\n", res);
		exit(1);
	}

	res = i2c_smbus_write_byte_data(file, PRESCALE, pre_scale); /* Set the prescaler for PWM control */
	if(res != 0) {
		printf("Failed to change prescale %d\r\n", res);
		exit(1);
	}

	res = i2c_smbus_write_byte_data(file, MODE1, old_mode); /* Take the chip out of sleep */
	if(res != 0) {
		printf("Failed to take chip out of sleep %d\r\n", res);
		exit(1);
	}

	res = i2c_smbus_write_byte_data(file, MODE1, old_mode | RESTART); /* Restart the chip */
	if(res != 0) {
		printf("Failed to restart the chip %d\r\n", res);
		exit(1);
	}
}

/*
 * Set the PWM signal on a channels 0-15 of the controller
 */
void set_pwm(int file, int channel, int on, int off) {
	__s32 res;
	/* 
	 * "On" is the starting point on the 0-4096 "number line" where the signal starts and "off" is
	 * where it ends.  Most often on starts at 0 and off is the length of the pulse you want
	 */
	if(channel < 16 && channel > -1) { /* Channel must be 0-15 */
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

void set_pwm_us(int file, int channel, int us) {
	int pulse_length;

	pulse_length = (1000000 / 50) / 4096;
	us /= pulse_length;

	set_pwm(file, channel, 0, us);
}



/*
 * Set the PWM signal on all channels of the controller
 */
void set_all_pwm(int file, int on, int off) {
	__s32 res;
	/* 
	 * "On" is the starting point on the 0-4096 "number line" where the signal starts and "off" is
	 * where it ends.  Most often on starts at 0 and off is the length of the pulse you want
	 */
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
