#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "i2c.h"

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
