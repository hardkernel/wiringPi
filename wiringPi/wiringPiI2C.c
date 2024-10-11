/*
 * wiringPiI2C.c:
 *	Simplified I2C access routines
 *	Copyright (c) 2013 Gordon Henderson
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with wiringPi.
 *    If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

/*
 * Notes:
 *	The Linux I2C code is actually the same (almost) as the SMBus code.
 *	SMBus is System Management Bus - and in essentially I2C with some
 *	additional functionality added, and stricter controls on the electrical
 *	specifications, etc. however I2C does work well with it and the
 *	protocols work over both.
 *
 *	I'm directly including the SMBus functions here as some Linux distros
 *	lack the correct header files, and also some header files are GPLv2
 *	rather than the LGPL that wiringPi is released under - presumably because
 *	originally no-one expected I2C/SMBus to be used outside the kernel -
 *	however enter the Raspberry Pi with people now taking directly to I2C
 *	devices without going via the kernel...
 *
 *	This may ultimately reduce the flexibility of this code, but it won't be
 *	hard to maintain it and keep it current, should things change.
 *
 *	Information here gained from: kernel/Documentation/i2c/dev-interface
 *	as well as other online resources.
 *********************************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "wiringPi.h"
#include "wiringPiI2C.h"

uint8_t fdToSlaveAddress[1024] = { 0xFF };

static inline int i2c_smbus_access (int fd, char rw, uint8_t command, int size, union i2c_smbus_data *data)
{
  struct i2c_smbus_ioctl_data args ;

  args.read_write = rw ;
  args.command    = command ;
  args.size       = size ;
  args.data       = data ;
  return ioctl (fd, I2C_SMBUS, &args) ;
}

/*
 * wiringPiI2CRead:
 *	Simple device read
 *********************************************************************************
 */

int wiringPiI2CRead (int fd)
{
  union i2c_smbus_data data ;

  if (i2c_smbus_access (fd, I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &data))
    return -1 ;
  else
    return data.byte & 0xFF ;
}


/*
 * wiringPiI2CReadReg8: wiringPiI2CReadReg16:
 *	Read an 8 or 16-bit value from a regisiter on the device
 *********************************************************************************
 */

int wiringPiI2CReadReg8 (int fd, int reg)
{
  union i2c_smbus_data data;

  if (i2c_smbus_access (fd, I2C_SMBUS_READ, reg, I2C_SMBUS_BYTE_DATA, &data))
    return -1 ;
  else
    return data.byte & 0xFF ;
}

int wiringPiI2CReadReg16 (int fd, int reg)
{
  union i2c_smbus_data data;

  if (i2c_smbus_access (fd, I2C_SMBUS_READ, reg, I2C_SMBUS_WORD_DATA, &data))
    return -1 ;
  else
    return data.word & 0xFFFF ;
}


/*
 * wiringPiI2CReadBlock:
 *	Read values from consecutive regisiters on the device
 *********************************************************************************
 */
int wiringPiI2CReadBlock (int fd, int reg, uint8_t *buff, int size)
{
	struct i2c_rdwr_ioctl_data	i2c;
	struct i2c_msg 			msgs[2];

	uint8_t reg_addr[1] = { reg };

	msgs[0].addr	= fdToSlaveAddress[fd];
	msgs[0].flags	= 0;
	msgs[0].len	= 1;
	msgs[0].buf	= reg_addr;

	msgs[1].addr	= fdToSlaveAddress[fd];
	msgs[1].flags	= I2C_M_RD;
	msgs[1].len	= size;
	msgs[1].buf	= buff;

	i2c.msgs	= msgs;
	i2c.nmsgs	= 2;

	return ioctl( fd, I2C_RDWR, &i2c );
}


/*
 * wiringPiI2CWrite:
 *	Simple device write
 *********************************************************************************
 */

int wiringPiI2CWrite (int fd, int data)
{
  return i2c_smbus_access (fd, I2C_SMBUS_WRITE, data, I2C_SMBUS_BYTE, NULL) ;
}


/*
 * wiringPiI2CWriteReg8: wiringPiI2CWriteReg16:
 *	Write an 8 or 16-bit value to the given register
 *********************************************************************************
 */

int wiringPiI2CWriteReg8 (int fd, int reg, int value)
{
  union i2c_smbus_data data ;

  data.byte = value ;
  return i2c_smbus_access (fd, I2C_SMBUS_WRITE, reg, I2C_SMBUS_BYTE_DATA, &data) ;
}

int wiringPiI2CWriteReg16 (int fd, int reg, int value)
{
  union i2c_smbus_data data ;

  data.word = value ;
  return i2c_smbus_access (fd, I2C_SMBUS_WRITE, reg, I2C_SMBUS_WORD_DATA, &data) ;
}


/*
 * wiringPiI2CReadBlock:
 *	Write values from consecutive regisiters on the device
 *********************************************************************************
 */
int wiringPiI2CWriteBlock (int fd, int reg, uint8_t *buff, int size)
{
	uint8_t temp[50];
	temp[0] = reg;
	memcpy(&temp[1], buff, size);

	struct i2c_rdwr_ioctl_data	i2c;
	struct i2c_msg			msgs;

	msgs.addr	= fdToSlaveAddress[fd];
	msgs.flags	= 0;
	msgs.len	= size + 1;
	msgs.buf	= temp;

	i2c.msgs	= &msgs;
	i2c.nmsgs	= 1;

	return ioctl( fd, I2C_RDWR, &i2c );
}


/*
 * wiringPiI2CSetupInterface:
 *	Open the I2C device, and regisiter the target device
 *********************************************************************************
 */

int wiringPiI2CSetupInterface (const char *device, int devId)
{
	int fd;
	if ((fd = open (device, O_RDWR)) < 0) {
		return wiringPiFailure (WPI_ALMOST, "Unable to open I2C device: %s\n", strerror (errno)) ;
	}

	if (ioctl (fd, I2C_SLAVE, devId) < 0) {
		return wiringPiFailure (WPI_ALMOST, "Unable to select I2C device: %s\n", strerror (errno)) ;
	}

	fdToSlaveAddress[fd] = devId;

	return fd ;
}


/*
 * wiringPiI2CSetup:
 *	Open the I2C device, and regisiter the target device
 *********************************************************************************
 */

int wiringPiI2CSetup (const int devId)
{
	int model, rev, mem, maker, overVolted ;
	const char *device = NULL;

	piBoardId (&model, &rev, &mem, &maker, &overVolted) ;

	switch(model)	{
	case MODEL_ODROID_C1:
	case MODEL_ODROID_C2:
		if (cmpKernelVersion(KERN_NUM_TO_MAJOR, 4))
			device = "/dev/i2c-0";
		else
			device = "/dev/i2c-1";
	break;
	case MODEL_ODROID_XU3:
		if (cmpKernelVersion(KERN_NUM_TO_MAJOR, 5))
			device = "/dev/i2c-0";
		else
			device = "/dev/i2c-1";
	break;
	case MODEL_ODROID_N1:
		device = "/dev/i2c-4";
	break;
	case MODEL_ODROID_N2:
	case MODEL_ODROID_C4:
	case MODEL_ODROID_HC4:
	case MODEL_ODROID_M1:
		if (cmpKernelVersion(KERN_NUM_TO_REVISION, 4, 9, 230))
			device = "/dev/i2c-0";
		else
			device = "/dev/i2c-2";
	break;
	case MODEL_ODROID_M1S:
	case MODEL_ODROID_M2:
		device = "/dev/i2c-0";
	case MODEL_ODROID_C5:
		device = "/dev/i2c-0";
	break;
	}

	return wiringPiI2CSetupInterface (device, devId) ;
}
