/*
 * wiringPiSPI.c:
 *	Simplified SPI access routines
 *	Copyright (c) 2012-2015 Gordon Henderson
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


#include <stdint.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <linux/spi/spidev.h>

#include "wiringPi.h"
#include "wiringPiSPI.h"


// The SPI bus parameters
//	Variables as they need to be passed as pointers later on

static const char       *spiDevType0    = "/dev/spidev0.";
static const char       *spiDevType1    = "/dev/spidev1.";
static const uint8_t     spiBPW         = 8;
static const uint16_t    spiDelay       = 0;

static uint32_t    spiSpeeds [8];
static int         spiFds [8];


/*
 * wiringPiSPIGetFd:
 *	Return the file-descriptor for the given channel
 *********************************************************************************
 */

int wiringPiSPIGetFd (int channel)
{
  return spiFds [channel & 0x7] ;
}


/*
 * wiringPiSPIDataRW:
 *	Write and Read a block of data over the SPI bus.
 *	Note the data ia being read into the transmit buffer, so will
 *	overwrite it!
 *	This is also a full-duplex operation.
 *********************************************************************************
 */

int wiringPiSPIDataRW (int channel, unsigned char *data, int len)
{
  struct spi_ioc_transfer spi ;

  channel &= 0x7 ;

// Mentioned in spidev.h but not used in the original kernel documentation
//	test program )-:

  memset (&spi, 0, sizeof (spi)) ;

  spi.tx_buf        = (unsigned long)data ;
  spi.rx_buf        = (unsigned long)data ;
  spi.len           = len ;
  spi.delay_usecs   = spiDelay ;
  spi.speed_hz      = spiSpeeds [channel] ;
  spi.bits_per_word = spiBPW ;

  return ioctl (spiFds [channel], SPI_IOC_MESSAGE(1), &spi) ;
}

/*
 * wiringPiSPISetupInterface:
 *	Open the SPI device, and set it up, with the mode, etc.
 *********************************************************************************
 */

int wiringPiSPISetupInterface	(const char *device, int channel, int speed, int mode)
{
	int fd ;

	channel &= 0x7;
	mode    &= 3;

	if ((fd = open (device, O_RDWR)) < 0)
		return wiringPiFailure (WPI_ALMOST,
			"Unable to open %s: %s\n",device , strerror (errno));

	spiSpeeds [channel] = speed ;
	spiFds    [channel] = fd ;

	// Set SPI parameters.
	if (ioctl (fd, SPI_IOC_WR_MODE, &mode) < 0)
		return wiringPiFailure (WPI_ALMOST,
			"SPI Mode Change failure: %s\n", strerror (errno));

	if (ioctl (fd, SPI_IOC_WR_BITS_PER_WORD, &spiBPW) < 0)
		return wiringPiFailure (WPI_ALMOST,
			"SPI BPW Change failure: %s\n", strerror (errno));

	if (ioctl (fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
		return wiringPiFailure (WPI_ALMOST,
			"SPI Speed Change failure: %s\n", strerror (errno));

	return fd ;
}

/*
 * wiringPiSPISetupMode:
 *	Open the SPI device, and set it up, with the mode, etc.
 *********************************************************************************
 */

int wiringPiSPISetupMode (int channel, int speed, int mode)
{
	char device[25];
	int model, temp;

	piBoardId (&model, &temp, &temp, &temp, &temp) ;

	switch(model)	{
	case MODEL_ODROID_C2:
		return wiringPiFailure (WPI_ALMOST,
			"ODROID C2 does not support hardware SPI. Check out the SPI bitbang and use wiringPiSPISetupInterface.\n");
	case MODEL_ODROID_HC4:
		return wiringPiFailure (WPI_ALMOST,
			"ODROID HC4 does not support hardware SPI.\n");
	case MODEL_ODROID_C1:
	case MODEL_ODROID_N2:
	case MODEL_ODROID_C4:
	case MODEL_ODROID_M1:
	case MODEL_ODROID_M1S:
	case MODEL_ODROID_M2:
	case MODEL_ODROID_C5:
		sprintf(device, "%s%d", spiDevType0, channel);
	break;
	case MODEL_ODROID_XU3:
		if (cmpKernelVersion(KERN_NUM_TO_MAJOR, 5))
			sprintf(device, "%s%d", spiDevType0, channel);
		else
			sprintf(device, "%s%d", spiDevType1, channel);
	break;
	case MODEL_ODROID_N1:
		sprintf(device, "%s%d", spiDevType1, channel);
	break;
	}

	return wiringPiSPISetupInterface(device, channel, speed, mode);
}

/*
 * wiringPiSPISetup:
 *	Open the SPI device, and set it up, etc. in the default MODE 0
 *********************************************************************************
 */

int wiringPiSPISetup (int channel, int speed)
{
  return wiringPiSPISetupMode (channel, speed, 0) ;
}
