/*
 * readall.c:
 *	The readall functions - getting a bit big, so split them out.
 *	Copyright (c) 2012-2017 Gordon Henderson
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <wiringPi.h>
/*----------------------------------------------------------------------------*/

extern int wpMode ;

/*----------------------------------------------------------------------------*/
#ifndef TRUE
#  define 	TRUE	(1==1)
#  define	FALSE	(1==2)
#endif

/*----------------------------------------------------------------------------*/
/*
 * doReadallExternal:
 *	A relatively crude way to read the pins on an external device.
 *	We don't know the input/output mode of pins, but we can tell
 *	if it's an analog pin or a digital one...
 */
/*----------------------------------------------------------------------------*/
static void doReadallExternal (void)
{
	int pin ;

	printf ("+------+---------+--------+\n") ;
	printf ("|  Pin | Digital | Analog |\n") ;
	printf ("+------+---------+--------+\n") ;

	for (pin = wiringPiNodes->pinBase ; pin <= wiringPiNodes->pinMax ; ++pin)
		printf ("| %4d |  %4d   |  %4d  |\n", pin, digitalRead (pin), analogRead (pin)) ;

	printf ("+------+---------+--------+\n") ;
}

/*----------------------------------------------------------------------------*/
static const char *alts [] =
{
	"IN", "OUT",
	"ALT1", "ALT2", "ALT3", "ALT4", "ALT5", "ALT6", "ALT7",
	"ALT8", "ALT9", "ALTa", "ALTb", "ALTc", "ALTd", "ALTe",
} ;

static const char *pupd [] =
{
	"DSBLD", "P/U", "P/D"
} ;

/*----------------------------------------------------------------------------*/
static const int physToWpi [64] =
{
	-1,	// 0
	-1, -1,	// 1, 2
	 8, -1,
	 9, -1,
	 7, 15,
	-1, 16,
	 0,  1,
	 2, -1,
	 3,  4,
	-1,  5,
	12, -1,
	13,  6,
	14, 10,
	-1, 11,	// 25, 26
	30, 31,	// Actually I2C, but not used
	21, -1,
	22, 26,
	23, -1,
	24, 27,
	25, 28,
	-1, 29,
	-1, -1,
	-1, -1,
	-1, -1,
	-1, -1,
	-1, -1,
	17, 18,
	19, 20,
	-1, -1, -1, -1, -1, -1, -1, -1, -1
} ;

/*----------------------------------------------------------------------------*/
static const int physToWpiHC4 [64] =
{
	-1,	// 0
	-1,  0,	// 1, 2
	 1,  2,
	-1,
} ;

/*----------------------------------------------------------------------------*/
static const char *physNamesOdroidC1All [64] =
{
	NULL,

	"    3.3V", "5V      ",
	"   SDA.1", "5V      ",
	"   SCL.1", "GND(0V) ",
	"GPIO. 83", "TxD1    ",
	" GND(0V)", "RxD1    ",
	"GPIO. 88", "GPIO. 87",
	"GPIO.116", "GND(0V) ",
	"GPIO.115", "GPIO.104",
	"    3.3V", "GPIO.102",
	"    MOSI", "GND(0V) ",
	"    MISO", "GPIO.103",
	"    SCLK", "CE0     ",
	" GND(0V)", "GPIO.118",
	"   SDA.2", "SCL.2   ",
	"GPIO.101", "GND(0V) ",
	"GPIO.100", "GPIO. 99",
	"GPIO.108", "GND(0V) ",
	"GPIO.97 ", "GPIO. 98",
	"   AIN.1", "1V8     ",
	" GND(0V)", "AIN.0   ",

	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,
};

/*----------------------------------------------------------------------------*/
static const char *physNamesOdroidC1 [64] =
{
	NULL,

	"   3.3V", "5V     ",
	"  SDA.1", "5V     ",
	"  SCL.1", "0V     ",
	" IO. 83", "TxD1   ",
	"     0V", "RxD1   ",
	" IO. 88", "IO. 87 ",
	" IO.116", "0V     ",
	" IO.115", "IO.104 ",
	"   3.3V", "IO.102 ",
	"   MOSI", "0V     ",
	"   MISO", "IO.103 ",
	"   SCLK", "CE0    ",
	"     0V", "IO.118 ",
	"  SDA.2", "SCL.2  ",
	" IO.101", "0V     ",
	" IO.100", "IO. 99 ",
	" IO.108", "0V     ",
	" IO.97 ", "IO. 98 ",
	"  AIN.1", "1V8    ",
	"     0V", "AIN.0  ",

	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,
};

/*----------------------------------------------------------------------------*/
static const char *physNamesOdroidC2All_Rev2 [64] =
{
	NULL,

	"    3.3V", "5V      ",
	"   SDA.1", "5V      ",
	"   SCL.1", "GND(0V) ",
	"GPIO.249", "TxD1    ",
	" GND(0V)", "RxD1    ",
	"GPIO.247", "GPIO.238",
	"GPIO.239", "GND(0V) ",
	"GPIO.237", "GPIO.236",
	"    3.3V", "GPIO.233",
	"GPIO.235", "GND(0V) ",
	"GPIO.232", "GPIO.231",
	"GPIO.230", "GPIO.229",
	" GND(0V)", "GPIO.225",
	"   SDA.2", "SCL.2   ",
	"GPIO.228", "GND(0V) ",
	"GPIO.219", "GPIO.224",
	"GPIO.234", "GND(0V) ",
	"GPIO.214", "GPIO.218",
	"   AIN.1", "1V8     ",
	" GND(0V)", "AIN.0   ",

	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,
};

/*----------------------------------------------------------------------------*/
static const char *physNamesOdroidC2_Rev2 [64] =
{
	NULL,

	"   3.3V", "5V     ",
	"  SDA.1", "5V     ",
	"  SCL.1", "0V     ",
	" IO.249", "TxD1   ",
	"     0V", "RxD1   ",
	" IO.247", "IO.238 ",
	" IO.239", "0V     ",
	" IO.237", "IO.236 ",
	"   3.3V", "IO.233 ",
	" IO.235", "0V     ",
	" IO.232", "IO.231 ",
	" IO.230", "IO.229 ",
	"     0V", "IO.225 ",
	"  SDA.2", "SCL.2  ",
	" IO.228", "0V     ",
	" IO.219", "IO.224 ",
	" IO.234", "0V     ",
	" IO.214", "IO.218 ",
	"  AIN.1", "1V8    ",
	"     0V", "AIN.0  ",

	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,
} ;

/*----------------------------------------------------------------------------*/
static const char *physNamesOdroidC2All_Rev1 [64] =
{
	NULL,

	"    3.3V", "5V      ",
	"   SDA.1", "5V      ",
	"   SCL.1", "GND(0V) ",
	"GPIO.214", "--------",
	" GND(0V)", "--------",
	"GPIO.219", "GPIO.218",
	"GPIO.247", "GND(0V) ",
	"--------", "GPIO.235",
	"    3.3V", "GPIO.233",
	"GPIO.238", "GND(0V) ",
	"GPIO.237", "GPIO.234",
	"GPIO.236", "GPIO.248",
	" GND(0V)", "GPIO.249",
	"   SDA.2", "SCL.2   ",
	"GPIO.232", "GND(0V) ",
	"GPIO.231", "GPIO.230",
	"GPIO.239", "GND(0V) ",
	"GPIO.228", "GPIO.229",
	"   AIN.1", "1V8     ",
	" GND(0V)", "AIN.0   ",

	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,
};

/*----------------------------------------------------------------------------*/
static const char *physNamesOdroidC2_Rev1 [64] =
{
	NULL,

	"   3.3V", "5V     ",
	"  SDA.1", "5V     ",
	"  SCL.1", "0V     ",
	" IO.214", "-------",
	"     0V", "-------",
	" IO.219", "IO.218 ",
	" IO.247", "0V     ",
	"-------", "IO.235 ",
	"   3.3V", "IO.233 ",
	" IO.238", "0V     ",
	" IO.237", "IO.234 ",
	" IO.236", "IO.248 ",
	"     0V", "IO.249 ",
	"  SDA.2", "SCL.2  ",
	" IO.232", "0V     ",
	" IO.231", "IO.230 ",
	" IO.239", "0V     ",
	" IO.228", "IO.229 ",
	"  AIN.1", "1V8    ",
	"     0V", "AIN.0  ",

	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,
};

/*----------------------------------------------------------------------------*/
static const char *physNamesOdroidXU3All [64] =
{
	NULL,

	"    3.3V", "5V      ",
	"I2C1.SDA", "5V      ",
	"I2C1.SCL", "GND(0V) ",
	"GPIO. 18", "UART0.TX",
	" GND(0V)", "UART0.RX",
	"GPIO.174", "GPIO.173",
	"GPIO. 21", "GND(0V) ",
	"GPIO. 22", "GPIO. 19",
	"    3.3V", "GPIO. 23",
	"    MOSI", "GND(0V) ",
	"    MISO", "GPIO. 24",
	"    SCLK", "CE0     ",
	" GND(0V)", "GPIO. 25",
	"I2C5.SDA", "I2C5.SCL",
	"GPIO. 28", "GND(0V) ",
	"GPIO. 30", "GPIO. 29",
	"GPIO. 31", "GND(0V) ",
	"POWER ON", "GPIO. 33",
	"   AIN.0", "1V8     ",
	" GND(0V)", "AIN.3   ",

	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,
};

/*----------------------------------------------------------------------------*/
static const char *physNamesOdroidXU3 [64] =
{
	NULL,

	"   3.3V", "5V     ",
	"I2C1.DA", "5V     ",
	"I2C1.CL", "0V     ",
	" IO. 18", "UART.TX",
	"     0V", "UART.RX",
	" IO.174", "IO.173 ",
	" IO. 21", "0V     ",
	" IO. 22", "IO. 19 ",
	"   3.3V", "IO. 23 ",
	"   MOSI", "0V     ",
	"   MISO", "IO. 24 ",
	"   SCLK", "CE0    ",
	"     0V", "IO. 25 ",
	"I2C5.DA", "I2C5.CL",
	" IO. 28", "0V     ",
	" IO. 30", "IO. 29 ",
	" IO. 31", "0V     ",
	" PWR ON", "IO. 33 ",
	"  AIN.0", "1V8    ",
	"     0V", "AIN.3  ",

	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,
};

/*----------------------------------------------------------------------------*/
static const char *physNamesOdroidN1All [64] =
{
	NULL,

	"    3.0V", "5V      ",
	"I2C4_SDA", "5V      ",
	"I2C4_SCL", "GND(0V) ",
	"GPIO1A.0", "UART0_TX",
	" GND(0V)", "UART0_RX",
	"GPIO1A.1", "GPIO1A.2",
	"GPIO1A.3", "GND(0V) ",
	"GPIO1A.4", "GPIO1B.5",
	"    3.0V", "GPIO1C.2",
	"SPI1_TXD", "GND(0V) ",
	"SPI1_RXD", "GPIO1D.0",
	"SPI1_CLK", "SPI1_CSN",
	" GND(0V)", "GPIO1C.6",
	"I2C8_SDA", "I2C8_SCL",
	"SPDIF_TX", "GND(0V) ",
	"    PWM1", "GPIO4D.4",
	"GPIO4D.0", "GND(0V) ",
	"GPIO4D.5", "GPIO4D.6",
	"ADC.AIN1", "1V8     ",
	" GND(0V)", "ADC.AIN0",

	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,
};

/*----------------------------------------------------------------------------*/
static const char *physNamesOdroidN1 [64] =
{
	NULL,

	"   3.0V", "5V     ",
	"I2C4.DA", "5V     ",
	"I2C4.CL", "0V     ",
	" IO1A.0", "UART.TX",
	"     0V", "UART.RX",
	" IO1A.1", "IO1A.2 ",
	" IO1A.3", "0V     ",
	" IO1A.4", "IO1B.5 ",
	"   3.0V", "IO1C.2 ",
	"SPI.TXD", "0V     ",
	"SPI.RXD", "IO1D.0 ",
	"SPI.CLK", "SPI.CSN",
	"     0V", "IO1C.6 ",
	"I2C8.DA", "I2C8.CL",
	"SPDIF.T", "0V     ",
	"   PWM1", "IO4D.4 ",
	" IO4D.0", "0V     ",
	" IO4D.5", "IO4D.6 ",
	"   AIN1", "1V8    ",
	"     0V", "AIN0   ",

	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,
};

/*----------------------------------------------------------------------------*/
static const char *physNamesOdroidN2All [64] =
{
	NULL,

	"    3.3V", "5V      ",
	"   SDA.2", "5V      ",
	"   SCL.2", "GND(0V) ",
	"GPIO.473", "TxD1    ",
	" GND(0V)", "RxD1    ",
	"GPIO.479", "GPIO.492",
	"GPIO.480", "GND(0V) ",
	"GPIO.483", "GPIO.476",
	"    3.3V", "GPIO.477",
	"    MOSI", "GND(0V) ",
	"    MISO", "GPIO.478",
	"    SCLK", "CE0     ",
	" GND(0V)", "GPIO.464",
	"   SDA.3", "SCL.3   ",
	"GPIO.490", "GND(0V) ",
	"GPIO.491", "GPIO.472",
	"GPIO.481", "GND(0V) ",
	"GPIO.482", "GPIO.495",
	"   AIN.3", "1V8     ",
	" GND(0V)", "AIN.2   ",

	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,
};

/*----------------------------------------------------------------------------*/
static const char *physNamesOdroidN2 [64] =
{
	NULL,

	"   3.3V", "5V     ",
	"  SDA.2", "5V     ",
	"  SCL.2", "0V     ",
	" IO.473", "TxD1   ",
	"     0V", "RxD1   ",
	" IO.479", "IO.492 ",
	" IO.480", "0V     ",
	" IO.483", "IO.476 ",
	"   3.3V", "IO.477 ",
	"   MOSI", "0V     ",
	"   MISO", "IO.478 ",
	"   SCLK", "CE0    ",
	"     0V", "IO.464 ",
	"  SDA.3", "SCL.3  ",
	" IO.490", "0V     ",
	" IO.491", "IO.472 ",
	" IO.481", "0V     ",
	" IO.482", "IO.495 ",
	"  AIN.3", "1V8    ",
	"     0V", "AIN.2  ",

	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,
};

/*----------------------------------------------------------------------------*/
static const char *physNamesOdroidC4All [64] =
{
	NULL,

	"    3.3V", "5V      ",
	"   SDA.2", "5V      ",
	"   SCL.2", "GND(0V) ",
	"GPIO.481", "TxD1    ",
	" GND(0V)", "RxD1    ",
	"GPIO.479", "GPIO.492",
	"GPIO.480", "GND(0V) ",
	"GPIO.483", "GPIO.476",
	"    3.3V", "GPIO.477",
	"    MOSI", "GND(0V) ",
	"    MISO", "GPIO.478",
	"    SLCK", "SS      ",
	" GND(0V)", "GPIO. 23",
	"   SDA.3", "SCL.3   ",
	"GPIO.490", "GND(0V) ",
	"GPIO.491", "GPIO. 24",
	"GPIO.482", "GND(0V) ",
	"GPIO.495", "GPIO. 22",
	"   AIN.2", "1V8     ",
	" GND(0V)", "AIN.0   ",

	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,
};

/*----------------------------------------------------------------------------*/
static const char *physNamesOdroidC4 [64] =
{
	NULL,

	"   3.3V", "5V     ",
	"  SDA.2", "5V     ",
	"  SCL.2", "0V     ",
	" IO.481", "TxD1   ",
	"     0V", "RxD1   ",
	" IO.479", "IO.492 ",
	" IO.480", "0V     ",
	" IO.483", "IO.476 ",
	"   3.3V", "IO.477 ",
	"   MOSI", "0V     ",
	"   MISO", "IO.478 ",
	"   SLCK", "SS     ",
	"     0V", "IO. 23 ",
	"  SDA.3", "SCL.3  ",
	" IO.490", "0V     ",
	" IO.491", "IO. 24 ",
	" IO.482", "0V     ",
	" IO.495", "IO. 22 ",
	"  AIN.2", "1V8    ",
	"     0V", "AIN.0  ",

	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,
};

/*----------------------------------------------------------------------------*/
static const char *physNamesOdroidHC4All [64] =
{
	NULL,

	"    3.3V",
	"   SDA.2",
	"   SCL.2",
	"GPIO.481",
	"      0V",

	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
};

/*----------------------------------------------------------------------------*/
static const char *physNamesOdroidHC4 [64] =
{
	NULL,

	"   3.3V",
	"  SDA.2",
	"  SCL.2",
	" IO.481",
	"     0V",

	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
};

/*----------------------------------------------------------------------------*/
static const char *physNamesOdroidM1All [64] =
{
	NULL,

	"    3.3V", "5V      ",
	"I2C0_SDA", "5V      ",
	"I2C0_SCL", "GND(0V) ",
	"GPIO0_B6", "UART1_TX",
	" GND(0V)", "UART1_RX",
	"GPIO0_C0", "GPIO3_D0",
	"GPIO0_C1", "GND(0V) ",
	"GPIO3_B2", "GPIO3_C6",
	"    3.3V", "GPIO3_C7",
	"SPI_MOSI", "GND(0V) ",
	"SPI_MISO", "GPIO3_D1",
	" SPI_CLK", "SPI_CS0 ",
	" GND(0V)", "GPIO3_D2",
	"I2C1_SDA", "I2C1_SCL",
	"GPIO4_C1", "GND(0V) ",
	"GPIO4_B6", "GPIO3_D3",
	"GPIO0_B5", "GND(0V) ",
	"GPIO3_D5", "GPIO3_D4",
	"ADC.AIN1", "1V8     ",
	" GND(0V)", "ADC.AIN0",

	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,
};

/*----------------------------------------------------------------------------*/
static const char *physNamesOdroidM1 [64] =
{
	NULL,

	"   3.3V", "5V     ",
	"   SDA0", "5V     ",
	"   SCL0", "0V     ",
	" IO0_B6", "UART.TX",
	"     0V", "UART.RX",
	" IO0_C0", "IO3_D0 ",
	" IO0_C1", "0V     ",
	" IO3_B2", "IO3_C6 ",
	"   3.3V", "IO3_C7 ",
	"SPI.TXD", "0V     ",
	"SPI.RXD", "IO3_D1 ",
	"SPI.CLK", "SPI.CS0",
	"     0V", "IO3_D2 ",
	"   SDA1", "SCL1   ",
	" IO4_C1", "0V     ",
	" IO4_B6", "IO3_D3 ",
	" IO0_B5", "0V     ",
	" IO3_D5", "IO3_D4 ",
	"   AIN1", "1V8    ",
	"     0V", "AIN0   ",

	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,
};

/*----------------------------------------------------------------------------*/
static const char *physNamesOdroidM1SAll [64] =
{
	NULL,

	"    3.3V", "5V      ",
	"I2C0_SDA", "5V      ",
	"I2C0_SCL", "GND(0V) ",
	"GPIO0_B6", "UART1_TX",
	" GND(0V)", "UART1_RX",
	"GPIO0_C0", "GPIO2_A7",
	"GPIO0_C1", "GND(0V) ",
	"GPIO0_C2", "GPIO2_B5",
	"    3.3V", "GPIO2_B6",
	"SPI_MOSI", "GND(0V) ",
	"SPI_MISO", "GPIO2_B0",
	" SPI_CLK", "SPI_CS0 ",
	" GND(0V)", "GPIO2_B1",
	"I2C1_SDA", "I2C1_SCL",
	"GPIO2_C0", "GND(0V) ",
	"GPIO2_B7", "GPIO2_B2",
	"GPIO0_B5", "GND(0V) ",
	"GPIO2_A5", "GPIO2_A6",
	"ADC.AIN1", "1V8     ",
	" GND(0V)", "ADC.AIN0",

	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,
};

/*----------------------------------------------------------------------------*/
static const char *physNamesOdroidM1S [64] =
{
	NULL,

	"   3.3V", "5V     ",
	"   SDA0", "5V     ",
	"   SCL0", "0V     ",
	" IO0_B6", "UART.TX",
	"     0V", "UART.RX",
	" IO0_C0", "IO2_A7 ",
	" IO0_C1", "0V     ",
	" IO0_C2", "IO2_B5 ",
	"   3.3V", "IO2_B6 ",
	"SPI.TXD", "0V     ",
	"SPI.RXD", "IO2_B0 ",
	"SPI.CLK", "SPI.CS0",
	"     0V", "IO2_B1 ",
	"   SDA1", "SCL1   ",
	" IO2_C0", "0V     ",
	" IO2_B7", "IO2_B2 ",
	" IO0_B5", "0V     ",
	" IO2_A5", "IO2_A6 ",
	"   AIN1", "1V8    ",
	"     0V", "AIN0   ",

	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,
};

/*----------------------------------------------------------------------------*/
static const char *physNamesOdroidM2All [64] =
{
	NULL,

	"    3.3V", "5V      ",
	"I2C0_SDA", "5V      ",
	"I2C0_SCL", "GND(0V) ",
	"GPIO0_D0", "UART1_TX",
	" GND(0V)", "UART1_RX",
	"GPIO3_D4", "GPIO3_B2",
	"GPIO3_D5", "GND(0V) ",
	"GPIO4_B3", "GPIO1_B2",
	"    3.3V", "GPIO1_B3",
	"SPI_MOSI", "GND(0V) ",
	"SPI_MISO", "GPIO0_D4",
	" SPI_CLK", "SPI_CS0 ",
	" GND(0V)", "GPIO1_B4",
	"I2C1_SDA", "I2C1_SCL",
	"GPIO1_B7", "GND(0V) ",
	"GPIO1_B6", "GPIO4_B4",
	"GPIO3_D0", "GND(0V) ",
	"GPIO3_C7", "GPIO4_B5",
	"ADC.AIN4", "1V8     ",
	" GND(0V)", "ADC.AIN5",

	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,
};

/*----------------------------------------------------------------------------*/
static const char *physNamesOdroidM2 [64] =
{
	NULL,

	"   3.3V", "5V     ",
	"   SDA0", "5V     ",
	"   SCL0", "0V     ",
	" IO0_D0", "UART.TX",
	"     0V", "UART.RX",
	" IO3_D4", "IO3_B2 ",
	" IO3_D5", "0V     ",
	" IO4_B3", "IO1_B2 ",
	"   3.3V", "IO1_B3 ",
	"SPI.TXD", "0V     ",
	"SPI.RXD", "IO0_D4 ",
	"SPI.CLK", "SPI.CS0",
	"     0V", "IO1_B4 ",
	"   SDA1", "SCL1   ",
	" IO1_B7", "0V     ",
	" IO1_B6", "IO4_B4 ",
	" IO3_D0", "0V     ",
	" IO3_C7", "IO4_B5 ",
	"   AIN4", "1V8    ",
	"     0V", "AIN5   ",

	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,
};

/*----------------------------------------------------------------------------*/
static const char *physNamesOdroidC5All [64] =
{
	NULL,

	"    3.3V", "5V      ",
	"   SDA.0", "5V      ",
	"   SCL.0", "GND(0V) ",
	"GPIOX_17", "TxD.0   ",
	" GND(0V)", "RxD.0   ",
	" GPIOX_5", "GPIOX_14",
	"GPIOX_15", "GND(0V) ",
	" GPIOX_4", "GPIODV_1",
	"    3.3V", "GPIODV_2",
	"    MOSI", "GND(0V) ",
	"    MISO", "GPIOX_6 ",
	"    SLCK", "SS      ",
	" GND(0V)", "GPIOX_7 ",
	"   SDA.1", "SCL.1   ",
	" GPIOH_4", "GND(0V) ",
	" GPIOH_5", "GPIOX_2 ",
	" GPIOX_0", "GND(0V) ",
	" GPIOX_1", "GPIOX_3 ",
	"   AIN.1", "1V8     ",
	" GND(0V)", "AIN.0   ",

	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,
};

/*----------------------------------------------------------------------------*/
static const char *physNamesOdroidC5 [64] =
{
	NULL,

	"   3.3V", "5V     ",
	"  SDA.0", "5V     ",
	"  SCL.0", "0V     ",
	"  IO_D4", "TxD1   ",
	"     0V", "RxD1   ",
	"  IO_X5", "IO_X14 ",
	" IO_X15", "0V     ",
	"  IO_X4", "IO_DV1 ",
	"   3.3V", "IO_DV2 ",
	"   MOSI", "0V     ",
	"   MISO", "IOX_6  ",
	"   SLCK", "SS     ",
	"     0V", "IOX_7  ",
	"  SDA.1", "SCL.1  ",
	"  IO_H4", "0V     ",
	"  IO_H5", "IOX_2  ",
	"  IO_X0", "0V     ",
	"  IO_X1", "IOX_3  ",
	"  AIN.1", "1V8    ",
	"     0V", "AIN.0  ",

	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
	NULL,NULL,NULL,
};

/*----------------------------------------------------------------------------*/

static void readallPhys(int model, int UNU rev, int physPin, const char *physNames[], int isAll) {
	int pin ;

	// GPIO, wPi pin number
	if (isAll == TRUE) {
		if ((physPinToGpio (physPin) == -1) && (physToWpi [physPin] == -1))
			printf(" |      |    ");
		else if (physPinToGpio (physPin) != -1) {
			printf(" |  %3d | %3d", physPinToGpio(physPin), physToWpi[physPin]);
		} else
			printf(" |      | %3d", physToWpi [physPin]);
	} else {
		if ((physPinToGpio (physPin) == -1) && (physToWpi [physPin] == -1))
			printf(" |     |    ");
		else if (physPinToGpio (physPin) != -1) {
			printf(" | %3d | %3d", physPinToGpio(physPin), physToWpi[physPin]);
		} else
			printf(" |     | %3d", physToWpi [physPin]);
	}

	// GPIO pin name
	printf (" | %s", physNames [physPin]) ;

	// GPIO pin mode, value
	if ((physToWpi [physPin] == -1) || (physPinToGpio (physPin) == -1)) {
		printf(" |      |  ");
		if (isAll == TRUE)
			printf(" |    |      ");
	} else {
		if (wpMode == MODE_GPIO)
			pin = physPinToGpio (physPin);
		else if (wpMode == MODE_PHYS)
			pin = physPin ;
		else
			pin = physToWpi [physPin];

		printf (" | %4s", alts [getAlt (pin)]) ;
		printf (" | %d", digitalRead (pin)) ;

		// GPIO pin drive strength, pu/pd
		if (isAll == TRUE) {
			switch (model) {
			case MODEL_ODROID_N1:
				printf (" |    |      ");
				break;
			case MODEL_ODROID_C1:
			case MODEL_ODROID_C2:
				printf (" |    | %5s", pupd[getPUPD(pin)]);
				break;
			case MODEL_ODROID_XU3:
			case MODEL_ODROID_N2:
			case MODEL_ODROID_C4:
			case MODEL_ODROID_M1:
			case MODEL_ODROID_M1S:
			case MODEL_ODROID_M2:
			case MODEL_ODROID_C5:
				printf (" | %2d | %5s", getDrive(pin), pupd[getPUPD(pin)]);
				break;
			default:
				break;
			}
		}
	}

	// Physical pin number
	printf (" | %2d", physPin) ;
	++physPin ;
	printf (" || %-2d", physPin) ;

	// GPIO pin mode, value
	if ((physToWpi [physPin] == -1) || (physPinToGpio (physPin) == -1)) {
		printf(" |");
		if (isAll == TRUE)
			printf("       |    |");
		printf("   |     ");
	} else {
		if (wpMode == MODE_GPIO)
			pin = physPinToGpio (physPin);
		else if (wpMode == MODE_PHYS)
			pin = physPin ;
		else
			pin = physToWpi [physPin];


		// GPIO pin drive strength, pu/pd
		if (isAll == TRUE) {
			switch (model) {
			case MODEL_ODROID_N1:
				printf (" |       |   ");
				break;
			case MODEL_ODROID_C1:
			case MODEL_ODROID_C2:
				printf (" | %-5s |   ", pupd[getPUPD(pin)]);
				break;
			case MODEL_ODROID_XU3:
			case MODEL_ODROID_N2:
			case MODEL_ODROID_C4:
			case MODEL_ODROID_M1:
			case MODEL_ODROID_M1S:
			case MODEL_ODROID_M2:
			case MODEL_ODROID_C5:
				printf (" | %-5s | %-2d", pupd[getPUPD(pin)], getDrive(pin));
				break;
			default:
				break;
			}
		}
		printf(" | %d", digitalRead (pin));
		printf(" | %-4s", alts [getAlt (pin)]);
	}

	// GPIO pin name
	printf (" | %-6s", physNames [physPin]);

	// GPIO, wPi pin number
	if (isAll == TRUE) {
		if ((physPinToGpio (physPin) == -1) && (physToWpi [physPin] == -1))
			printf(" |     |     ");
		else if (physPinToGpio (physPin) != -1)
			printf(" | %-3d | %-3d ", physToWpi [physPin], physPinToGpio (physPin));
		else
			printf(" | %-3d |     ", physToWpi [physPin]);
	} else {
		if ((physPinToGpio (physPin) == -1) && (physToWpi [physPin] == -1))
			printf(" |     |    ");
		else if (physPinToGpio (physPin) != -1)
			printf(" | %-3d | %-3d", physToWpi [physPin], physPinToGpio (physPin));
		else
			printf(" | %-3d |    ", physToWpi [physPin]);
	}

	printf (" |\n") ;
}

static void printHeader(const char *headerName, int isAll) {
	const char *headerLeft = " +-----+-----+---------+------+---+";
	const char *headerRight = "+---+------+---------+-----+-----+\n";
	const char *headerLeftAll = " +------+-----+----------+------+---+----+";
	const char *headerRightAll = "+----+---+------+----------+-----+------+\n";

	(isAll == FALSE) ? printf("%s", headerLeft) : printf("%s", headerLeftAll);
	printf("%s", headerName);
	(isAll == FALSE) ? printf("%s", headerRight) : printf("%s", headerRightAll);
}

/*----------------------------------------------------------------------------*/
static void printBody(int model, int rev, const char *physNames[], int isAll) {
	(isAll == FALSE)
		? printf(
			" | I/O | wPi |   Name  | Mode | V | Physical | V | Mode |  Name   | wPi | I/O |\n"
			" +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n")
		: printf(
			" | GPIO | wPi |   Name   | Mode | V | DS | PU/PD | Physical | PU/PD | DS | V | Mode |   Name   | wPi | GPIO |\n"
			" +------+-----+----------+------+---+----+-------+----++----+-------+----+---+------+----------+-----+------+\n");
	for (int pin = 1; pin <= 40; pin += 2)
		readallPhys(model, rev, pin, physNames, isAll);
	(isAll == FALSE)
		? printf(
			" +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n"
			" | I/O | wPi |   Name  | Mode | V | Physical | V | Mode |  Name   | wPi | I/O |\n")
		: printf(
			" +------+-----+----------+------+---+----+-------+----++----+-------+----+---+------+----------+-----+------+\n"
			" | GPIO | wPi |   Name   | Mode | V | DS | PU/PD | Physical | PU/PD | DS | V | Mode |   Name   | wPi | GPIO |\n");
}

/*----------------------------------------------------------------------------*/
static void readallPhysHC4(int model, int UNU rev, int physPin, const char *physNames[], int isAll) {
	int pin ;

	// GPIO, wPi pin number
	if (isAll == TRUE) {
		if ((physPinToGpio (physPin) == -1) && (physToWpiHC4 [physPin] == -1))
			printf(" |      |    ");
		else if (physPinToGpio (physPin) != -1) {
			printf(" |  %3d | %3d", physPinToGpio(physPin), physToWpiHC4[physPin]);
		} else
			printf(" |      | %3d", physToWpiHC4 [physPin]);
	} else {
		if ((physPinToGpio (physPin) == -1) && (physToWpiHC4 [physPin] == -1))
			printf(" |     |    ");
		else if (physPinToGpio (physPin) != -1) {
			printf(" | %3d | %3d", physPinToGpio(physPin), physToWpiHC4[physPin]);
		} else
			printf(" |     | %3d", physToWpiHC4 [physPin]);
	}

	// GPIO pin name
	printf (" | %s", physNames [physPin]) ;

	// GPIO pin mode, value
	if ((physToWpiHC4 [physPin] == -1) || (physPinToGpio (physPin) == -1)) {
		printf(" |      |  ");
		if (isAll == TRUE)
			printf(" |    |      ");
	} else {
		if (wpMode == MODE_GPIO)
			pin = physPinToGpio (physPin);
		else if (wpMode == MODE_PHYS)
			pin = physPin ;
		else
			pin = physToWpiHC4 [physPin];

		printf (" | %4s", alts [getAlt (pin)]) ;
		printf (" | %d", digitalRead (pin)) ;

		// GPIO pin drive strength, pu/pd
		if (isAll == TRUE) {
			switch (model) {
			case MODEL_ODROID_HC4:
				printf (" | %2d | %5s", getDrive(pin), pupd[getPUPD(pin)]);
				break;
			default:
				break;
			}
		}
	}

	// Physical pin number
	printf(" |  %2d |\n", physPin);
}

/*----------------------------------------------------------------------------*/
static void printHeaderHC4(const char *headerName, int isAll) {
	const char *header = " +-----+-----+---------+------+---+-----+";
	const char *headerAll = " +------+-----+----------+------+---+----+-------+-----+";

	(isAll == FALSE) ? printf("%s\n", header) : printf("%s\n", headerAll);
	(isAll == FALSE)
		? printf(" |              %s              |\n", headerName)
		: printf(" |             %s              |\n", headerName);
	(isAll == FALSE) ? printf("%s\n", header) : printf("%s\n", headerAll);
}

/*----------------------------------------------------------------------------*/
static void printBodyHC4(int model, int rev, const char *physNames[], int isAll) {
	(isAll == FALSE)
		? printf(
			" | I/O | wPi |   Name  | Mode | V | Phy |\n"
			" +-----+-----+---------+------+---+-----+\n")
		: printf(
			" | GPIO | wPi |   Name   | Mode | V | DS | PU/PD | Phy |\n"
			" +------+-----+----------+------+---+----+-------+-----+\n");
	for (int pin = 1; pin <= 5; pin ++)
		readallPhysHC4(model, rev, pin, physNames, isAll);
	(isAll == FALSE)
		? printf(
			" +-----+-----+---------+------+---+-----+\n"
			" | I/O | wPi |   Name  | Mode | V | Phy |\n")
		: printf(
			" +------+-----+----------+------+---+----+-------+-----+\n"
			" | GPIO | wPi |   Name   | Mode | V | DS | PU/PD | Phy |\n");
}

/*----------------------------------------------------------------------------*/
/*
 * doReadall:
 *	Read all the GPIO pins
 *	We also want to use this to read the state of pins on an externally
 *	connected device, so we need to do some fiddling with the internal
 *	wiringPi node structures - since the gpio command can only use
 *	one external device at a time, we'll use that to our advantage...
 */
/*----------------------------------------------------------------------------*/
void doReadall(int argc, char *argv[]) {
	int model, rev, mem, maker, overVolted, isAll;
	char *headerName, *physNames;

	// External readall
	if (wiringPiNodes != NULL) {
		doReadallExternal();
		return;
	}

	if (argc <= 2) {
		isAll = FALSE;
	} else if (argc == 3 && (strcasecmp(argv[2], "-a") == 0 || strcasecmp(argv[2], "--all") == 0)) {
		isAll = TRUE;
	} else {
		printf("Oops - unknown readall option:\n");
		for (int i = 3; i < argc + 1; i++)
			printf("\targv[%d]: %s\n", i, argv[i - 1]);

		return;
	}

	piBoardId (&model, &rev, &mem, &maker, &overVolted);

	switch (model) {
		case MODEL_ODROID_C1:
			headerName = (isAll == FALSE) ? "--- C1 ---" : "---- Model  ODROID-C1 ----";
			physNames = (char *) ((isAll == FALSE) ? physNamesOdroidC1 : physNamesOdroidC1All);
			break;
		case MODEL_ODROID_C2:
			headerName = (isAll == FALSE) ? "--- C2 ---" : "---- Model  ODROID-C2 ----";
			if (rev == 1)
				physNames = (char *) ((isAll == FALSE) ? physNamesOdroidC2_Rev1 : physNamesOdroidC2All_Rev1);
			else
				physNames = (char *) ((isAll == FALSE) ? physNamesOdroidC2_Rev2 : physNamesOdroidC2All_Rev2);
			break;
		case MODEL_ODROID_XU3:
			headerName = (isAll == FALSE) ? "- XU3, 4 -" : "--- Model ODROID-XU3/4 ---";
			physNames = (char *) ((isAll == FALSE) ? physNamesOdroidXU3 : physNamesOdroidXU3All);
			break;
		case MODEL_ODROID_N1:
			headerName = (isAll == FALSE) ? "--- N1 ---" : "---- Model  ODROID-N1 ----";
			physNames = (char *) ((isAll == FALSE) ? physNamesOdroidN1 : physNamesOdroidN1All);
			break;
		case MODEL_ODROID_N2:
			headerName = (isAll == FALSE) ? "--- N2 ---" : "---- Model  ODROID-N2 ----";
			physNames = (char *) ((isAll == FALSE) ? physNamesOdroidN2 : physNamesOdroidN2All);
			break;
		case MODEL_ODROID_C4:
			headerName = (isAll == FALSE) ? "--- C4 ---" : "---- Model  ODROID-C4 ----";
			physNames = (char *) ((isAll == FALSE) ? physNamesOdroidC4 : physNamesOdroidC4All);
			break;
		case MODEL_ODROID_HC4:
			headerName = (isAll == FALSE) ? "   HC4    " : "     Model ODROID-HC4     ";
			physNames = (char *) ((isAll == FALSE) ? physNamesOdroidHC4 : physNamesOdroidHC4All);
			break;
		case MODEL_ODROID_M1:
			headerName = (isAll == FALSE) ? "--- M1 ---" : "---- Model  ODROID-M1 ----";
			physNames = (char *) ((isAll == FALSE) ? physNamesOdroidM1 : physNamesOdroidM1All);
			break;
		case MODEL_ODROID_M1S:
			headerName = (isAll == FALSE) ? "    M1S   " : "     Model ODROID-M1S     ";
			physNames = (char *) ((isAll == FALSE) ? physNamesOdroidM1S : physNamesOdroidM1SAll);
			break;
		case MODEL_ODROID_M2:
			headerName = (isAll == FALSE) ? "--- M2 ---" : "---- Model ODROID-M2 ----";
			physNames = (char *) ((isAll == FALSE) ? physNamesOdroidM2 : physNamesOdroidM2All);
			break;
		case MODEL_ODROID_C5:
			headerName = (isAll == FALSE) ? "--- C5 ---" : "---- Model ODROID-C5 ----";
			physNames = (char *) ((isAll == FALSE) ? physNamesOdroidC5 : physNamesOdroidC5All);
		break;
		default:
			printf("Oops - unknown model: %d\n", model);
			return;
	}

	switch (model) {
		case MODEL_ODROID_HC4:
			printHeaderHC4((const char *) headerName, isAll);
			printBodyHC4(model, rev, (const char **) physNames, isAll);
			printHeaderHC4((const char *) headerName, isAll);
			break;
		default:
			printHeader((const char *) headerName, isAll);
			printBody(model, rev, (const char **) physNames, isAll);
			printHeader((const char *) headerName, isAll);
			break;
	}
}

/*----------------------------------------------------------------------------*/
/*
 * doAllReadall:
 *	Force reading of all pins regardless of Pi model
 */
/*----------------------------------------------------------------------------*/
void doAllReadall(void) {
	char *fakeArgv[3] = { "", "", "--all" };

	doReadall(3, fakeArgv);
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
