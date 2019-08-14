/*----------------------------------------------------------------------------*/
//
//
//	WiringPi ODROID-XU3/XU4 Board Control file
//
//
/*----------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <sys/mman.h>
#include <sys/utsname.h>

/*----------------------------------------------------------------------------*/
#include "softPwm.h"
#include "softTone.h"

/*----------------------------------------------------------------------------*/
#include "wiringPi.h"
#include "odroidxu3.h"

/*----------------------------------------------------------------------------*/
// wiringPi gpio map define
/*----------------------------------------------------------------------------*/
static const int pinToGpio[64] = {
	// wiringPi number to native gpio number
	174,173,	//  0 |  1 : GPA0.3(UART_0.CTSN), GPA0.2(UART_0.RTSN)
	21,  22,	//  2 |  3 : GPX1.5, GPX1.6
	19,  23,	//  4 |  5 : GPX1.3, GPX1.7
	24,  18,	//  6 |  7 : GPX2.0, GPX1.2
	209,210,	//  8 |  9 : GPB3.2(I2C_1.SDA), GPB3.3(I2C_1.SCL)
	190, 25,	// 10 | 11 : GPA2.5(SPI_1.CSN), GPX2.1
	192,191,	// 12 | 13 : GPA2.7(SPI_1.MOSI), GPA2.6(SPI_1.MISO)
	189,172,	// 14 | 15 : GPA2.4(SPI_1.SCLK), GPA0.1(UART_0.TXD)
	171, -1,	// 16 | 17 : GPA0.0(UART_0.RXD),
	-1,  -1,	// 18 | 19
	-1,  28,	// 20 | 21 :  , GPX2.4
	30,  31,	// 22 | 23 : GPX2.6, GPX2.7
	-1,  -1,	// 24 | 25   PWR_ON(INPUT), ADC_0.AIN0
	29,  33,	// 26 | 27 : GPX2.5, GPX3.1
	-1,  -1,	// 28 | 29 : REF1.8V OUT, ADC_0.AIN3
	187,188,	// 30 | 31 : GPA2.2(I2C_5.SDA), GPA2.3(I2C_5.SCL)

	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 32...47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 48...63
};

static const int phyToGpio[64] = {
	// physical header pin number to native gpio number
	-1,		//  0
	-1,  -1,	//  1 |  2 : 3.3V, 5.0V
	209, -1,	//  3 |  4 : GPB3.2(I2C_1.SDA), 5.0V
	210, -1,	//  5 |  6 : GPB3.3(I2C_1.SCL), GND
	18, 172,	//  7 |  8 : GPX1.2, GPA0.1(UART_0.TXD)
	-1, 171,	//  9 | 10 : GND, GPA0.0(UART_0.RXD)
	174,173,	// 11 | 12 : GPA0.3(UART_0.CTSN), GPA0.2(UART_0.RTSN)
	21,  -1,	// 13 | 14 : GPX1.5, GND
	22,  19,	// 15 | 16 : GPX1.6, GPX1.3
	-1,  23,	// 17 | 18 : 3.3V, GPX1.7
	192, -1,	// 19 | 20 : GPA2.7(SPI_1.MOSI), GND
	191, 24,	// 21 | 22 : GPA2.6(SPI_1.MISO), GPX2.0
	189,190,	// 23 | 24 : GPA2.4(SPI_1.SCLK), GPA2.5(SPI_1.CSN)
	-1,  25,	// 25 | 26 : GND, GPX2.1
	187,188,	// 27 | 28 : GPA2.2(I2C_5.SDA), GPA2.4(I2C_5.SCL)
	28,  -1,	// 29 | 30 : GPX2.4, GND
	30,  29,	// 31 | 32 : GPX2.6, GPX2.5
	31,  -1,	// 33 | 34 : GPX2.7, GND
	-1,  33,	// 35 | 36 : PWR_ON(INPUT), GPX3.1
	-1,  -1,	// 37 | 38 : ADC_0.AIN0, 1.8V REF OUT
	-1,  -1,	// 39 | 40 : GND, AADC_0.AIN3

	// Not used
	-1, -1, -1, -1, -1, -1, -1, -1,	// 41...48
	-1, -1, -1, -1, -1, -1, -1, -1,	// 49...56
	-1, -1, -1, -1, -1, -1, -1	// 57...63
};

/*----------------------------------------------------------------------------*/
//
// Global variable define
//
/*----------------------------------------------------------------------------*/
/* ADC file descriptor */
static int adcFds[2];

/* GPIO mmap control */
static volatile uint32_t *gpio, *gpio1;

/* wiringPi Global library */
static struct libodroid	*lib = NULL;

/*----------------------------------------------------------------------------*/
// Function prototype define
/*----------------------------------------------------------------------------*/
static int	gpioToGPLEVReg	(int pin);
static int	gpioToPUPDReg	(int pin);
static int	gpioToShiftReg	(int pin);
static int	gpioToGPFSELReg	(int pin);
static int	gpioToDSReg	(int pin);

/*----------------------------------------------------------------------------*/
// wiringPi core function
/*----------------------------------------------------------------------------*/
static int		_getModeToGpio		(int mode, int pin);
static int		_setPadDrive		(int pin, int value);
static int		_getPadDrive		(int pin);
static int		_pinMode		(int pin, int mode);
static int		_getAlt			(int pin);
static int		_getPUPD		(int pin);
static int		_pullUpDnControl	(int pin, int pud);
static int		_digitalRead		(int pin);
static int		_digitalWrite		(int pin, int value);
static int		_analogRead		(int pin);
static int		_digitalWriteByte	(const unsigned int value);
static unsigned int	_digitalReadByte	(void);

/*----------------------------------------------------------------------------*/
// board init function
/*----------------------------------------------------------------------------*/
static 	void init_gpio_mmap	(void);
static 	void init_adc_fds	(void);
	void init_odroidxu3 	(struct libodroid *libwiring);

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Input regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToGPLEVReg (int pin)
{
	switch (pin) {
	case	XU3_GPIO_X1_START...XU3_GPIO_X1_END:
		return  (XU3_GPIO_X1_DAT_OFFSET >> 2);
	case	XU3_GPIO_X2_START...XU3_GPIO_X2_END:
		return  (XU3_GPIO_X2_DAT_OFFSET >> 2);
	case	XU3_GPIO_X3_START...XU3_GPIO_X3_END:
		return  (XU3_GPIO_X3_DAT_OFFSET >> 2);
	case	XU3_GPIO_A0_START...XU3_GPIO_A0_END:
		return  (XU3_GPIO_A0_DAT_OFFSET >> 2);
	case	XU3_GPIO_A2_START...XU3_GPIO_A2_END:
		return  (XU3_GPIO_A2_DAT_OFFSET >> 2);
	case	XU3_GPIO_B3_START...XU3_GPIO_B3_END:
		return  (XU3_GPIO_B3_DAT_OFFSET >> 2);
	default:
	break;
	}
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Pull up/down regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToPUPDReg (int pin)
{
	switch (pin) {
	case	XU3_GPIO_X1_START...XU3_GPIO_X1_END:
		return  (XU3_GPIO_X1_PUD_OFFSET >> 2);
	case	XU3_GPIO_X2_START...XU3_GPIO_X2_END:
		return  (XU3_GPIO_X2_PUD_OFFSET >> 2);
	case	XU3_GPIO_X3_START...XU3_GPIO_X3_END:
		return  (XU3_GPIO_X3_PUD_OFFSET >> 2);
	case	XU3_GPIO_A0_START...XU3_GPIO_A0_END:
		return  (XU3_GPIO_A0_PUD_OFFSET >> 2);
	case	XU3_GPIO_A2_START...XU3_GPIO_A2_END:
		return  (XU3_GPIO_A2_PUD_OFFSET >> 2);
	case	XU3_GPIO_B3_START...XU3_GPIO_B3_END:
		return  (XU3_GPIO_B3_PUD_OFFSET >> 2);
	default:
		break;
	}
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO bit
//
/*----------------------------------------------------------------------------*/
static int gpioToShiftReg (int pin)
{
	switch (pin) {
	case	XU3_GPIO_X1_START...XU3_GPIO_X1_END:
		return  (pin - XU3_GPIO_X1_START);
	case	XU3_GPIO_X2_START...XU3_GPIO_X2_END:
		return  (pin - XU3_GPIO_X2_START);
	case	XU3_GPIO_X3_START...XU3_GPIO_X3_END:
		return  (pin - XU3_GPIO_X3_START);
	case	XU3_GPIO_A0_START...XU3_GPIO_A0_END:
		return  (pin - XU3_GPIO_A0_START);
	case	XU3_GPIO_A2_START...XU3_GPIO_A2_END:
		return  (pin - XU3_GPIO_A2_START);
	case	XU3_GPIO_B3_START...XU3_GPIO_B3_END:
		return  (pin - XU3_GPIO_B3_START);
	default:
		break;
	}
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Function register
//
/*----------------------------------------------------------------------------*/
static int gpioToGPFSELReg (int pin)
{
	switch (pin) {
	case	XU3_GPIO_X1_START...XU3_GPIO_X1_END:
		return  (XU3_GPIO_X1_CON_OFFSET >> 2);
	case	XU3_GPIO_X2_START...XU3_GPIO_X2_END:
		return  (XU3_GPIO_X2_CON_OFFSET >> 2);
	case	XU3_GPIO_X3_START...XU3_GPIO_X3_END:
		return  (XU3_GPIO_X3_CON_OFFSET >> 2);
	case	XU3_GPIO_A0_START...XU3_GPIO_A0_END:
		return  (XU3_GPIO_A0_CON_OFFSET >> 2);
	case	XU3_GPIO_A2_START...XU3_GPIO_A2_END:
		return  (XU3_GPIO_A2_CON_OFFSET >> 2);
	case	XU3_GPIO_B3_START...XU3_GPIO_B3_END:
		return  (XU3_GPIO_B3_CON_OFFSET >> 2);
	default:
		break;
	}
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Drive Strength register
//
/*----------------------------------------------------------------------------*/
static int gpioToDSReg (int pin)
{
	switch (pin) {
	case	XU3_GPIO_X1_START...XU3_GPIO_X1_END:
		return  (XU3_GPIO_X1_DRV_OFFSET >> 2);
	case	XU3_GPIO_X2_START...XU3_GPIO_X2_END:
		return  (XU3_GPIO_X2_DRV_OFFSET >> 2);
	case	XU3_GPIO_X3_START...XU3_GPIO_X3_END:
		return  (XU3_GPIO_X3_DRV_OFFSET >> 2);
	case	XU3_GPIO_A0_START...XU3_GPIO_A0_END:
		return  (XU3_GPIO_A0_DRV_OFFSET >> 2);
	case	XU3_GPIO_A2_START...XU3_GPIO_A2_END:
		return  (XU3_GPIO_A2_DRV_OFFSET >> 2);
	case	XU3_GPIO_B3_START...XU3_GPIO_B3_END:
		return  (XU3_GPIO_B3_DRV_OFFSET >> 2);
	default:
		break;
	}
	return	-1;
}

/*----------------------------------------------------------------------------*/
static int _getModeToGpio (int mode, int pin)
{
	switch (mode) {
	/* Native gpio number */
	case	MODE_GPIO:
		return	pin;
	/* Native gpio number for sysfs */
	case	MODE_GPIO_SYS:
		return	lib->sysFds[pin] != -1 ? pin : -1;
	/* wiringPi number */
	case	MODE_PINS:
		return	pin < 64 ? pinToGpio[pin] : -1;
	/* header pin number */
	case	MODE_PHYS:
		return	pin < 64 ? phyToGpio[pin] : -1;
	default	:
		break;
	}
	msg(MSG_WARN, "%s : Unknown Mode %d\n", __func__, mode);
	return	-1;
}

/*----------------------------------------------------------------------------*/
static int _setPadDrive (int pin, int value)
{
	int ds, shift;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	if (value < 0 || value > 3) {
		msg(MSG_WARN, "%s : Invalid value %d (Must be 0 ~ 3)\n", __func__, value);
		return -1;
	}

	ds    = gpioToDSReg(pin);
	shift = gpioToShiftReg(pin) << 1;

	if (pin < 100) {
		*(gpio  + ds) &= ~(0b11 << shift);
		*(gpio  + ds) |= (value << shift);
	} else {
		*(gpio1 + ds) &= ~(0b11 << shift);
		*(gpio1 + ds) |= (value << shift);
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int _getPadDrive (int pin)
{
	int ds, shift;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	ds    = gpioToDSReg(pin);
	shift = gpioToShiftReg(pin) << 1;

	if (pin < 100)
		return (*(gpio  + ds) >> shift) & 0b11;
	else
		return (*(gpio1 + ds) >> shift) & 0b11;
}

/*----------------------------------------------------------------------------*/
static int _pinMode (int pin, int mode)
{
	int fsel, shift, origPin = pin;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	softPwmStop  (origPin);
	softToneStop (origPin);

	fsel  = gpioToGPFSELReg(pin);
	shift = gpioToShiftReg (pin) << 2;

	switch (mode) {
	case	INPUT:
		if(pin < 100) {
			*(gpio  + fsel) &= ~(0xF << shift);
		} else {
			*(gpio1 + fsel) &= ~(0xF << shift);
		}
		_pullUpDnControl(origPin, PUD_OFF);
		break;
	case	OUTPUT:
		if(pin < 100) {
			*(gpio  + fsel) &= ~(0xF << shift);
			*(gpio  + fsel) |=  (0x1 << shift);
		} else {
			*(gpio1 + fsel) &= ~(0xF << shift);
			*(gpio1 + fsel) |=  (0x1 << shift);
		}
		break;
	case	INPUT_PULLUP:
		if(pin < 100) {
			*(gpio  + fsel) &= ~(0xF << shift);
		} else {
			*(gpio1 + fsel) &= ~(0xF << shift);
		}
		_pullUpDnControl(origPin, PUD_UP);
		break;
	case	INPUT_PULLDOWN:
		if(pin < 100) {
			*(gpio  + fsel) &= ~(0xF << shift);
		} else {
			*(gpio1 + fsel) &= ~(0xF << shift);
		}
		_pullUpDnControl(origPin, PUD_DOWN);
		break;
	case	SOFT_PWM_OUTPUT:
		softPwmCreate (origPin, 0, 100);
		break;
	case	SOFT_TONE_OUTPUT:
		softToneCreate (origPin);
		break;
	default:
		msg(MSG_WARN, "%s : Unknown Mode %d\n", __func__, mode);
		return -1;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int _getAlt (int pin)
{
	int fsel, shift, mode;

	if (lib->mode == MODE_GPIO_SYS)
		return	-1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return	-1;

	fsel  = gpioToGPFSELReg(pin);
	shift = gpioToShiftReg(pin) << 2;

	if (pin < 100)	// GPX0,1,2,3
		mode = (*(gpio  + fsel) >> shift) & 0xF;
	else		// GPA0,1,2, GPB0,1,2,3,4
		mode = (*(gpio1 + fsel) >> shift) & 0xF;

	// If mode is bigger than 8 including EXT_INT(0xF), it returns ALT7
	return	mode <= 8 ? mode : 8;
}

/*----------------------------------------------------------------------------*/
static int _getPUPD (int pin)
{
	int pupd, shift, pull;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	pupd  = gpioToPUPDReg(pin);
	shift = gpioToShiftReg(pin) << 1;

	if (pin < 100)
		pull = (*(gpio  + pupd) >> shift) & 0x3;
	else
		pull = (*(gpio1 + pupd) >> shift) & 0x3;

	// Pull up when 0x3, down when 0x1
	return	pull == 0 ? 0 : (pull == 0x3 ? 1 : 2);
}

/*----------------------------------------------------------------------------*/
static int _pullUpDnControl (int pin, int pud)
{
	int shift = 0;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	shift = gpioToShiftReg(pin) << 1;

	if (pud) {
		if (pin < 100) {
			*(gpio  + gpioToPUPDReg(pin)) &= ~(0x3 << shift);
			if (pud == PUD_UP)
				*(gpio  + gpioToPUPDReg(pin)) |= (0x3 << shift);
			else
				*(gpio  + gpioToPUPDReg(pin)) |= (0x1 << shift);
		} else {
			*(gpio1 + gpioToPUPDReg(pin)) &= ~(0x3 << shift);
			if (pud == PUD_UP)
				*(gpio1 + gpioToPUPDReg(pin)) |= (0x3 << shift);
			else
				*(gpio1 + gpioToPUPDReg(pin)) |= (0x1 << shift);
		}
	} else {
		// Disable Pull/Pull-down resister
		if (pin < 100)
			*(gpio  + gpioToPUPDReg(pin)) &= ~(0x3 << shift);
		else
			*(gpio1 + gpioToPUPDReg(pin)) &= ~(0x3 << shift);
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int _digitalRead (int pin)
{
	char c ;

	if (lib->mode == MODE_GPIO_SYS) {
		if (lib->sysFds[pin] == -1)
			return -1;

		lseek	(lib->sysFds[pin], 0L, SEEK_SET);
		if (read(lib->sysFds[pin], &c, 1) < 0) {
			msg(MSG_WARN, "%s: Failed with reading from sysfs GPIO node. \n", __func__);
			return -1;
		}

		return	(c == '0') ? LOW : HIGH;
	}

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return	-1;

	if (pin < 100)
		return	*(gpio  + gpioToGPLEVReg(pin)) & (1 << gpioToShiftReg(pin)) ? HIGH : LOW;
	else
		return	*(gpio1 + gpioToGPLEVReg(pin)) & (1 << gpioToShiftReg(pin)) ? HIGH : LOW;
}

/*----------------------------------------------------------------------------*/
static int _digitalWrite (int pin, int value)
{
	if (lib->mode == MODE_GPIO_SYS) {
		if (lib->sysFds[pin] != -1) {
			if (value == LOW) {
				if (write(lib->sysFds[pin], "0\n", 2) < 0)
					msg(MSG_WARN, "%s: Failed with reading from sysfs GPIO node. \n", __func__);
			} else {
				if (write(lib->sysFds[pin], "1\n", 2) < 0)
					msg(MSG_WARN, "%s: Failed with reading from sysfs GPIO node. \n", __func__);
			}
		}
		return -1;
	}

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	if (pin < 100) {
		if (value == LOW)
			*(gpio  + gpioToGPLEVReg(pin)) &= ~(1 << gpioToShiftReg(pin));
		else
			*(gpio  + gpioToGPLEVReg(pin)) |=  (1 << gpioToShiftReg(pin));
	} else {
		if (value == LOW)
			*(gpio1 + gpioToGPLEVReg(pin)) &= ~(1 << gpioToShiftReg(pin));
		else
			*(gpio1 + gpioToGPLEVReg(pin)) |=  (1 << gpioToShiftReg(pin));
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int _analogRead (int pin)
{
	char value[5] = {0,};

	if (lib->mode == MODE_GPIO_SYS)
		return	-1;

	/* wiringPi ADC number = pin 25, pin 29 */
	switch (pin) {
#if defined(ARDUINO)
	/* To work with physical analog channel numbering */
	case	0:	case	25:
		pin = 0;
	break;
	case	3:	case	29:
		pin = 1;
	break;
#else
	case	0:	case	25:
		pin = 0;
	break;
	case	1:	case	29:
		pin = 1;
	break;
#endif
	default:
		return	0;
	}
	if (adcFds [pin] == -1)
		return 0;

	lseek(adcFds [pin], 0L, SEEK_SET);
	if (read(adcFds [pin], &value[0], 4) < 0) {
		msg(MSG_WARN, "%s: Error occurs when it reads from ADC file descriptor. \n", __func__);
		return -1;
	}

	return	atoi(value);
}

/*----------------------------------------------------------------------------*/
static int _digitalWriteByte (const unsigned int value)
{
	union	reg_bitfield	gpx1, gpx2, gpa0;

	if (lib->mode == MODE_GPIO_SYS) {
		return -1;
	}
	/* Read data register */
	gpx1.wvalue = *(gpio  + (XU3_GPIO_X1_DAT_OFFSET >> 2));
	gpx2.wvalue = *(gpio  + (XU3_GPIO_X2_DAT_OFFSET >> 2));
	gpa0.wvalue = *(gpio1 + (XU3_GPIO_A0_DAT_OFFSET >> 2));

	/* Wiring PI GPIO0 = XU3/4 GPA0.3 */
	gpa0.bits.bit3 = (value & 0x01);
	/* Wiring PI GPIO1 = XU3/4 GPA0.2 */
	gpa0.bits.bit2 = (value & 0x02);
	/* Wiring PI GPIO2 = XU3/4 GPX1.5 */
	gpx1.bits.bit5 = (value & 0x04);
	/* Wiring PI GPIO3 = XU3/4 GPX1.6 */
	gpx1.bits.bit6 = (value & 0x08);
	/* Wiring PI GPIO4 = XU3/4 GPX1.3 */
	gpx1.bits.bit3 = (value & 0x10);
	/* Wiring PI GPIO5 = XU3/4 GPX1.7 */
	gpx1.bits.bit7 = (value & 0x20);
	/* Wiring PI GPIO6 = XU3/4 GPX2.0 */
	gpx2.bits.bit0 = (value & 0x40);
	/* Wiring PI GPIO7 = XU3/4 GPX1.2 */
	gpx1.bits.bit2 = (value & 0x80);

	/* update data register */
	*(gpio  + (XU3_GPIO_X1_DAT_OFFSET >> 2)) = gpx1.wvalue;
	*(gpio  + (XU3_GPIO_X2_DAT_OFFSET >> 2)) = gpx2.wvalue;
	*(gpio1 + (XU3_GPIO_A0_DAT_OFFSET >> 2)) = gpa0.wvalue;

	return 0;
}

/*----------------------------------------------------------------------------*/
static unsigned int _digitalReadByte (void)
{
	union reg_bitfield	gpx1, gpx2, gpa0;
	unsigned int		value = 0;

	if (lib->mode == MODE_GPIO_SYS) {
		return	-1;
	}
	/* Read data register */
	gpx1.wvalue = *(gpio  + (XU3_GPIO_X1_DAT_OFFSET >> 2));
	gpx2.wvalue = *(gpio  + (XU3_GPIO_X2_DAT_OFFSET >> 2));
	gpa0.wvalue = *(gpio1 + (XU3_GPIO_A0_DAT_OFFSET >> 2));

	/* Wiring PI GPIO0 = XU3/4 GPA0.3 */
	if (gpa0.bits.bit3)
		value |= 0x01;
	/* Wiring PI GPIO1 = XU3/4 GPA0.2 */
	if (gpa0.bits.bit2)
		value |= 0x02;
	/* Wiring PI GPIO2 = XU3/4 GPX1.5 */
	if (gpx1.bits.bit5)
		value |= 0x04;
	/* Wiring PI GPIO3 = XU3/4 GPX1.6 */
	if (gpx1.bits.bit6)
		value |= 0x08;
	/* Wiring PI GPIO4 = XU3/4 GPX1.3 */
	if (gpx1.bits.bit3)
		value |= 0x10;
	/* Wiring PI GPIO5 = XU3/4 GPX1.7 */
	if (gpx1.bits.bit7)
		value |= 0x20;
	/* Wiring PI GPIO6 = XU3/4 GPX2.0 */
	if (gpx2.bits.bit0)
		value |= 0x40;
	/* Wiring PI GPIO7 = XU3/4 GPX1.2 */
	if (gpx1.bits.bit2)
		value |= 0x80;

	return	value;
}

/*----------------------------------------------------------------------------*/
static void init_gpio_mmap (void)
{
	int fd = -1;
	void *mapped[2];

	/* GPIO mmap setup */
	if (!getuid()) {
		if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
			msg (MSG_ERR,
				"wiringPiSetup: Unable to open /dev/mem: %s\n",
				strerror (errno));
	} else {
		if (access("/dev/gpiomem",0) == 0) {
			if ((fd = open ("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
				msg (MSG_ERR,
					"wiringPiSetup: Unable to open /dev/gpiomem: %s\n",
					strerror (errno));

			setUsingGpioMem( TRUE );
		} else
			msg (MSG_ERR,
				"wiringPiSetup: /dev/gpiomem doesn't exist. Please try again with sudo.\n");
	}

	if (fd < 0) {
		msg(MSG_ERR, "wiringPiSetup: Cannot open memory area for GPIO use. \n");
	} else {
		//#define ODROIDXU_GPX_BASE   0x13400000  // GPX0,1,2,3
		mapped[0] = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, XU3_GPX_BASE);
		//#define ODROIDXU_GPA_BASE   0x14010000  // GPA0,1,2, GPB0,1,2,3,4
		mapped[1] = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, XU3_GPA_BASE);

		if (mapped[0] == MAP_FAILED || mapped[1] == MAP_FAILED) {
			msg(MSG_ERR, "wiringPiSetup: mmap (GPIO) failed: %s \n", strerror (errno));
		} else {
			gpio  = (uint32_t *) mapped[0];
			gpio1 = (uint32_t *) mapped[1];
		}
	}
}

/*----------------------------------------------------------------------------*/
static void init_adc_fds (void)
{
	const char *AIN0_NODE, *AIN1_NODE;
	struct utsname uname_buf;

	/* ADC node setup */
	uname(&uname_buf);
	if (strncmp(uname_buf.release, "4.14", 4) == 0) {
		AIN0_NODE = "/sys/devices/platform/soc/12d10000.adc/iio:device0/in_voltage0_raw";
		AIN1_NODE = "/sys/devices/platform/soc/12d10000.adc/iio:device0/in_voltage3_raw";
	} else if (strncmp(uname_buf.release, "4.9", 3) == 0) {
		AIN0_NODE = "/sys/devices/platform/soc:/12d10000.adc:/iio:device0/in_voltage0_raw";
		AIN1_NODE = "/sys/devices/platform/soc:/12d10000.adc:/iio:device0/in_voltage3_raw";
	} else { // 3.10 kernel
		AIN0_NODE = "/sys/devices/12d10000.adc/iio:device0/in_voltage0_raw";
		AIN1_NODE = "/sys/devices/12d10000.adc/iio:device0/in_voltage3_raw";
	}
	adcFds[0] = open(AIN0_NODE, O_RDONLY);
	adcFds[1] = open(AIN1_NODE, O_RDONLY);
}

/*----------------------------------------------------------------------------*/
void init_odroidxu3 (struct libodroid *libwiring)
{
	init_gpio_mmap();

	init_adc_fds();

	/* wiringPi Core function initialize */
	libwiring->getModeToGpio	= _getModeToGpio;
	libwiring->setPadDrive		= _setPadDrive;
	libwiring->getPadDrive		= _getPadDrive;
	libwiring->pinMode		= _pinMode;
	libwiring->getAlt		= _getAlt;
	libwiring->getPUPD		= _getPUPD;
	libwiring->pullUpDnControl	= _pullUpDnControl;
	libwiring->digitalRead		= _digitalRead;
	libwiring->digitalWrite		= _digitalWrite;
	libwiring->analogRead		= _analogRead;
	libwiring->digitalWriteByte	= _digitalWriteByte;
	libwiring->digitalReadByte	= _digitalReadByte;

	/* specify pin base number */
	libwiring->pinBase		= XU3_GPIO_PIN_BASE;

	/* global variable setup */
	lib = libwiring;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
