/*----------------------------------------------------------------------------*/
//
//
//	WiringPi ODROID-C0/C1/C1+ Board Control file (AMLogic 32Bits Platform)
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
#include "odroidc1.h"

/*----------------------------------------------------------------------------*/
// wiringPi gpio map define
/*----------------------------------------------------------------------------*/
static const int pinToGpio[64] = {
	// wiringPi number to native gpio number
	88,  87,	//  0 |  1 : GPIOY.8, GPIOY.7
	116, 115,	//  2 |  3 : GPIOX.19, GPIOX.18
	104, 102,	//  4 |  5 : GPIOX.7, GPIOX.5
	103, 83,	//  6 |  7 : GPIOX.6, GPIOY.3
	74,  75,	//  8 |  9 : GPIODV.24(I2CA_SDA), GPIODV.25(I2CA_SCL)
	117, 118,	// 10 | 11 : GPIOX.20(SPI_CE0), GPIOX.21(SPI_CE1)
	107, 106,	// 12 | 13 : GPIOX.10(SPI_MOSI), GPIOX.9(SPI_MISO)
	105, -1,	// 14 | 15 : GPIOX.8(SPI_SCLK), GPIOX.16(UART_TX_B)
	-1,  -1,	// 16 | 17 : GPIOX.17(UART_RX_B),
	-1,  -1,	// 18 | 19 :
	-1,  101,	// 20 | 21 : , GPIOX.4
	100, 108,	// 22 | 23 : GPIOX.3, GPIOX.11
	97,  -1,	// 24 | 25 : GPIOX.0, ADC.AIN1
	99,  98,	// 26 | 27 : GPIOX.2, GPIOX.1
	-1,  -1,	// 28 | 29 : REF1.8V OUT, ADC.AIN0
	-1,  -1,	// 30 | 31 : GPIODV.26(I2CB_SDA), GPIODV.27(I2CB_SCL)

	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 32...47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 48...63
};

static const int phyToGpio[64] = {
	// physical header pin number to native gpio number
	-1,		//  0
	-1,  -1,	//  1 |  2 : 3.3V, 5.0V
	74,  -1,	//  3 |  4 : GPIODV.24(I2CA_SDA), 5.0V
	75,  -1,	//  5 |  6 : GPIODV.25(I2CA_SCL), GND
	83,  -1,	//  7 |  8 : GPIOY.3, GPIOX.16(UART_TX_B)
	-1,  -1,	//  9 | 10 : GND, GPIOX.17(UART_RX_B)
	88,  87,	// 11 | 12 : GPIOY.8, GPIOY.7
	116, -1,	// 13 | 14 : GPIOX.19, GND
	115, 104,	// 15 | 16 : GPIOX.18, GPIOX.7
	-1,  102,	// 17 | 18 : 3.3V, GPIOX.5
	107,  -1,	// 19 | 20 : GPIOX.10(SPI_MOSI), GND
	106, 103,	// 21 | 22 : GPIOX.9(SPI_MISO), GPIOX.6
	105, 117,	// 23 | 24 : GPIOX.8(SPI_SCLK), GPIOX.20(SPI_CE0)
	-1,  118,	// 25 | 26 : GND, GPIOX.21(SPI_CE1)
	-1,  -1,	// 27 | 28 : GPIODV.26(I2CB_SDA), GPIODV.27(I2CB_SCL)
	101, -1,	// 29 | 30 : GPIOX.4, GND
	100, 99,	// 31 | 32 : GPIOX.3, GPIOX.2
	108, -1,	// 33 | 34 : GPIOX.11, GND
	97,  98,	// 35 | 36 : GPIOX.0, GPIOX.1
	-1,  -1,	// 37 | 38 : ADC.AIN1, 1.8V REF OUT
	-1,  -1,	// 39 | 40 : GND, ADC.AIN0

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
static volatile uint32_t *gpio;

/* wiringPi Global library */
static struct libodroid	*lib = NULL;

/*----------------------------------------------------------------------------*/
// Function prototype define
/*----------------------------------------------------------------------------*/
static int	gpioToGPSETReg	(int pin);
static int	gpioToGPLEVReg	(int pin);
static int	gpioToPUENReg	(int pin);
static int	gpioToPUPDReg	(int pin);
static int	gpioToShiftReg	(int pin);
static int	gpioToGPFSELReg	(int pin);

/*----------------------------------------------------------------------------*/
// wiringPi core function
/*----------------------------------------------------------------------------*/
static int		_getModeToGpio		(int mode, int pin);
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

	void init_odroidc1 	(struct libodroid *libwiring);

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Set regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToGPSETReg (int pin)
{
	if (pin >= C1_GPIOX_PIN_START && pin <= C1_GPIOX_PIN_END)
		return  C1_GPIOX_OUTP_REG_OFFSET;
	if (pin >= C1_GPIOY_PIN_START && pin <= C1_GPIOY_PIN_END)
		return  C1_GPIOY_OUTP_REG_OFFSET;
	if (pin >= C1_GPIODV_PIN_START && pin <= C1_GPIODV_PIN_END)
		return  C1_GPIODV_OUTP_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Input regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToGPLEVReg (int pin)
{
	if (pin >= C1_GPIOX_PIN_START && pin <= C1_GPIOX_PIN_END)
		return  C1_GPIOX_INP_REG_OFFSET;
	if (pin >= C1_GPIOY_PIN_START && pin <= C1_GPIOY_PIN_END)
		return  C1_GPIOY_INP_REG_OFFSET;
	if (pin >= C1_GPIODV_PIN_START && pin <= C1_GPIODV_PIN_END)
		return  C1_GPIODV_INP_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Pull up/down enable regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToPUENReg (int pin)
{
	if (pin >= C1_GPIOX_PIN_START && pin <= C1_GPIOX_PIN_END)
		return  C1_GPIOX_PUEN_REG_OFFSET;
	if (pin >= C1_GPIOY_PIN_START && pin <= C1_GPIOY_PIN_END)
		return  C1_GPIOY_PUEN_REG_OFFSET;
	if (pin >= C1_GPIODV_PIN_START && pin <= C1_GPIODV_PIN_END)
		return  C1_GPIODV_PUEN_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Pull up/down regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToPUPDReg (int pin)
{
	if (pin >= C1_GPIOX_PIN_START && pin <= C1_GPIOX_PIN_END)
		return  C1_GPIOX_PUPD_REG_OFFSET;
	if (pin >= C1_GPIOY_PIN_START && pin <= C1_GPIOY_PIN_END)
		return  C1_GPIOY_PUPD_REG_OFFSET;
	if (pin >= C1_GPIODV_PIN_START && pin <= C1_GPIODV_PIN_END)
		return  C1_GPIODV_PUPD_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO bit
//
/*----------------------------------------------------------------------------*/
static int gpioToShiftReg (int pin)
{
	if (pin >= C1_GPIOX_PIN_START && pin <= C1_GPIOX_PIN_END)
		return  pin - C1_GPIOX_PIN_START;
	if (pin >= C1_GPIOY_PIN_START && pin <= C1_GPIOY_PIN_END)
		return  pin - C1_GPIOY_PIN_START;
	if (pin >= C1_GPIODV_PIN_START && pin <= C1_GPIODV_PIN_END)
		return  pin - C1_GPIODV_PIN_START;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Function register
//
/*----------------------------------------------------------------------------*/
static int gpioToGPFSELReg (int pin)
{
	if (pin >= C1_GPIOX_PIN_START && pin <= C1_GPIOX_PIN_END)
		return  C1_GPIOX_FSEL_REG_OFFSET;
	if (pin >= C1_GPIOY_PIN_START && pin <= C1_GPIOY_PIN_END)
		return  C1_GPIOY_FSEL_REG_OFFSET;
	if (pin >= C1_GPIODV_PIN_START && pin <= C1_GPIODV_PIN_END)
		return  C1_GPIODV_FSEL_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
static int _getModeToGpio (int mode, int pin)
{
	int retPin = -1;

	switch (mode) {
	/* Native gpio number */
	case	MODE_GPIO:
		retPin = pin;
		break;
	/* Native gpio number for sysfs */
	case	MODE_GPIO_SYS:
		retPin = lib->sysFds[pin] != -1 ? pin : -1;
		break;
	/* wiringPi number */
	case	MODE_PINS:
		retPin = pin < 64 ? pinToGpio[pin] : -1;
		break;
	/* header pin number */
	case	MODE_PHYS:
		retPin = pin < 64 ? phyToGpio[pin] : -1;
		break;
	default	:
		msg(MSG_WARN, "%s : Unknown Mode %d\n", __func__, mode);
		return -1;
	}

	/* To check I2C module loaded */
	if (retPin == 74 || retPin == 75) {
		if (moduleLoaded(AML_MODULE_I2C))
			return -1;
	}

	return retPin;
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
	shift = gpioToShiftReg (pin);

	switch (mode) {
	case	INPUT:
		*(gpio + fsel) = (*(gpio + fsel) |  (1 << shift));
		_pullUpDnControl(origPin, PUD_OFF);
		break;
	case	OUTPUT:
		*(gpio + fsel) = (*(gpio + fsel) & ~(1 << shift));
		break;
	case 	INPUT_PULLUP:
		*(gpio + fsel) = (*(gpio + fsel) |  (1 << shift));
		_pullUpDnControl(origPin, PUD_UP);
		break;
	case 	INPUT_PULLDOWN:
		*(gpio + fsel) = (*(gpio + fsel) |  (1 << shift));
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
	int fsel, shift;
	int mode = 0;

	if (lib->mode == MODE_GPIO_SYS)
		return	-1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return	-1;

	fsel  = gpioToGPFSELReg(pin);
	shift = gpioToShiftReg(pin);

	switch (pin) {
	case C1_GPIOX_PIN_START  ...C1_GPIOX_PIN_END:
		switch (shift) {
		case	0:
			if (*(gpio  + C1_MUX_REG_8_OFFSET) & (1 << 5))			{ mode = 1; break; }
			if (*(gpio  + C1_MUX_REG_5_OFFSET) & (1 << 14))			{ mode = 2; break; }
			break;
		case	1 ... 3:
			if (*(gpio  + C1_MUX_REG_8_OFFSET) & (1 << (5-shift))) 		{ mode = 1; break; }
			if (*(gpio  + C1_MUX_REG_5_OFFSET) & (1 << 13))			{ mode = 2; break; }
			break;
		case	4 ... 5:
			if ((*(gpio + C1_MUX_REG_5_OFFSET) & (1 << (33 - shift))) &&
			    (*(gpio + C1_MUX_REG_5_OFFSET) & (1 << 12)))		{ mode = 1; break; }
			if (*(gpio  + C1_MUX_REG_3_OFFSET) & (1 << (34 - shift)))	{ mode = 2; break; }
			if (*(gpio  + C1_MUX_REG_4_OFFSET) & (1 << (21 - shift)))	{ mode = 3; break; }
			break;
		case	6 ... 7:
			if ((*(gpio + C1_MUX_REG_5_OFFSET) & (1 << 28)) &&
			    (*(gpio + C1_MUX_REG_5_OFFSET) & (1 << 12)))		{ mode = 1; break; }
			if (*(gpio  + C1_MUX_REG_3_OFFSET) & (1 << 27))			{ mode = 2; break; }
			if (*(gpio  + C1_MUX_REG_4_OFFSET) & (1 << (21 - shift)))	{ mode = 3; break; }
			if (*(gpio  + C1_MUX_REG_5_OFFSET) & (1 << (15 - shift)))	{ mode = 4; break; }
			break;
		case	8:
			if (*(gpio  + C1_MUX_REG_8_OFFSET) & (1 << 1))			{ mode = 1; break; }
			if (*(gpio  + C1_MUX_REG_5_OFFSET) & (1 << 11))			{ mode = 2; break; }
			if (*(gpio  + C1_MUX_REG_6_OFFSET) & (1 << 19))			{ mode = 3; break; }
			if (*(gpio  + C1_MUX_REG_4_OFFSET) & (1 << 22))			{ mode = 4; break; }
			if (*(gpio  + C1_MUX_REG_3_OFFSET) & (1 << 6))			{ mode = 5; break; }
			break;
		case	9:
			if (*(gpio  + C1_MUX_REG_8_OFFSET) & (1 << 0))			{ mode = 1; break; }
			if (*(gpio  + C1_MUX_REG_5_OFFSET) & (1 << 10))			{ mode = 2; break; }
			if (*(gpio  + C1_MUX_REG_6_OFFSET) & (1 << 18))			{ mode = 3; break; }
			if (*(gpio  + C1_MUX_REG_4_OFFSET) & (1 << 24))			{ mode = 4; break; }
			if (*(gpio  + C1_MUX_REG_3_OFFSET) & (1 << 6))			{ mode = 5; break; }
			break;
		case	10:
			if (*(gpio  + C1_MUX_REG_3_OFFSET) & (1 << 22))			{ mode = 1; break; }
			if ((*(gpio + C1_MUX_REG_7_OFFSET) & (1 << 31)) &&
			    (*(gpio + C1_MUX_REG_9_OFFSET) & (1 << 19)))		{ mode = 2; break; }
			if (*(gpio  + C1_MUX_REG_6_OFFSET) & (1 << 17))			{ mode = 3; break; }
			if (*(gpio  + C1_MUX_REG_4_OFFSET) & (1 << 23))			{ mode = 4; break; }
			if (*(gpio  + C1_MUX_REG_3_OFFSET) & (1 << 8))			{ mode = 5; break; }
			break;
		case	11:
			if (*(gpio  + C1_MUX_REG_3_OFFSET) & (1 << 20))			{ mode = 1; break; }
			if (*(gpio  + C1_MUX_REG_7_OFFSET) & (1 << 30))			{ mode = 2; break; }
			if (*(gpio  + C1_MUX_REG_2_OFFSET) & (1 << 3))			{ mode = 5; break; }
			break;
		case	16 ... 17:
			if (*(gpio  + C1_MUX_REG_4_OFFSET) & (1 << (25 - shift)))	{ mode = 1; break; }
			if (*(gpio  + C1_MUX_REG_4_OFFSET) & (1 << (37 - shift)))	{ mode = 4; break; }
			if (*(gpio  + C1_MUX_REG_4_OFFSET) & (1 << (21 - shift)))	{ mode = 5; break; }
			break;
		case	18 ... 19:
			if (*(gpio  + C1_MUX_REG_4_OFFSET) & (1 << (25 - shift)))	{ mode = 1; break; }
			if (*(gpio  + C1_MUX_REG_4_OFFSET) & (1 << (37 - shift)))	{ mode = 4; break; }
			break;
		case	20:
			if (*(gpio  + C1_MUX_REG_6_OFFSET) & (1 << 16))			{ mode = 3; break; }
			if (*(gpio  + C1_MUX_REG_4_OFFSET) & (1 << 25))			{ mode = 4; break; }
			if (*(gpio  + C1_MUX_REG_3_OFFSET) & (1 << 9))			{ mode = 5; break; }
			break;
		}
		break;

	case C1_GPIOY_PIN_START  ...C1_GPIOY_PIN_END:
		switch (shift) {
		case	0 ... 1:
			if (*(gpio  + C1_MUX_REG_3_OFFSET) & (1 << (2 - shift)))	{ mode = 1; break; }
			break;
		case	3:
			if (*(gpio  + C1_MUX_REG_1_OFFSET) & (1 << 7))			{ mode = 2; break; }
			if (*(gpio  + C1_MUX_REG_3_OFFSET) & (1 << 18))			{ mode = 3; break; }
			break;
		case	8:
			if (*(gpio  + C1_MUX_REG_3_OFFSET) & (1 << 0))			{ mode = 1; break; }
			break;
		case	9:
			if (*(gpio  + C1_MUX_REG_3_OFFSET) & (1 << 4))			{ mode = 1; break; }
			break;
		case	6 ... 7:
		case	10 ... 12:
			if (*(gpio  + C1_MUX_REG_3_OFFSET) & (1 << 5))			{ mode = 1; break; }
			break;
		case	13 ... 14:
			if (*(gpio  + C1_MUX_REG_3_OFFSET) & (1 << 5))			{ mode = 1; break; }
			if (*(gpio  + C1_MUX_REG_5_OFFSET) & (1 << (20 - shift)))	{ mode = 3; break; }
			break;
		}
		break;
	case C1_GPIODV_PIN_START...C1_GPIODV_PIN_END:
		switch (shift) {
		case	24 ... 27:
			if (*(gpio  + C1_MUX_REG_6_OFFSET) & (1 << (47 - shift)))	{ mode = 4; break; }
			if (*(gpio  + C1_MUX_REG_9_OFFSET) & (1 << (55 - shift)))	{ mode = 5; break; }
			break;
		}
		break;
	default:
		return -1;
	}

	return	mode ? mode + 1 : (*(gpio + fsel) & (1 << shift)) ? 0 : 1;
}

/*----------------------------------------------------------------------------*/
static int _getPUPD (int pin)
{
	int puen, pupd, shift;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	puen  = gpioToPUENReg(pin);
	pupd  = gpioToPUPDReg(pin);
	shift = gpioToShiftReg(pin);

	if (*(gpio + puen) & (1 << shift))
		return *(gpio + pupd) & (1 << shift) ? 1 : 2;
	else
		return 0;
}

/*----------------------------------------------------------------------------*/
static int _pullUpDnControl (int pin, int pud)
{
	int shift = 0;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	shift = gpioToShiftReg(pin);

	if (pud) {
		// Enable Pull/Pull-down resister
		*(gpio + gpioToPUENReg(pin)) =
			(*(gpio + gpioToPUENReg(pin)) | (1 << shift));

		if (pud == PUD_UP)
			*(gpio + gpioToPUPDReg(pin)) =
				(*(gpio + gpioToPUPDReg(pin)) |  (1 << shift));
		else
			*(gpio + gpioToPUPDReg(pin)) =
				(*(gpio + gpioToPUPDReg(pin)) & ~(1 << shift));
	} else	// Disable Pull/Pull-down resister
		*(gpio + gpioToPUENReg(pin)) =
			(*(gpio + gpioToPUENReg(pin)) & ~(1 << shift));

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

	if ((*(gpio + gpioToGPLEVReg(pin)) & (1 << gpioToShiftReg(pin))) != 0)
		return HIGH ;
	else
		return LOW ;
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

	if (value == LOW)
		*(gpio + gpioToGPSETReg(pin)) &= ~(1 << gpioToShiftReg(pin));
	else
		*(gpio + gpioToGPSETReg(pin)) |=  (1 << gpioToShiftReg(pin));

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
	case	1:	case	25:
		pin = 0;
	break;
	case	0:	case	29:
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
	union	reg_bitfield	gpiox, gpioy;

	gpiox.wvalue = *(gpio + C1_GPIOX_INP_REG_OFFSET);
	gpioy.wvalue = *(gpio + C1_GPIOY_INP_REG_OFFSET);

	/* Wiring PI GPIO0 = C1 GPIOY.8 */
	gpioy.bits.bit8 = (value & 0x01);
	/* Wiring PI GPIO1 = C1 GPIOY.7 */
	gpioy.bits.bit7 = (value & 0x02);
	/* Wiring PI GPIO2 = C1 GPIOX.19 */
	gpiox.bits.bit19 = (value & 0x04);
	/* Wiring PI GPIO3 = C1 GPIOX.18 */
	gpiox.bits.bit18 = (value & 0x08);
	/* Wiring PI GPIO4 = C1 GPIOX.7 */
	gpiox.bits.bit7 = (value & 0x10);
	/* Wiring PI GPIO5 = C1 GPIOX.5 */
	gpiox.bits.bit5 = (value & 0x20);
	/* Wiring PI GPIO6 = C1 GPIOX.6 */
	gpiox.bits.bit6 = (value & 0x40);
	/* Wiring PI GPIO7 = C1 GPIOY.3 */
	gpioy.bits.bit3 = (value & 0x80);

	*(gpio + C1_GPIOX_OUTP_REG_OFFSET) = gpiox.wvalue;
	*(gpio + C1_GPIOY_OUTP_REG_OFFSET) = gpioy.wvalue;

	return 0;
}

/*----------------------------------------------------------------------------*/
static unsigned int _digitalReadByte (void)
{
	union reg_bitfield	gpiox, gpioy;
	unsigned int		value = 0;

	gpiox.wvalue = *(gpio + C1_GPIOX_INP_REG_OFFSET);
	gpioy.wvalue = *(gpio + C1_GPIOY_INP_REG_OFFSET);

	/* Wiring PI GPIO0 = C1 GPIOY.8 */
	if (gpioy.bits.bit8)
		value |= 0x01;
	/* Wiring PI GPIO1 = C1 GPIOY.7 */
	if (gpioy.bits.bit7)
		value |= 0x02;
	/* Wiring PI GPIO2 = C1 GPIOX.19 */
	if (gpiox.bits.bit19)
		value |= 0x04;
	/* Wiring PI GPIO3 = C1 GPIOX.18 */
	if (gpiox.bits.bit18)
		value |= 0x08;
	/* Wiring PI GPIO4 = C1 GPIOX.7 */
	if (gpiox.bits.bit7)
		value |= 0x10;
	/* Wiring PI GPIO5 = C1 GPIOX.5 */
	if (gpiox.bits.bit5)
		value |= 0x20;
	/* Wiring PI GPIO6 = C1 GPIOX.6 */
	if (gpiox.bits.bit6)
		value |= 0x40;
	/* Wiring PI GPIO7 = C1 GPIOY.3 */
	if (gpioy.bits.bit3)
		value |= 0x80;

	return	value;
}

/*----------------------------------------------------------------------------*/
static void init_gpio_mmap (void)
{
	int fd = -1;
	void *mapped;

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
		// #define C1_GPIO_BASE	0xC1108000
#ifdef ANDROID
		mapped = mmap64(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, (off64_t)C1_GPIO_BASE);
#else
		mapped = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, C1_GPIO_BASE);
#endif

		if (mapped == MAP_FAILED)
			msg(MSG_ERR, "wiringPiSetup: mmap (GPIO) failed: %s \n", strerror (errno));
		else
			gpio = (uint32_t *) mapped;
	}
}

/*----------------------------------------------------------------------------*/
static void init_adc_fds (void)
{
	const char *AIN0_NODE, *AIN1_NODE;

	/* ADC node setup */
	AIN0_NODE = "/sys/class/saradc/saradc_ch0";
	AIN1_NODE = "/sys/class/saradc/saradc_ch1";

	adcFds[0] = open(AIN0_NODE, O_RDONLY);
	adcFds[1] = open(AIN1_NODE, O_RDONLY);
}

/*----------------------------------------------------------------------------*/
void init_odroidc1 (struct libodroid *libwiring)
{
	init_gpio_mmap();

	init_adc_fds();

	/* wiringPi Core function initialize */
	libwiring->getModeToGpio	= _getModeToGpio;
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
	libwiring->pinBase		= C1_GPIO_PIN_BASE;

	/* global variable setup */
	lib = libwiring;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

