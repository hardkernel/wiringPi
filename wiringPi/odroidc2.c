/*----------------------------------------------------------------------------*/
//
//
//	WiringPi ODROID-C2 Board Control file (AMLogic 64Bits Platform)
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
#include "odroidc2.h"

/*----------------------------------------------------------------------------*/
// wiringPi gpio map define
/*----------------------------------------------------------------------------*/
static const int pinToGpio_rev1[64] = {
	// wiringPi number to native gpio number
	219, 218,	//  0 |  1 : GPIOY.8, GPIOY.7
	247, -1,	//  2 |  3 : GPIOX.19,
	235, 233,	//  4 |  5 : GPIOX.7, GPIOX.5
	234, 214,	//  6 |  7 : GPIOX.6, GPIOY.3
	205, 206,	//  8 |  9 : GPIODV.24(I2CA_SDA), GPIODV.25(I2CA_SCL)
	248, 249,	// 10 | 11 : GPIOX.20, GPIOX.21
	238, 237,	// 12 | 13 : GPIOX.10, GPIOX.9
	236, -1,	// 14 | 15 : GPIOX.8,
	-1,  -1,	// 16 | 17 :
	-1,  -1,	// 18 | 19 :
	-1,  232,	// 20 | 21 : , GPIOX.4
	231, 239,	// 22 | 23 : GPIOX.3, GPIOX.11
	228, -1,	// 24 | 25 : GPIOX.0, ADC.AIN1
	230, 229,	// 26 | 27 : GPIOX.2, GPIOX.1
	-1,  -1,	// 28 | 29 : REF1.8V OUT, ADC.AIN0
	-1,  -1,	// 30 | 31 : GPIODV.26(I2CB_SDA), GPIODV.27(I2CB_SCL)
	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 32...47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 48...63
};

static const int phyToGpio_rev1[64] = {
	// physical header pin number to native gpio number
	-1,		//  0
	-1,  -1,	//  1 |  2 : 3.3V, 5.0V
	205, -1,	//  3 |  4 : GPIODV.24(I2CA_SDA), 5.0V
	206, -1,	//  5 |  6 : GPIODV.25(I2CA_SCL), GND
	214, -1,	//  7 |  8 : GPIOY.3, GPIOX.16(UART_TX_B)
	-1,  -1,	//  9 | 10 : GND, GPIOX.17(UART_RX_B)
	219,218,	// 11 | 12 : GPIOY.8, GPIOY.7
	247, -1,	// 13 | 14 : GPIOX.19, GND
	-1, 235,	// 15 | 16 : GPIOX.18, GPIOX.7
	-1, 233,	// 17 | 18 : 3.3V, GPIOX.5
	238, -1,	// 19 | 20 : GPIOX.10(SPI_MOSI), GND
	237,234,	// 21 | 22 : GPIOX.9(SPI_MISO), GPIOX.6
	236,248,	// 23 | 24 : GPIOX.8(SPI_SCLK), GPIOX.20(SPI_CE0)
	-1, 249,	// 25 | 26 : GND, GPIOX.21(SPI_CE1)
	-1,  -1,	// 27 | 28 : GPIODV.26(I2CB_SDA), GPIODV.27(I2CB_SCL)
	232, -1,	// 29 | 30 : GPIOX.4, GND
	231,230,	// 31 | 32 : GPIOX.3, GPIOX.2
	239, -1,	// 33 | 34 : GPIOX.11, GND
	228,229,	// 35 | 36 : GPIOX.0, GPIOX.1
	-1,  -1,	// 37 | 38 : ADC.AIN1, 1.8V REF OUT
	-1,  -1,	// 39 | 40 : GND, ADC.AIN0
	// Not used
	-1, -1, -1, -1, -1, -1, -1, -1,	// 41...48
	-1, -1, -1, -1, -1, -1, -1, -1,	// 49...56
	-1, -1, -1, -1, -1, -1, -1	// 57...63
};

/*----------------------------------------------------------------------------*/
static const int pinToGpio_rev2[64] = {
	// wiringPi number to native gpio number
	247, 238,	//  0 |  1 : GPIOX.19, GPIOX.10
	239, 237,	//  2 |  3 : GPIOX.11, GPIOX.9
	236, 233,	//  4 |  5 : GPIOX.8, GPIOX.5
	231, 249,	//  6 |  7 : GPIOX.3, GPIOX.21
	205, 206,	//  8 |  9 : GPIODV.24(I2CA_SDA), GPIODV.25(I2CA_SCL)
	229, 225,	// 10 | 11 : GPIOX.1, GPIOY.14
	235, 232,	// 12 | 13 : GPIOX.7(PWM1), GPIOX.4
	230, -1,	// 14 | 15 : GPIOX.2, GPIOX.12(UART_TX_B)
	-1,  -1,	// 16 | 17 : GPIOX.13(UART_RX_B),
	-1,  -1,	// 18 | 19 :
	-1,  228,	// 20 | 21 : , GPIOX.0
	219, 234,	// 22 | 23 : GPIOY.8, GPIOX.6(PWM0)
	214, -1,	// 24 | 25 : GPIOY.3, ADC.AIN1
	224, 218,	// 26 | 27 : GPIOY.13, GPIOY.7
	-1,  -1,	// 28 | 29 : REF1.8V OUT, ADC.AIN0
	-1,  -1,	// 30 | 31 : GPIODV.26(I2CB_SDA), GPIODV.27(I2CB_SCL)
	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 32...47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 48...63
};

static const int phyToGpio_rev2[64] = {
	// physical header pin number to native gpio number
	-1,		//  0
	-1,  -1,	//  1 |  2 : 3.3V, 5.0V
	205, -1,	//  3 |  4 : GPIODV.24(I2CA_SDA), 5.0V
	206, -1,	//  5 |  6 : GPIODV.25(I2CA_SCL), GND
	249, -1,	//  7 |  8 : GPIOX.21, GPIOX.12(UART_TX_B)
	-1,  -1,	//  9 | 10 : GND, GPIOX.13(UART_RX_B)
	247,238,	// 11 | 12 : GPIOX.19, GPIOX.10
	239, -1,	// 13 | 14 : GPIOX.11, GND
	237,236,	// 15 | 16 : GPIOX.9, GPIOX.8
	-1, 233,	// 17 | 18 : 3.3V, GPIOX.5
	235, -1,	// 19 | 20 : GPIOX.7(PWM1), GND
	232,231,	// 21 | 22 : GPIOX.4, GPIOX.3
	230,229,	// 23 | 24 : GPIOX.2, GPIOX.1
	-1, 225,	// 25 | 26 : GND, GPIOY.14
	-1,  -1,	// 27 | 28 : GPIODV.26(I2CB_SDA), GPIODV.27(I2CB_SCL)
	228, -1,	// 29 | 30 : GPIOX.0, GND
	219,224,	// 31 | 32 : GPIOY.8, GPIOY.13
	234, -1,	// 33 | 34 : GPIOX.6(PWM0), GND
	214,218,	// 35 | 36 : GPIOY.3, GPIOY.7
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
// wiringPi Pinmap control arrary
/*----------------------------------------------------------------------------*/
const int *pinToGpio, *phyToGpio;

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

	void init_odroidc2 	(struct libodroid *libwiring);

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Set regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToGPSETReg (int pin)
{
	if (pin >= C2_GPIOX_PIN_START && pin <= C2_GPIOX_PIN_END)
		return  C2_GPIOX_OUTP_REG_OFFSET;
	if (pin >= C2_GPIOY_PIN_START && pin <= C2_GPIOY_PIN_END)
		return  C2_GPIOY_OUTP_REG_OFFSET;
	if (pin >= C2_GPIODV_PIN_START && pin <= C2_GPIODV_PIN_END)
		return  C2_GPIODV_OUTP_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Input regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToGPLEVReg (int pin)
{
	if (pin >= C2_GPIOX_PIN_START && pin <= C2_GPIOX_PIN_END)
		return  C2_GPIOX_INP_REG_OFFSET;
	if (pin >= C2_GPIOY_PIN_START && pin <= C2_GPIOY_PIN_END)
		return  C2_GPIOY_INP_REG_OFFSET;
	if (pin >= C2_GPIODV_PIN_START && pin <= C2_GPIODV_PIN_END)
		return  C2_GPIODV_INP_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Pull up/down enable regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToPUENReg (int pin)
{
	if (pin >= C2_GPIOX_PIN_START && pin <= C2_GPIOX_PIN_END)
		return  C2_GPIOX_PUEN_REG_OFFSET;
	if (pin >= C2_GPIOY_PIN_START && pin <= C2_GPIOY_PIN_END)
		return  C2_GPIOY_PUEN_REG_OFFSET;
	if (pin >= C2_GPIODV_PIN_START && pin <= C2_GPIODV_PIN_END)
		return  C2_GPIODV_PUEN_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Pull up/down regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToPUPDReg (int pin)
{
	if (pin >= C2_GPIOX_PIN_START && pin <= C2_GPIOX_PIN_END)
		return	C2_GPIOX_PUPD_REG_OFFSET;
	if (pin >= C2_GPIOY_PIN_START && pin <= C2_GPIOY_PIN_END)
		return  C2_GPIOY_PUPD_REG_OFFSET;
	if (pin >= C2_GPIODV_PIN_START && pin <= C2_GPIODV_PIN_END)
		return  C2_GPIODV_PUPD_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO bit
//
/*----------------------------------------------------------------------------*/
static int gpioToShiftReg (int pin)
{
	if (pin >= C2_GPIOX_PIN_START && pin <= C2_GPIOX_PIN_END)
		return  pin - C2_GPIOX_PIN_START;
	if (pin >= C2_GPIOY_PIN_START && pin <= C2_GPIOY_PIN_END)
		return  pin - C2_GPIOY_PIN_START;
	if (pin >= C2_GPIODV_PIN_START && pin <= C2_GPIODV_PIN_END)
		return  pin - C2_GPIODV_PIN_START;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Function register
//
/*----------------------------------------------------------------------------*/
static int gpioToGPFSELReg (int pin)
{
	if(pin >= C2_GPIOX_PIN_START && pin <= C2_GPIOX_PIN_END)
		return  C2_GPIOX_FSEL_REG_OFFSET;
	if(pin >= C2_GPIOY_PIN_START && pin <= C2_GPIOY_PIN_END)
		return  C2_GPIOY_FSEL_REG_OFFSET;
	if(pin >= C2_GPIODV_PIN_START && pin <= C2_GPIODV_PIN_END)
		return  C2_GPIODV_FSEL_REG_OFFSET;
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
	if (retPin == 205 || retPin == 206) {
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
	case C2_GPIOX_PIN_START  ...C2_GPIOX_PIN_END:
		switch (shift) {
		case	0 ... 5:
			if (*(gpio + C2_MUX_REG_8_OFFSET) & (1 << (5 - shift)))		{ mode = 1; break; }
			break;
		case	6:
			if (*(gpio + C2_MUX_REG_3_OFFSET) & (1 << 9))			{ mode = 4; break; }
			if (*(gpio + C2_MUX_REG_3_OFFSET) & (1 << 17))			{ mode = 5; break; }
			break;
		case	7:
			if (*(gpio + C2_MUX_REG_8_OFFSET) & (1 << 11))			{ mode = 1; break; }
			if (*(gpio + C2_MUX_REG_3_OFFSET) & (1 << 8))			{ mode = 4; break; }
			if (*(gpio + C2_MUX_REG_3_OFFSET) & (1 << 18))			{ mode = 5; break; }
			break;
		case	8:
			if (*(gpio + C2_MUX_REG_4_OFFSET) & (1 << 7))			{ mode = 1; break; }
			if (*(gpio + C2_MUX_REG_3_OFFSET) & (1 << 30))			{ mode = 3; break; }
			if (*(gpio + C2_MUX_REG_3_OFFSET) & (1 << 10))			{ mode = 4; break; }
			break;
		case	9:
			if (*(gpio + C2_MUX_REG_4_OFFSET) & (1 << 6))			{ mode = 1; break; }
			if (*(gpio + C2_MUX_REG_3_OFFSET) & (1 << 29))			{ mode = 3; break; }
			if (*(gpio + C2_MUX_REG_3_OFFSET) & (1 << 7))			{ mode = 4; break; }
			break;
		case	10 ... 11:
			if (*(gpio + C2_MUX_REG_3_OFFSET) & (1 << (38 - shift)))	{ mode = 3; break; }
			break;
		case	12 ... 15:
			if (*(gpio + C2_MUX_REG_4_OFFSET) & (1 << (25 - shift)))	{ mode = 2; break; }
			if (*(gpio + C2_MUX_REG_4_OFFSET) & (1 << (29 - shift)))	{ mode = 3; break; }
			break;
		case	19:
			if (*(gpio + C2_MUX_REG_2_OFFSET) & (1 << 22))			{ mode = 2; break; }
			if (*(gpio + C2_MUX_REG_2_OFFSET) & (1 << 30))			{ mode = 5; break; }
			break;
		}
		break;
	case C2_GPIOY_PIN_START  ...C2_GPIOY_PIN_END:
		switch (shift) {
		case	0 ... 1:
			if (*(gpio + C2_MUX_REG_1_OFFSET) & (1 << shift))		{ mode = 5; break; }
			if (*(gpio + C2_MUX_REG_2_OFFSET) & (1 << (19 - shift)))	{ mode = 1; break; }
			if (*(gpio + C2_MUX_REG_3_OFFSET) & (1 << (2 - shift)))		{ mode = 2; break; }
			break;
		case	2:
			if (*(gpio + C2_MUX_REG_2_OFFSET) & (1 << 17))			{ mode = 1; break; }
			if (*(gpio + C2_MUX_REG_3_OFFSET) & (1 << 0))			{ mode = 2; break; }
			break;
		case	3:
			if (*(gpio + C2_MUX_REG_2_OFFSET) & (1 << 16))			{ mode = 1; break; }
			if (*(gpio + C2_MUX_REG_3_OFFSET) & (1 << 4))			{ mode = 2; break; }
			if (*(gpio + C2_MUX_REG_1_OFFSET) & (1 << 2))			{ mode = 5; break; }
			break;
		case	4 ... 5:
			if (*(gpio + C2_MUX_REG_2_OFFSET) & (1 << 16))			{ mode = 1; break; }
			if (*(gpio + C2_MUX_REG_3_OFFSET) & (1 << 5))			{ mode = 2; break; }
			if (*(gpio + C2_MUX_REG_1_OFFSET) & (1 << (shift + 8)))		{ mode = 4; break; }
			break;
		case	6 ... 10:
			if (*(gpio + C2_MUX_REG_2_OFFSET) & (1 << 16))			{ mode = 1; break; }
			if (*(gpio + C2_MUX_REG_3_OFFSET) & (1 << 5))			{ mode = 2; break; }
			if (*(gpio + C2_MUX_REG_1_OFFSET) & (1 << (shift - 3)))		{ mode = 5; break; }
			break;
		case	11:
			if (*(gpio + C2_MUX_REG_3_OFFSET) & (1 << 3))			{ mode = 2; break; }
			if (*(gpio + C2_MUX_REG_1_OFFSET) & (1 << 19))			{ mode = 3; break; }
			if (*(gpio + C2_MUX_REG_1_OFFSET) & (1 << 8))			{ mode = 5; break; }
			break;
		case    12 ... 14:
			if (*(gpio + C2_MUX_REG_1_OFFSET) & (1 << (30 - shift)))	{ mode = 3; break; }
			if (*(gpio + C2_MUX_REG_1_OFFSET) & (1 << (shift - 3)))		{ mode = 5; break; }
			break;
		case	15:
			if (*(gpio + C2_MUX_REG_2_OFFSET) & (1 << 20))			{ mode = 1; break; }
			if (*(gpio + C2_MUX_REG_1_OFFSET) & (1 << 20))			{ mode = 4; break; }
			if (*(gpio + C2_MUX_REG_1_OFFSET) & (1 << 22))			{ mode = 5; break; }
			break;
		case	16:
			if (*(gpio + C2_MUX_REG_2_OFFSET) & (1 << 21))			{ mode = 1; break; }
			if (*(gpio + C2_MUX_REG_1_OFFSET) & (1 << 21))			{ mode = 4; break; }
			break;
		}
		break;
	case C2_GPIODV_PIN_START...C2_GPIODV_PIN_END:
		switch (shift) {
		case	24 ... 25:
			if (*(gpio + C2_MUX_REG_0_OFFSET) & (1 << (31 - shift)))	{ mode = 1; break; }
			if (*(gpio + C2_MUX_REG_0_OFFSET) & (1 << (36 - shift)))	{ mode = 2; break; }
			if (*(gpio + C2_MUX_REG_5_OFFSET) & (1 << (36 - shift)))	{ mode = 3; break; }
			if (*(gpio + C2_MUX_REG_2_OFFSET) & (1 << (53 - shift)))	{ mode = 5; break; }
			if (*(gpio + C2_MUX_REG_7_OFFSET) & (1 << (shift + 2)))		{ mode = 6; break; }
			break;
		case	26:
			if (*(gpio + C2_MUX_REG_0_OFFSET) & (1 << 10))			{ mode = 2; break; }
			if (*(gpio + C2_MUX_REG_5_OFFSET) & (1 << 10))			{ mode = 3; break; }
			if (*(gpio + C2_MUX_REG_2_OFFSET) & (1 << 27))			{ mode = 5; break; }
			if (*(gpio + C2_MUX_REG_7_OFFSET) & (1 << 24))			{ mode = 6; break; }
			break;
		case	27:
			if (*(gpio + C2_MUX_REG_5_OFFSET) & (1 << 8))			{ mode = 4; break; }
			if (*(gpio + C2_MUX_REG_0_OFFSET) & (1 << 9))			{ mode = 2; break; }
			if (*(gpio + C2_MUX_REG_5_OFFSET) & (1 << 9))			{ mode = 3; break; }
			if (*(gpio + C2_MUX_REG_2_OFFSET) & (1 << 26))			{ mode = 5; break; }
			if (*(gpio + C2_MUX_REG_7_OFFSET) & (1 << 25))			{ mode = 6; break; }
			break;
		case	28:
			if (*(gpio + C2_MUX_REG_3_OFFSET) & (1 << 20))			{ mode = 5; break; }
			if (*(gpio + C2_MUX_REG_7_OFFSET) & (1 << 22))			{ mode = 6; break; }
			break;
		case	29:
			if (*(gpio + C2_MUX_REG_3_OFFSET) & (1 << 22))			{ mode = 4; break; }
			if (*(gpio + C2_MUX_REG_3_OFFSET) & (1 << 21))			{ mode = 5; break; }
			if (*(gpio + C2_MUX_REG_7_OFFSET) & (1 << 23))			{ mode = 6; break; }
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
	union	reg_bitfield	gpiox;

	gpiox.wvalue = *(gpio + C2_GPIOX_INP_REG_OFFSET);

	/* Wiring PI GPIO0 = C1 GPIOX.19 */
	gpiox.bits.bit19 = (value & 0x01);
	/* Wiring PI GPIO1 = C1 GPIOX.10 */
	gpiox.bits.bit10 = (value & 0x02);
	/* Wiring PI GPIO2 = C1 GPIOX.11 */
	gpiox.bits.bit11 = (value & 0x04);
	/* Wiring PI GPIO3 = C1 GPIOX.9 */
	gpiox.bits.bit9 = (value & 0x08);
	/* Wiring PI GPIO4 = C1 GPIOX.8 */
	gpiox.bits.bit8 = (value & 0x10);
	/* Wiring PI GPIO5 = C1 GPIOX.5 */
	gpiox.bits.bit5 = (value & 0x20);
	/* Wiring PI GPIO6 = C1 GPIOX.3 */
	gpiox.bits.bit3 = (value & 0x40);
	/* Wiring PI GPIO7 = C1 GPIOX.21 */
	gpiox.bits.bit21 = (value & 0x80);

	*(gpio + C2_GPIOX_OUTP_REG_OFFSET) = gpiox.wvalue;

	return 0;
}

/*----------------------------------------------------------------------------*/
static unsigned int _digitalReadByte (void)
{
	return	-1;
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
		//#define C2_GPIO_BASE	0xC8834000
#ifdef ANDROID
#if defined(__aarch64__)
		mapped = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, C2_GPIO_BASE);
#else
		mapped = mmap64(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, (off64_t)C2_GPIO_BASE);
#endif
#else
		mapped = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, C2_GPIO_BASE);
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
	AIN0_NODE = "/sys/class/saradc/ch0";
	AIN1_NODE = "/sys/class/saradc/ch1";

	adcFds[0] = open(AIN0_NODE, O_RDONLY);
	adcFds[1] = open(AIN1_NODE, O_RDONLY);
}

/*----------------------------------------------------------------------------*/
void init_odroidc2 (struct libodroid *libwiring)
{
	init_gpio_mmap();

	init_adc_fds();

	if (libwiring->rev == 1) {
		pinToGpio = pinToGpio_rev1;
		phyToGpio = phyToGpio_rev1;
	} else {
		pinToGpio = pinToGpio_rev2;
		phyToGpio = phyToGpio_rev2;
	}

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
	libwiring->pinBase		= C2_GPIO_PIN_BASE;

	/* global variable setup */
	lib = libwiring;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
