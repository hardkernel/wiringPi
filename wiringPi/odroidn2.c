/*----------------------------------------------------------------------------*/
//
//
//	WiringPi ODROID-N2 Board Control file (AMLogic 64Bits Platform)
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
#include "odroidn2.h"

/*----------------------------------------------------------------------------*/
// wiringPi gpio map define
/*----------------------------------------------------------------------------*/
static const int pinToGpio_rev1[64] = {
			// wiringPi number to native gpio number
	479, 492,	// GPIOX.3			0	| 1	GPIOX.16(PWM_E)
	480, 483,	// GPIOX.4			2	| 3	GPIOX.7(PWM_F)
	476, 477,	// GPIOX.0			4	| 5	GPIOX.1
	478, 473,	// GPIOX.2			6	| 7	GPIOA.13
	493, 494,	// GPIOX.17(I2C-2_SDA)		8	| 9	GPIOX.18(I2C-2_SCL)
	486, 464,	// GPIOX.10			10	| 11	GPIOA.4
	484, 485,	// GPIOX.8			12	| 13	GPIOX.9
	487, 488,	// GPIOX.11			14	| 15	GPIOX.12
	489,  -1,	// GPIOX.13			16	| 17
	-1,  -1,	// 				18	| 19
	-1,  490,	// 				20	| 21	GPIOX.14
	491, 481,	// GPIOX.15			22	| 23	GPIOX.5(PWM_C)
	482, -1,	// GPIOX.6(PWM_D)		24	| 25	ADC.AIN3
	472, 495,	// GPIOA.12			26	| 27	GPIOX.19
	-1,  -1,	// REF1.8V OUT			28	| 29	ADC.AIN2
	474, 475,	// GPIOA.14(I2C-3_SDA)		30	| 31	GPIOA.15(I2C-3_SCL)
	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 32...47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 48...63
};

static const int phyToGpio_rev1[64] = {
			// physical header pin number to native gpio number
	 -1,		// 			0
	 -1,  -1,	// 3.3V			1	| 2	5.0V
	493,  -1,	// GPIOX.17(I2C-2_SDA)	3	| 4	5.0V
	494,  -1,	// GPIOX.18(I2C-2_SCL)	5	| 6	GND
	473, 488,	// GPIOA.13		7	| 8	GPIOX.12(UART_TX_B)
	 -1, 489,	// GND			9	| 10	GPIOX.13(UART_RX_B)
	479, 492,	// GPIOX.3		11	| 12	GPIOX.16(PWM_E)
	480,  -1,	// GPIOX.4		13	| 14	GND
	483, 476,	// GPIOX.7(PWM_F)	15	| 16	GPIOX.0
	 -1, 477,	// 3.3V			17	| 18	GPIOX.1
	484,  -1,	// GPIOX.8(SPI_MOSI)	19	| 20	GND
	485, 478,	// GPIOX.9(SPI_MISO)	21	| 22	GPIOX.2
	487, 486,	// GPIOX.11(SPI_SCLK)	23	| 24	GPIOX.10(SPI_CE0)
	 -1, 464,	// GND			25	| 26	GPIOA.4(SPI_CE1)
	474, 475,	// GPIOA.14(I2C-3_SDA)	27	| 28	GPIOA.15(I2C-3_SCL)
	490,  -1,	// GPIOX.14		29	| 30	GND
	491, 472,	// GPIOX.15		31	| 32	GPIOA.12
	481,  -1,	// GPIOX.5(PWM_C)	33	| 34	GND
	482, 495,	// GPIOX.6(PWM_D)	35	| 36	GPIOX.19
	 -1,  -1,	// ADC.AIN3		37	| 38	1.8V REF OUT
	 -1,  -1,	// GND			39	| 40	ADC.AIN2
	// Not used
	-1, -1, -1, -1, -1, -1, -1, -1,	// 41...48
	-1, -1, -1, -1, -1, -1, -1, -1,	// 49...56
	-1, -1, -1, -1, -1, -1, -1	// 57...63
};

static int16_t _gpioTophysPin [] = {
			// (native gpio number - N2_GPIOA_PIN_START) to physical header pin number
	 -1,		// 			0 + N2_GPIOA_PIN_START(460)
	 -1,  -1,	// 			1	| 2
	 -1,  26,	// 			3	| 4	GPIOA.4(SPI_CE1)
	 -1,  -1,	// 			5	| 6
	 -1,  -1,	// 			7	| 8
	 -1,  -1,	// 			9	| 10
	 -1,  32,	// 			11	| 12	GPIOA.12
	  7,  27,	// GPIOA.13		13	| 14	GPIOA.14(I2C-3_SDA)
	 28,  16,	// GPIOA.15(I2C-3_SCL)	15	| 16	GPIOX.0
	 18,  22,	// GPIOX.1		17	| 18	GPIOX.2
	 11,  13,	// GPIOX.3		19	| 20	GPIOX.4
	 33,  35,	// GPIOX.5(PWM_C)	21	| 22	GPIOX.6(PWM_D)
	 15,  19,	// GPIOX.7(PWM_F)	23	| 24	GPIOX.8(SPI_MOSI)
	 21,  24,	// GPIOX.9(SPI_MISO)	25	| 26	GPIOX.10(SPI_CE0)
	 23,   8,	// GPIOX.11(SPI_SCLK)	27	| 28	GPIOX.12(UART_TX_B)
	 10,  29,	// GPIOX.13(UART_RX_B)	29	| 30	GPIOX.14
	 31,  12,	// GPIOX.15		31	| 32	GPIOX.16(PWM_E)
	  3,   5,	// GPIOX.17(I2C-2_SDA)	33	| 34	GPIOX.18(I2C-2_SCL)
	 36,  -1,	// GPIOX.19		35	| 36
	 -1,  -1,	// 			37	| 38
	 -1,  -1,	// 			39	| 40
	// Not used
	-1, -1, -1, -1, -1, -1, -1, -1,	// 41...48
	-1, -1, -1, -1, -1, -1, -1, -1,	// 49...56
	-1, -1, -1, -1, -1, -1, -1	// 57...63
};

static int8_t _gpioToPwmPin [] = {
			// (native gpio number - N2_GPIOA_PIN_START) to PWM pin number
	 -1,		// 			0 + N2_GPIOA_PIN_START(460)
	 -1,  -1,	// 			1	| 2
	 -1,  -1,	// 			3	| 4	GPIOA.4(SPI_CE1)
	 -1,  -1,	// 			5	| 6
	 -1,  -1,	// 			7	| 8
	 -1,  -1,	// 			9	| 10
	 -1,  -1,	// 			11	| 12	GPIOA.12
	 -1,  -1,	// GPIOA.13		13	| 14	GPIOA.14(I2C-3_SDA)
	 -1,  -1,	// GPIOA.15(I2C-3_SCL)	15	| 16	GPIOX.0
	 -1,  -1,	// GPIOX.1		17	| 18	GPIOX.2
	 -1,  -1,	// GPIOX.3		19	| 20	GPIOX.4
	  2,   3,	// GPIOX.5(PWM_C)	21	| 22	GPIOX.6(PWM_D)
	  5,  -1,	// GPIOX.7(PWM_F)	23	| 24	GPIOX.8(SPI_MOSI)
	 -1,  -1,	// GPIOX.9(SPI_MISO)	25	| 26	GPIOX.10(SPI_CE0)
	 -1,  -1,	// GPIOX.11(SPI_SCLK)	27	| 28	GPIOX.12(UART_TX_B)
	 -1,  -1,	// GPIOX.13(UART_RX_B)	29	| 30	GPIOX.14
	 -1,   4,	// GPIOX.15		31	| 32	GPIOX.16(PWM_E)
	 -1,  -1,	// GPIOX.17(I2C-2_SDA)	33	| 34	GPIOX.18(I2C-2_SCL)
	 -1,  -1,	// GPIOX.19		35	| 36
	 -1,  -1,	// 			37	| 38
	 -1,  -1,	// 			39	| 40
	// Not used
	-1, -1, -1, -1, -1, -1, -1, -1,	// 41...48
	-1, -1, -1, -1, -1, -1, -1, -1,	// 49...56
	-1, -1, -1, -1, -1, -1, -1	// 57...63
};

static uint16_t pwmPinToALT [] = {
	0, 0,	// A, B
	4, 4,	// C 481 GPIOX.5 , D 482 GPIOX.6
	1, 1	// E 492 GPIOX.16, F 483 GPIOX.7
};

static uint16_t pwmPinToRange [] = {
	0, 0,	// A, B
	0, 0,	// C 481 GPIOX.5 , D 482 GPIOX.6
	0, 0	// E 492 GPIOX.16, F 483 GPIOX.7
};

static uint16_t pwmPinToDutyOffset [] = {
	N2_PWM_0_DUTY_CYCLE_OFFSET, N2_PWM_1_DUTY_CYCLE_OFFSET,	// A, B
	N2_PWM_0_DUTY_CYCLE_OFFSET, N2_PWM_1_DUTY_CYCLE_OFFSET,	// C 481 GPIOX.5 , D 482 GPIOX.6
	N2_PWM_0_DUTY_CYCLE_OFFSET, N2_PWM_1_DUTY_CYCLE_OFFSET	// E 492 GPIOX.16, F 483 GPIOX.7
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
static volatile uint32_t *pwm[3];

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
static int	gpioToDSReg	(int pin);
static int	gpioToMuxReg	(int pin);
static int	gpioToPwmPin	(int pin);
static int	gpioTophysPin	(int pin) UNU;

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
static int		_pwmWrite		(int pin, int value);
static int		_analogRead		(int pin);
static int		_digitalWriteByte	(const unsigned int value);
static unsigned int	_digitalReadByte	(void);
static void		_pwmSetRange		(unsigned int range);
static void		_pwmSetClock		(int divisor);

/*----------------------------------------------------------------------------*/
// board init function
/*----------------------------------------------------------------------------*/
static 	void init_gpio_mmap	(void);
static 	void init_adc_fds	(void);

	void init_odroidn2 	(struct libodroid *libwiring);

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Set regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToGPSETReg (int pin)
{
	if (pin >= N2_GPIOX_PIN_START && pin <= N2_GPIOX_PIN_END)
		return  N2_GPIOX_OUTP_REG_OFFSET;
	if (pin >= N2_GPIOA_PIN_START && pin <= N2_GPIOA_PIN_END)
		return  N2_GPIOA_OUTP_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Input regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToGPLEVReg (int pin)
{
	if (pin >= N2_GPIOX_PIN_START && pin <= N2_GPIOX_PIN_END)
		return  N2_GPIOX_INP_REG_OFFSET;
	if (pin >= N2_GPIOA_PIN_START && pin <= N2_GPIOA_PIN_END)
		return  N2_GPIOA_INP_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Pull up/down enable regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToPUENReg (int pin)
{
	if (pin >= N2_GPIOX_PIN_START && pin <= N2_GPIOX_PIN_END)
		return  N2_GPIOX_PUEN_REG_OFFSET;
	if (pin >= N2_GPIOA_PIN_START && pin <= N2_GPIOA_PIN_END)
		return  N2_GPIOA_PUEN_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Pull up/down regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToPUPDReg (int pin)
{
	if (pin >= N2_GPIOX_PIN_START && pin <= N2_GPIOX_PIN_END)
		return	N2_GPIOX_PUPD_REG_OFFSET;
	if (pin >= N2_GPIOA_PIN_START && pin <= N2_GPIOA_PIN_END)
		return  N2_GPIOA_PUPD_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO bit
//
/*----------------------------------------------------------------------------*/
static int gpioToShiftReg (int pin)
{
	if (pin >= N2_GPIOX_PIN_START && pin <= N2_GPIOX_PIN_END)
		return  pin - N2_GPIOX_PIN_START;
	if (pin >= N2_GPIOA_PIN_START && pin <= N2_GPIOA_PIN_END)
		return  pin - N2_GPIOA_PIN_START;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Function register
//
/*----------------------------------------------------------------------------*/
static int gpioToGPFSELReg (int pin)
{
	if(pin >= N2_GPIOX_PIN_START && pin <= N2_GPIOX_PIN_END)
		return  N2_GPIOX_FSEL_REG_OFFSET;
	if(pin >= N2_GPIOA_PIN_START && pin <= N2_GPIOA_PIN_END)
		return  N2_GPIOA_FSEL_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Drive Strength register
//
/*----------------------------------------------------------------------------*/
static int gpioToDSReg (int pin)
{
	if (pin >= N2_GPIOX_PIN_START && pin <= N2_GPIOX_PIN_MID)
		return  N2_GPIOX_DS_REG_2A_OFFSET;
	if (pin > N2_GPIOX_PIN_MID && pin <= N2_GPIOX_PIN_END)
		return  N2_GPIOX_DS_REG_2B_OFFSET;
	if (pin >= N2_GPIOA_PIN_START && pin <= N2_GPIOA_PIN_END)
		return  N2_GPIOA_DS_REG_5A_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Pin Mux register
//
/*----------------------------------------------------------------------------*/
static int gpioToMuxReg (int pin)
{
	switch (pin) {
	case	N2_GPIOA_PIN_START	...N2_GPIOA_PIN_START + 7:
		return  N2_GPIOA_MUX_D_REG_OFFSET;
	case	N2_GPIOA_PIN_START + 8	...N2_GPIOA_PIN_END:
		return  N2_GPIOA_MUX_E_REG_OFFSET;
	case	N2_GPIOX_PIN_START	...N2_GPIOX_PIN_START + 7:
		return  N2_GPIOX_MUX_3_REG_OFFSET;
	case	N2_GPIOX_PIN_START + 8	...N2_GPIOX_PIN_START + 15:
		return  N2_GPIOX_MUX_4_REG_OFFSET;
	case	N2_GPIOX_PIN_START + 16	...N2_GPIOX_PIN_END:
		return  N2_GPIOX_MUX_5_REG_OFFSET;
	default:
		return -1;
	}
}

/*----------------------------------------------------------------------------*/
static int gpioToPwmPin (int pin)
{
	return _gpioToPwmPin[pin - N2_GPIOA_PIN_START];
}

/*----------------------------------------------------------------------------*/
static int gpioTophysPin (int pin)
{
	return _gpioTophysPin[pin - N2_GPIOA_PIN_START];
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

	return retPin;
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
	shift = gpioToShiftReg(pin);
	shift = pin > N2_GPIOX_PIN_MID ? (shift - 16) * 2 : shift * 2 ;

	*(gpio + ds) &= ~(0b11 << shift);
	*(gpio + ds) |= (value << shift);

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
	shift = gpioToShiftReg(pin);
	shift = pin > N2_GPIOX_PIN_MID ? (shift - 16) * 2 : shift * 2 ;

	return (*(gpio + ds) >> shift) & 0b11;
}

/*----------------------------------------------------------------------------*/
static int _pinMode (int pin, int mode)
{
	int fsel, mux, target, shift, origPin = pin;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	softPwmStop  (origPin);
	softToneStop (origPin);

	fsel	= gpioToGPFSELReg(pin);
	mux	= gpioToMuxReg(pin);
	shift	= gpioToShiftReg (pin);
	target	= shift * 4;

	switch (mode) {
	case	INPUT:
		*(gpio + mux) = (*(gpio + mux) & ~(0xF << target));
		*(gpio + fsel) = (*(gpio + fsel) |  (1 << shift));
		_pullUpDnControl(origPin, PUD_OFF);
		break;
	case	OUTPUT:
		*(gpio + mux) = (*(gpio + mux) & ~(0xF << target));
		*(gpio + fsel) = (*(gpio + fsel) & ~(1 << shift));
		break;
	case 	INPUT_PULLUP:
		*(gpio + mux) = (*(gpio + mux) & ~(0xF << target));
		*(gpio + fsel) = (*(gpio + fsel) |  (1 << shift));
		_pullUpDnControl(origPin, PUD_UP);
		break;
	case 	INPUT_PULLDOWN:
		*(gpio + mux) = (*(gpio + mux) & ~(0xF << target));
		*(gpio + fsel) = (*(gpio + fsel) |  (1 << shift));
		_pullUpDnControl(origPin, PUD_DOWN);
		break;
	case	SOFT_PWM_OUTPUT:
		softPwmCreate (origPin, 0, 100);
		break;
	case	SOFT_TONE_OUTPUT:
		softToneCreate (origPin);
		break;
	case	PWM_OUTPUT:
		usingGpioMemCheck ("pinMode PWM");

		int pwm_pin, alt;
		pwm_pin = gpioToPwmPin(pin);
		if( pwm_pin == -1 )
		{
			msg(MSG_WARN, "%s : This pin does not support hardware PWM mode.\n", __func__);
			return -1;
		}

		alt		= pwmPinToALT[pwm_pin];
		*(gpio + mux)	= (*(gpio + mux) & ~(0xF << target)) | (alt << target);

#ifndef ANDROID
		/**
		 * 24 MHz / 120
		 * 200 kHz / 500
		 * frequency of PWM: 400 Hz
		 * period of PWM: 2500 us
		 */
		_pwmSetClock(120);
		_pwmSetRange(500);
#endif

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
	int fsel, mux, shift, target, mode;

	if (lib->mode == MODE_GPIO_SYS)
		return	-1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return	-1;

	fsel   = gpioToGPFSELReg(pin);
	mux    = gpioToMuxReg(pin);
	target = shift = gpioToShiftReg(pin);

	while (target >= 8) {
		target -= 8;
	}

	mode = (*(gpio + mux) >> (target * 4)) & 0xF;
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
// PWM signal ___-----------___________---------------_______-----_
//               <--value-->           <----value---->
//               <-------range--------><-------range-------->
/*----------------------------------------------------------------------------*/
static int _pwmWrite (int pin, int value)
{
	/**
	 * @todo Add node
	 * struct wiringPiNodeStruct *node = wiringPiNodes;
	 */

	setupCheck ("pwmWrite") ;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	int pwm_pin	= gpioToPwmPin(pin);
	uint16_t range	= pwmPinToRange[pwm_pin];

	if( value > range ) {
		value = range;
	}

	*(pwm[pwm_pin/2] + pwmPinToDutyOffset[pwm_pin]) = (value << 16) | (range - value);

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
	case	3:	case	25:
		pin = 0;
	break;
	case	2:	case	29:
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
	union	reg_bitfield	gpioa;

	gpiox.wvalue = *(gpio + N2_GPIOX_INP_REG_OFFSET);
	gpioa.wvalue = *(gpio + N2_GPIOA_INP_REG_OFFSET);

	/* Wiring PI GPIO0 = N2 GPIOX.3 */
	gpiox.bits.bit3 = (value & 0x01);
	/* Wiring PI GPIO1 = N2 GPIOX.16 */
	gpiox.bits.bit16 = (value & 0x02);
	/* Wiring PI GPIO2 = N2 GPIOX.4 */
	gpiox.bits.bit4 = (value & 0x04);
	/* Wiring PI GPIO3 = N2 GPIOX.7 */
	gpiox.bits.bit7 = (value & 0x08);
	/* Wiring PI GPIO4 = N2 GPIOX.0 */
	gpiox.bits.bit0 = (value & 0x10);
	/* Wiring PI GPIO5 = N2 GPIOX.1 */
	gpiox.bits.bit1 = (value & 0x20);
	/* Wiring PI GPIO6 = N2 GPIOX.2 */
	gpiox.bits.bit2 = (value & 0x40);
	/* Wiring PI GPIO7 = N2 GPIOA.13 */
	gpioa.bits.bit13 = (value & 0x80);

	*(gpio + N2_GPIOX_OUTP_REG_OFFSET) = gpiox.wvalue;
	*(gpio + N2_GPIOA_OUTP_REG_OFFSET) = gpioa.wvalue;

	return 0;
}

/*----------------------------------------------------------------------------*/
// PWM signal ___-----------___________---------------_______-----_
//               <--value-->           <----value---->
//               <-------range--------><-------range-------->
// PWM frequency == (PWM clock) / range
/*----------------------------------------------------------------------------*/
static void _pwmSetRange (unsigned int range)
{
	range = range & 0xFFFF;
	for( int i = 0; i < 6; ++i )
	{
		pwmPinToRange[i] = range;
	}
}

/*----------------------------------------------------------------------------*/
// Internal clock == 24MHz
// PWM clock == (Internal clock) / divisor
// PWM frequency == (PWM clock) / range
/*----------------------------------------------------------------------------*/
static void _pwmSetClock (int divisor)
{
	if((divisor < 1) || (divisor > 128))
	{
		msg(MSG_ERR,
			"Set the clock prescaler (divisor) to 1 or more and 128 or less.: %s\n",
			strerror (errno));
	}
	divisor = (divisor - 1);

	for(uint16_t i = 1; i < 3; ++i) {
		*( pwm[i] + N2_PWM_MISC_REG_01_OFFSET ) = \
			(1 << N2_PWM_1_CLK_EN) \
			| ( divisor << N2_PWM_1_CLK_DIV0) \
			| (1 << N2_PWM_0_CLK_EN) \
			| ( divisor << N2_PWM_0_CLK_DIV0) \
			| (0 << N2_PWM_1_CLK_SEL0) \
			| (0 << N2_PWM_0_CLK_SEL0) \
			| (1 << N2_PWM_1_EN) \
			| (1 << N2_PWM_0_EN);
	}
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
			msg(MSG_ERR,
				"wiringPiSetup: Unable to open /dev/mem: %s\n",
				strerror (errno));
	} else {
		if (access("/dev/gpiomem",0) == 0) {
			if ((fd = open ("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
				msg(MSG_ERR,
					"wiringPiSetup: Unable to open /dev/gpiomem: %s\n",
					strerror (errno));

			setUsingGpioMem( TRUE );
		} else
			msg(MSG_ERR,
				"wiringPiSetup: /dev/gpiomem doesn't exist. Please try again with sudo.\n");
	}

	if (fd < 0) {
		msg(MSG_ERR, "wiringPiSetup: Cannot open memory area for GPIO use. \n");
	} else {
		//#define N2_GPIO_BASE	0xff634000
#ifdef ANDROID
#if defined(__aarch64__)
		mapped = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, N2_GPIO_BASE);
#else
		mapped = mmap64(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, (off64_t)N2_GPIO_BASE);
#endif
#else
		mapped = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, N2_GPIO_BASE);
#endif

		if (mapped == MAP_FAILED)
			msg(MSG_ERR, "wiringPiSetup: mmap (GPIO) failed: %s \n", strerror (errno));
		else
			gpio = (uint32_t *) mapped;

		for(uint16_t i = 1; i < 3; ++i) {
			pwm[i] = ( uint32_t * )mmap( 0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, N2_GPIO_PWM_BASE + (0x1000 * (2 - i)) );
			if( ( void * )pwm == MAP_FAILED )
				msg(MSG_ERR, "wiringPiSetup: mmap (PWM) failed: %s \n", strerror (errno));
		}
	}
}

/*----------------------------------------------------------------------------*/
static void init_adc_fds (void)
{
	const char *AIN0_NODE, *AIN1_NODE;

	/* ADC node setup */
	AIN0_NODE = "/sys/devices/platform/ff809000.saradc/iio:device0/in_voltage2_raw";
	AIN1_NODE = "/sys/devices/platform/ff809000.saradc/iio:device0/in_voltage3_raw";

	adcFds[0] = open(AIN0_NODE, O_RDONLY);
	adcFds[1] = open(AIN1_NODE, O_RDONLY);
}

/*----------------------------------------------------------------------------*/
void init_odroidn2 (struct libodroid *libwiring)
{
	init_gpio_mmap();

	init_adc_fds();

	pinToGpio = pinToGpio_rev1;
	phyToGpio = phyToGpio_rev1;

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
	libwiring->pwmWrite		= _pwmWrite;
	libwiring->analogRead		= _analogRead;
	libwiring->digitalWriteByte	= _digitalWriteByte;
	libwiring->digitalReadByte	= _digitalReadByte;
	libwiring->pwmSetRange		= _pwmSetRange;
	libwiring->pwmSetClock		= _pwmSetClock;

	/* specify pin base number */
	libwiring->pinBase		= N2_GPIO_PIN_BASE;

	/* global variable setup */
	lib = libwiring;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
