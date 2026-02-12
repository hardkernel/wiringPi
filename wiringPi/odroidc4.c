/*----------------------------------------------------------------------------*/
//
//
//	WiringPi ODROID-C4 Board Control file (AMLogic 64Bits Platform)
//
//
/*----------------------------------------------------------------------------*/
#include <dirent.h>
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
#include <sys/stat.h>

/*----------------------------------------------------------------------------*/
#include "softPwm.h"
#include "softTone.h"

/*----------------------------------------------------------------------------*/
#include "wiringPi.h"
#include "odroidc4.h"

/*----------------------------------------------------------------------------*/
// wiringPi gpio map define
/*----------------------------------------------------------------------------*/
static const int pinToGpio[64] = {
	// wiringPi number to native gpio number
	479, 492,	//  0 |  1 : GPIOX.3, GPIOX.16
	480, 483,	//  2 |  3 : GPIOX.4, GPIOX.7
	476, 477,	//  4 |  5 : GPIOX.0, GPIOX.1
	478, 481,	//  6 |  7 : GPIOX.2, GPIOX.5
	493, 494,	//  8 |  9 : GPIOX.17(I2C-2_SDA), GPIOX.18(I2C-2_SCL)
	486, 433,	// 10 | 11 : GPIOX.10(SPI_SS), GPIOH.6
	484, 485,	// 12 | 13 : GPIOX.8(SPI_MOSI), GPIOX.9(SPI_MISO)
	487, 488,	// 14 | 15 : GPIOX.11(SPI_CLK), GPIOX.12(UART_TX_B)
	489,  -1,	// 16 | 17 : GPIOX.13(UART_RX_B),
	 -1, - 1,	// 18 | 19 :
	 -1, 490,	// 20 | 21 : , GPIOX.14
	491, 482,	// 22 | 23 : GPIOX.15, GPIOX.6
	495,  -1,	// 24 | 25 : GPIOX.19, ADC.AIN3
	434, 432,	// 26 | 27 : GPIOH.7, GPIOH.5
	 -1, - 1,	// 28 | 29 : REF1.8V OUT, ADC.AIC4
	474, 475,	// 30 | 31 : GPIOA.14(I2C-3_SDA), GPIOA.15(I2C-3_SCL)
	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 32...47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1	// 48...63
};

static const int phyToGpio[64] = {
	// physical header pin number to native gpio number
	 -1,		//  0
	 -1,  -1,	//  1 |  2 : 3.3V, 5.0V
	493,  -1,	//  3 |  4 : GPIOX.17(I2C-2_SDA), 5.0V
	494,  -1,	//  5 |  6 : GPIOX.18(I2C-2_SCL), GND
	481, 488,	//  7 |  8 : GPIOX.5, GPIOX.12(UART_TX_B)
	 -1, 489,	//  9 | 10 : GND, GPIOX.13(UART_RX_B)
	479, 492,	// 11 | 12 : GPIOX.3, GPIOX.16
	480,  -1,	// 13 | 14 : GPIOX.4, GND
	483, 476,	// 15 | 16 : GPIOX.7, GPIOX.0
	 -1, 477,	// 17 | 18 : 3.3V, GPIOX.1
	484,  -1,	// 19 | 20 : GPIOX.8(SPI_MOSI), GND
	485, 478,	// 21 | 22 : GPIOX.9(SPI_MISO), GPIOX.2
	487, 486,	// 23 | 24 : GPIOX.11(SPI_CLK), GPIOX.10(SPI_SS)
	 -1, 433,	// 25 | 26 : GND, GPIOH.6
	474, 475,	// 27 | 28 : GPIOA.14(I2C-3_SDA), GPIOA.15(I2C-3_SCL)
	490,  -1,	// 29 | 30 : GPIOX.14, GND
	491, 434,	// 31 | 32 : GPIOX.15, GPIOH.7
	482,  -1,	// 33 | 34 : GPIOX.6, GND
	495, 432,	// 35 | 36 : GPIOX.19, GPIOH.5
	 -1,  -1,	// 37 | 38 : ADC.AIN3, 1.8V REF OUT
	 -1,  -1,	// 39 | 40 : GND, ADC.AIC4
	// Not used
	-1, -1, -1, -1, -1, -1, -1, -1,	// 41...48
	-1, -1, -1, -1, -1, -1, -1, -1,	// 49...56
	-1, -1, -1, -1, -1, -1, -1	// 57...63
};

static const char *pinToPwm[64] = {
	// wiringPi number to pwm group number
		"ffd1a000", "ffd19000",		//  0 |  1 : GPIOX.3(PWM_CD), GPIOX.16(PWM_EF)
		"None", "ffd19000",		//  2 |  3 : GPIOX.4, GPIOX.7(PWM_EF)
		"None", "None",				//  4 |  5 : GPIOX.0, GPIOX.1
		"None", "ffd1a000",		//  6 |  7 : GPIOX.2, GPIOX.5(PWM_CD)
		"None", "None",			//  8 |  9 : GPIOX.17(I2C-2_SDA), GPIOX.18(I2C-2_SCL)
		"None", "None",			// 10 | 11 : GPIOX.10(SPI_SS), GPIOH.6
		"None", "None",			// 12 | 13 : GPIOX.8(SPI_MOSI), GPIOX.9(SPI_MISO)
		"None", "None",			// 14 | 15 : GPIOX.11(SPI_CLK), GPIOX.12(UART_TX_B)
		"None", "None",			// 16 | 17 : GPIOX.13(UART_RX_B),
		"None", "None",			// 18 | 19 :
		"None", "None",			// 20 | 21 : , GPIOX.14
		"None", "ffd1b000",	// 22 | 23 : GPIOX.15, GPIOX.6(PWM_AB)
		"ffd1b000", "None",	// 24 | 25 : GPIOX.19(PWM_AB), ADC.AIN3
		"None", "None",	// 26 | 27 : GPIOH.7, GPIOH.5
		"None", "None",	// 28 | 29 : REF1.8V OUT, ADC.AIC4
		"None", "None",	// 30 | 31 : GPIOA.14(I2C-3_SDA), GPIOA.15(I2C-3_SCL)
	// Padding:
	"None","None","None","None","None","None","None","None","None","None","None","None","None","None","None","None", // 32...47
	"None","None","None","None","None","None","None","None","None","None","None","None","None","None","None","None"  // 48...63
};

static const int pinToPwmNum[64] = {
	// wiringPi number to pwm pin number
	  3,  4,	//  0 |  1 : GPIOX.3(PWM_D), GPIOX.16(PWM_E)
	 -1,  5,	//  2 |  3 : GPIOX.4, GPIOX.7(PWM_F)
	 -1, -1,	//  4 |  5 : GPIOX.0, GPIOX.1
	 -1,  2,	//  6 |  7 : GPIOX.2, GPIOX.5(PWM_C)
	 -1, -1,	//  8 |  9 : GPIOX.17(I2C-2_SDA), GPIOX.18(I2C-2_SCL)
	 -1, -1,	// 10 | 11 : GPIOX.10(SPI_SS), GPIOH.6
	 -1, -1,	// 12 | 13 : GPIOX.8(SPI_MOSI), GPIOX.9(SPI_MISO)
	 -1, -1,	// 14 | 15 : GPIOX.11(SPI_CLK), GPIOX.12(UART_TX_B)
	 -1, -1,	// 16 | 17 : GPIOX.13(UART_RX_B),
	 -1, -1,	// 18 | 19 :
	 -1, -1,	// 20 | 21 : , GPIOX.14
	 -1,  0,	// 22 | 23 : GPIOX.15, GPIOX.6(PWM_A)
	  1, -1,	// 24 | 25 : GPIOX.19(PWM_B), ADC.AIN3
	 -1, -1,	// 26 | 27 : GPIOH.7, GPIOH.5
	 -1, -1,	// 28 | 29 : REF1.8V OUT, ADC.AIC4
	 -1, -1,	// 30 | 31 : GPIOA.14(I2C-3_SDA), GPIOA.15(I2C-3_SCL)
	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 32...47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1	// 48...63
};

static char pwmPinPath[10][(BLOCK_SIZE)] = {
	"","",
	"","",
	"","",
	// Padding:
	"None","None","None","None"
};

static char setupedPwmPinPath[10][BLOCK_SIZE] = {
	"None","None",
	"None","None",
	"None","None",
	"None","None",
	"None","None"
};

/*----------------------------------------------------------------------------*/
//
// Global variable define
//
/*----------------------------------------------------------------------------*/
// wiringPi Pinmap control arrary
/*----------------------------------------------------------------------------*/
/* ADC file descriptor */
static int adcFds[2];

/* GPIO mmap control */
static volatile uint32_t *gpio;

/* wiringPi Global library */
static struct libodroid	*lib = NULL;

/* pwm sysnode */
static DIR *pwm;
static struct dirent *pwmchip;
/* pwm params */
static char sysPwmPath[(BLOCK_SIZE / 4)];
static char pwmExport[(BLOCK_SIZE / 16)];
static char pwmUnexport[(BLOCK_SIZE / 16)];
static char pwmPeriod[(BLOCK_SIZE / 16)];
static char pwmDuty[(BLOCK_SIZE / 16)];
static unsigned int pwmClock;
static unsigned int pwmRange;

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
/*----------------------------------------------------------------------------*/
// Function of pwm define
/*----------------------------------------------------------------------------*/
static int	pinToSysPwmPath	(int pin);
static int	pwmSetup (int pin);
static int	pwmRelease (int pin);
/*----------------------------------------------------------------------------*/
// wiringPi core function
/*----------------------------------------------------------------------------*/
static int		_getModeToGpio		(int mode, int pin);
static int		_setDrive		(int pin, int value);
static int		_getDrive		(int pin);
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

	void init_odroidc4 	(struct libodroid *libwiring);

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Set regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToGPSETReg (int pin)
{
	if (pin >= C4_GPIOH_PIN_START && pin <= C4_GPIOH_PIN_END)
		return  C4_GPIOH_OUTP_REG_OFFSET;
	if (pin >= C4_GPIOA_PIN_START && pin <= C4_GPIOA_PIN_END)
		return  C4_GPIOA_OUTP_REG_OFFSET;
	if (pin >= C4_GPIOX_PIN_START && pin <= C4_GPIOX_PIN_END)
		return  C4_GPIOX_OUTP_REG_OFFSET;
	return	-1;
}

/*---------------------------------------------------------------------------r-*/
//
// offset to the GPIO Input regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToGPLEVReg (int pin)
{
	if (pin >= C4_GPIOH_PIN_START && pin <= C4_GPIOH_PIN_END)
		return  C4_GPIOH_INP_REG_OFFSET;
	if (pin >= C4_GPIOA_PIN_START && pin <= C4_GPIOA_PIN_END)
		return  C4_GPIOA_INP_REG_OFFSET;
	if (pin >= C4_GPIOX_PIN_START && pin <= C4_GPIOX_PIN_END)
		return  C4_GPIOX_INP_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Pull up/down enable regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToPUENReg (int pin)
{
	if (pin >= C4_GPIOH_PIN_START && pin <= C4_GPIOH_PIN_END)
		return  C4_GPIOH_PUEN_REG_OFFSET;
	if (pin >= C4_GPIOA_PIN_START && pin <= C4_GPIOA_PIN_END)
		return  C4_GPIOA_PUEN_REG_OFFSET;
	if (pin >= C4_GPIOX_PIN_START && pin <= C4_GPIOX_PIN_END)
		return  C4_GPIOX_PUEN_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Pull up/down regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToPUPDReg (int pin)
{
	if (pin >= C4_GPIOH_PIN_START && pin <= C4_GPIOH_PIN_END)
		return  C4_GPIOH_PUPD_REG_OFFSET;
	if (pin >= C4_GPIOA_PIN_START && pin <= C4_GPIOA_PIN_END)
		return  C4_GPIOA_PUPD_REG_OFFSET;
	if (pin >= C4_GPIOX_PIN_START && pin <= C4_GPIOX_PIN_END)
		return	C4_GPIOX_PUPD_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO bit
//
/*----------------------------------------------------------------------------*/
static int gpioToShiftReg (int pin)
{
	if (pin >= C4_GPIOH_PIN_START && pin <= C4_GPIOH_PIN_END)
		return  pin - C4_GPIOH_PIN_START;
	if (pin >= C4_GPIOA_PIN_START && pin <= C4_GPIOA_PIN_END)
		return  pin - C4_GPIOA_PIN_START;
	if (pin >= C4_GPIOX_PIN_START && pin <= C4_GPIOX_PIN_END)
		return  pin - C4_GPIOX_PIN_START;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Function register
//
/*----------------------------------------------------------------------------*/
static int gpioToGPFSELReg (int pin)
{
	if (pin >= C4_GPIOH_PIN_START && pin <= C4_GPIOH_PIN_END)
		return  C4_GPIOH_FSEL_REG_OFFSET;
	if(pin >= C4_GPIOA_PIN_START && pin <= C4_GPIOA_PIN_END)
		return  C4_GPIOA_FSEL_REG_OFFSET;
	if(pin >= C4_GPIOX_PIN_START && pin <= C4_GPIOX_PIN_END)
		return  C4_GPIOX_FSEL_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Drive Strength register
//
/*----------------------------------------------------------------------------*/
static int gpioToDSReg (int pin)
{
	if (pin >= C4_GPIOH_PIN_START && pin <= C4_GPIOH_PIN_END)
		return  C4_GPIOH_DS_REG_3A_OFFSET;
	if (pin >= C4_GPIOA_PIN_START && pin <= C4_GPIOA_PIN_END)
		return  C4_GPIOA_DS_REG_5A_OFFSET;
	if (pin >= C4_GPIOX_PIN_START && pin <= C4_GPIOX_PIN_MID)
		return  C4_GPIOX_DS_REG_2A_OFFSET;
	if (pin > C4_GPIOX_PIN_MID && pin <= C4_GPIOX_PIN_END)
		return  C4_GPIOX_DS_REG_2B_OFFSET;
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
	case	C4_GPIOH_PIN_START	...C4_GPIOH_PIN_END:
		return  C4_GPIOH_MUX_B_REG_OFFSET;
	case	C4_GPIOA_PIN_START	...C4_GPIOA_PIN_START + 7:
		return  C4_GPIOA_MUX_D_REG_OFFSET;
	case	C4_GPIOA_PIN_START + 8	...C4_GPIOA_PIN_END:
		return  C4_GPIOA_MUX_E_REG_OFFSET;
	case	C4_GPIOX_PIN_START	...C4_GPIOX_PIN_START + 7:
		return  C4_GPIOX_MUX_3_REG_OFFSET;
	case	C4_GPIOX_PIN_START + 8	...C4_GPIOX_PIN_START + 15:
		return  C4_GPIOX_MUX_4_REG_OFFSET;
	case	C4_GPIOX_PIN_START + 16	...C4_GPIOX_PIN_END:
		return  C4_GPIOX_MUX_5_REG_OFFSET;
	default:
		return -1;
	}
}

/*----------------------------------------------------------------------------*/
//
// config pwm sys path. "/sys/class/pwm/pwmchip?"
//
/*----------------------------------------------------------------------------*/
static int pinToSysPwmPath (int pin)
{
	const char *pwmGroup;
	char pwmLinkSrc[(BLOCK_SIZE / 8)];
	char pwmPath[(BLOCK_SIZE / 8)];
	int sz_link;

	memset(pwmLinkSrc, 0, sizeof(pwmLinkSrc));
	memset(pwmPath, 0, sizeof(pwmPath));

	pwmGroup = pinToPwm[pin];
	pwm = opendir("/sys/class/pwm");
	if (pwm == NULL) {
		printf("need to set device: pwm\n");
		return -1;
	}

	while (1) {
		pwmchip = readdir(pwm);

		if (pwmchip == NULL) {
			break;
		}

		if (strlen(pwmchip->d_name) <= 2)
			continue;

		sprintf(pwmPath, "%s/%s", "/sys/class/pwm", pwmchip->d_name);
		sz_link = readlink(pwmPath, pwmLinkSrc, sizeof(pwmLinkSrc));
		if (sz_link < 0) {
			perror("Read symbolic link fail");
			return sz_link;
		}

		if (strstr(pwmLinkSrc, pwmGroup) != NULL) {
			strncpy(sysPwmPath, pwmPath, (sizeof(sysPwmPath) - 1));
			break;
		}
	}
	closedir(pwm);

	return 0;
}

static int pwmSetup (int pin) {
	char cmd[(BLOCK_SIZE * 2)];
	int pwmPin, ret;

	memset(cmd, 0, sizeof(cmd));
	memset(pwmExport, 0, sizeof(pwmExport));

	if ((ret = pinToSysPwmPath(pin)) < 0) {
		perror("set pwm dtb overlays");
		return ret;
	}

	if (strstr(sysPwmPath, "pwmchip") == NULL) {
		printf("config pwm dtb overlays\n");
		return -1;
	}

	pwmPin = pinToPwmNum[pin];
	pwmClock = C4_PWM_INTERNAL_CLK;
	sprintf(pwmExport, "%d", (pwmPin % 2));
	sprintf(pwmPinPath[pwmPin], "%s/pwm%d", sysPwmPath, (pwmPin % 2));
	strncpy(setupedPwmPinPath[pwmPin], pwmPinPath[pwmPin], (BLOCK_SIZE - 1));
#ifdef ANDROID
	sprintf(cmd, "su -s sh -c %s %s", SYS_ACCESS_SCRIPT, pwmPinPath[pwmPin]);
#else
	sprintf(cmd, "sudo sh %s %s", SYS_ACCESS_SCRIPT, pwmPinPath[pwmPin]);
#endif
	inputToSysNode(sysPwmPath, "export", pwmExport);
	system(cmd);
	printf("PWM/pin%d: Don't change to gpio mode with overlay registered.\n", pin);

	return 0;
}

static int pwmRelease (int pin) {
	int pwmPin, ret;

	if ((ret = pinToSysPwmPath(pin)) < 0) {
		return ret;
	}

	if (strstr(sysPwmPath, "pwmchip") == NULL) {
		return -1;
	}

	pwmPin = pinToPwmNum[pin];
	sprintf(pwmUnexport, "%d", (pwmPin % 2));
	sprintf(pwmPinPath[pwmPin], "%s/pwm%d", sysPwmPath, (pwmPin % 2));
	if ((pwm = opendir(pwmPinPath[pwmPin])) != NULL) {
		inputToSysNode(pwmPinPath[pwmPin], "enable", "0");
		inputToSysNode(sysPwmPath, "unexport", pwmUnexport);
		closedir(pwm);
	}

	return 0;
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
static int _setDrive (int pin, int value)
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
	shift = pin > C4_GPIOX_PIN_MID ? (shift - 16) * 2 : shift * 2;

	*(gpio + ds) &= ~(0b11 << shift);
	*(gpio + ds) |= (value << shift);

	return 0;
}

/*----------------------------------------------------------------------------*/
static int _getDrive (int pin)
{
	int ds, shift;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	ds    = gpioToDSReg(pin);
	shift = gpioToShiftReg(pin);
	shift = pin > C4_GPIOX_PIN_MID ? (shift - 16) * 2 : shift * 2;

	return (*(gpio + ds)	>> shift) & 0b11;
}

/*----------------------------------------------------------------------------*/
static int _pinMode (int pin, int mode)
{
	int fsel, shift, origPin = pin;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	pwmRelease (origPin);
	softPwmStop  (origPin);
	softToneStop (origPin);

	fsel  = gpioToGPFSELReg(pin);
	shift = gpioToShiftReg (pin);

	switch (mode) {
	case	INPUT:
		*(gpio + fsel) = (*(gpio + fsel) | (1 << shift));
		break;
	case	OUTPUT:
		*(gpio + fsel) = (*(gpio + fsel) & ~(1 << shift));
		break;
	case	SOFT_PWM_OUTPUT:
		softPwmCreate (pin, 0, 100);
		break;
	case	SOFT_TONE_OUTPUT:
		softToneCreate (pin);
		break;
	case	PWM_OUTPUT:
		pwmSetup(origPin);
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
		return *(gpio + pupd) & (1 << shift) ? 2 : 1;
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
	unsigned int duty;
	int pwmPin;

	memset(pwmDuty, 0, sizeof(pwmDuty));

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if (((unsigned int)value > pwmRange) || (pwmRange <= 0)) {
		printf("warn : pwm range value is greater than or equal pwmWrite's\n");
		return -1;
	}

	pwmPin = pinToPwmNum[pin];
	duty = ((value * 100) / pwmRange);
	sprintf(pwmDuty, "%d", ((atoi(pwmPeriod) * duty) / 100));

	inputToSysNode(pwmPinPath[pwmPin], "duty_cycle", pwmDuty);

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
	case	2:	case	25:
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

	lseek (adcFds [pin], 0L, SEEK_SET);
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

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	gpiox.wvalue = *(gpio + C4_GPIOX_INP_REG_OFFSET);

	/* Wiring PI GPIO0 = C4 GPIOX.3 */
	gpiox.bits.bit3 = (value & 0x01);
	/* Wiring PI GPIO1 = C4 GPIOX.16 */
	gpiox.bits.bit16 = (value & 0x02);
	/* Wiring PI GPIO2 = C4 GPIOX.4 */
	gpiox.bits.bit4 = (value & 0x04);
	/* Wiring PI GPIO3 = C4 GPIOX.7 */
	gpiox.bits.bit7 = (value & 0x08);
	/* Wiring PI GPIO4 = C4 GPIOX.0 */
	gpiox.bits.bit0 = (value & 0x10);
	/* Wiring PI GPIO5 = C4 GPIOX.1 */
	gpiox.bits.bit1 = (value & 0x20);
	/* Wiring PI GPIO6 = C4 GPIOX.2 */
	gpiox.bits.bit2 = (value & 0x40);
	/* Wiring PI GPIO7 = C4 GPIOX.5 */
	gpiox.bits.bit5 = (value & 0x80);

	*(gpio + C4_GPIOX_OUTP_REG_OFFSET) = gpiox.wvalue;

	return 0;
}

/*----------------------------------------------------------------------------*/
static unsigned int _digitalReadByte (void)
{
	union	reg_bitfield	gpiox;
	unsigned int		value = 0;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	gpiox.wvalue = *(gpio + C4_GPIOX_INP_REG_OFFSET);

	/* Wiring PI GPIO0 = C4 GPIOX.3 */
	if (gpiox.bits.bit3)
		value |= 0x01;
	/* Wiring PI GPIO1 = C4 GPIOX.16 */
	if (gpiox.bits.bit16)
		value |= 0x02;
	/* Wiring PI GPIO2 = C4 GPIOX.4 */
	if (gpiox.bits.bit4)
		value |= 0x04;
	/* Wiring PI GPIO3 = C4 GPIOX.7 */
	if (gpiox.bits.bit7)
		value |= 0x08;
	/* Wiring PI GPIO4 = C4 GPIOX.0 */
	if (gpiox.bits.bit0)
		value |= 0x10;
	/* Wiring PI GPIO5 = C4 GPIOX.1 */
	if (gpiox.bits.bit1)
		value |= 0x20;
	/* Wiring PI GPIO6 = C4 GPIOX.2 */
	if (gpiox.bits.bit2)
		value |= 0x40;
	/* Wiring PI GPIO7 = C4 GPIOX.5 */
	if (gpiox.bits.bit5)
		value |= 0x80;

	return	value;
}

/*----------------------------------------------------------------------------*/
// PWM signal ___-----------___________---------------_______-----_
//               <--value-->           <----value---->
//               <-------range--------><-------range-------->
// PWM frequency == (PWM clock) / range
/*----------------------------------------------------------------------------*/

static void _pwmSetRange (unsigned int range)
{
	unsigned int freq, period;

	memset(pwmPeriod, 0, sizeof(pwmPeriod));

	if (lib->mode == MODE_GPIO_SYS)
		return;

	if (pwmClock < 2) {
		printf("error : pwm freq: %dMHz / (pwmSetClock's value) >= 2\n",
				(C4_PWM_INTERNAL_CLK / 1000000));
		return;
	}

	pwmRange = range;
	if ((pwmRange < 1) || (pwmRange >= pwmClock)) {
		printf("error : invalied value. ( < pwm freq)\n");
		return;
	}

	freq = (pwmClock / pwmRange);
	period = (1000000000 / freq); // period: s to ns.
	sprintf(pwmPeriod, "%d", period);

	for (int i = 0; i < 10; i++) {
		if (strstr(setupedPwmPinPath[i], "None") != NULL)
			continue;
		inputToSysNode(setupedPwmPinPath[i], "period", pwmPeriod);
		inputToSysNode(setupedPwmPinPath[i], "polarity", "normal");
		inputToSysNode(setupedPwmPinPath[i], "enable", "1");
	}
}


/*----------------------------------------------------------------------------*/
// Internal clock == 24MHz
// PWM clock == (Internal clock) / divisor
// PWM frequency == (PWM clock) / range
/*----------------------------------------------------------------------------*/

static void _pwmSetClock (int divisor)
{
	if (pwmClock > 0)
		pwmClock = (pwmClock / divisor);
	else {
		printf("error : pwm mode error\n");
		return;
	}
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
			setUsingGpiomem(TRUE);
		} else
			msg (MSG_ERR,
				"wiringPiSetup: /dev/gpiomem doesn't exist. Please try again with sudo.\n");
	}

	if (fd < 0) {
		msg(MSG_ERR, "wiringPiSetup: Cannot open memory area for GPIO use. \n");
	} else {
		// #define C4_GPIO_BASE		0xff634000
#ifdef ANDROID
#if defined(__aarch64__)
		mapped = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, C4_GPIO_BASE);
#else
		mapped = mmap64(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, (off64_t)C4_GPIO_BASE);
#endif
#else
		mapped = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, C4_GPIO_BASE);
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
	const char *AIN25_NODE, *AIN29_NODE;

	/* ADC node setup */
	if (cmpKernelVersion(KERN_NUM_TO_MAJOR, 5)) {
		AIN25_NODE = "/sys/devices/platform/soc/ff800000.bus/ff809000.adc/iio:device0/in_voltage2_raw";
		AIN29_NODE = "/sys/devices/platform/soc/ff800000.bus/ff809000.adc/iio:device0/in_voltage0_raw";
	}

	else {
		AIN25_NODE = "/sys/devices/platform/ff809000.saradc/iio:device0/in_voltage2_raw";
		AIN29_NODE = "/sys/devices/platform/ff809000.saradc/iio:device0/in_voltage0_raw";
	}

	adcFds[0] = open(AIN25_NODE, O_RDONLY);
	adcFds[1] = open(AIN29_NODE, O_RDONLY);
}

/*----------------------------------------------------------------------------*/
void init_odroidc4 (struct libodroid *libwiring)
{
	init_gpio_mmap();

	init_adc_fds();

	/* wiringPi Core function initialize */
	libwiring->getModeToGpio	= _getModeToGpio;
	libwiring->setDrive		= _setDrive;
	libwiring->getDrive		= _getDrive;
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
	libwiring->pinBase		= C4_GPIO_PIN_BASE;

	/* global variable setup */
	lib = libwiring;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
