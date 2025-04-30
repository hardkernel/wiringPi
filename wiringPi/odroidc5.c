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
#include <sys/mman.h>
#include <sys/stat.h>

/*----------------------------------------------------------------------------*/
#include "softPwm.h"
#include "softTone.h"

/*----------------------------------------------------------------------------*/
#include "wiringPi.h"
#include "odroidc5.h"

/*----------------------------------------------------------------------------*/
// wiringPi gpio map define
/*----------------------------------------------------------------------------*/
static const int pinToGpio[64] = {
	// wiringPi number to native gpio number
	481, 482,	//  0 |  1
    491, 480,	//  2 |  3
    458, 459,	//  4 |  5
    490, 456,	//  6 |  7
    493, 494,	//  8 |  9
    486, 483,	// 10 | 11
    484, 485,	// 12 | 13
    487, 488,	// 14 | 15
    489, -1,	// 16 | 17
    -1, -1,		// 18 | 19
    -1, 468,	// 20 | 21
    469, 476,	// 22 | 23
    477, -1,	// 24 | 25
    478, 479,	// 26 | 27
    -1, -1,		// 28 | 29
	455, 454,	// 30 | 31
	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 32...47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1	// 48...63
};

static const int phyToGpio[64] = {
	// physical header pin number to native gpio number
	 -1,		//  0
	 -1,  -1,	//  1 |  2
	493,  -1,	//  3 |  4
	494,  -1,	//  5 |  6
	456, 488,	//  7 |  8
	 -1, 489,	//  9 | 10
	481, 482,	// 11 | 12
	491,  -1,	// 13 | 14
	480, 458,	// 15 | 16
	 -1, 459,	// 17 | 18
	484,  -1,	// 19 | 20
	485, 490,	// 21 | 22
	487, 486,	// 23 | 24
	 -1, 433,	// 25 | 26
	455, 454,	// 27 | 28
	468,  -1,	// 29 | 30
	469, 478,	// 31 | 32
	476,  -1,	// 33 | 34
	477, 479,	// 35 | 36
	 -1,  -1,	// 37 | 38
	 -1,  -1,	// 39 | 40
	// Not used
	-1, -1, -1, -1, -1, -1, -1, -1,	// 41...48
	-1, -1, -1, -1, -1, -1, -1, -1,	// 49...56
	-1, -1, -1, -1, -1, -1, -1	// 57...63
};

static const char *pinToPwm[64] = {
	// wiringPi number to pwm group number
		"None", "fe058000",		//  0 |  1 : PWM_A
		"None", "None",			//  2 |  3
		"None", "fe058400",		//  4 |  5 :      , PWM_C
		"None", "fe058200",		//  6 |  7 :      , PWM_B
		"None", "None",			//  8 |  9
		"None", "fe058a00",		// 10 | 11 :      , PWM_F
		"None", "None",			// 12 | 13
		"None", "None",			// 14 | 15
		"None", "None",			// 16 | 17
		"None", "None",			// 18 | 19
		"None", "None",			// 20 | 21
		"None", "None",			// 22 | 23
		"None", "None",			// 24 | 25
		"None", "None",			// 26 | 27
		"None", "None",			// 28 | 29
		"None", "None",		    // 30 | 31
	// Padding:
	"None","None","None","None","None","None","None","None","None","None","None","None","None","None","None","None", // 32...47
	"None","None","None","None","None","None","None","None","None","None","None","None","None","None","None","None"  // 48...63
};

static const int pinToPwmNum[64] = {
	// wiringPi number to pwm pin number
	 -1,  1,	//  0 |  1
	 -1, -1,	//  2 |  3
	 -1,  0,	//  4 |  5 :      , PWM_C
	 -1,  2,	//  6 |  7 : PWM_A, PWM_B
	 -1, -1,	//  8 |  9
	 -1,  3,	// 10 | 11 :      , PWM_F
	 -1, -1,	// 12 | 13
	 -1, -1,	// 14 | 15
	 -1, -1,	// 16 | 17
	 -1, -1,	// 18 | 19
	 -1, -1,	// 20 | 21
	 -1, -1,	// 22 | 23
	  1, -1,	// 24 | 25
	 -1, -1,	// 26 | 27
	 -1, -1,	// 28 | 29
	 -1, -1,	// 30 | 31
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
static int gpioToOffset(int pin);
static int	gpioToInputReg	(int pin);
static int	gpioToOutputReg	(int pin);
static int	gpioToPullEnReg	(int pin);
static int	gpioToPullDirReg	(int pin);
static int	gpioToDirectionReg	(int pin);
static int	gpioToShiftReg	(int pin);
static int	gpioToDSReg	(int pin);
static int gpioToDSShift (int pin);
static int	gpioToMuxReg	(int pin);
static int gpioToMuxShift (int pin);
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

/*----------------------------------------------------------------------------*/

/**
 * Offset to the GPIO base offset
 * @param pin gpio number
 * @return Base register or -1
 */
static int gpioToOffset(int pin) {
	if (C5_IS_GPIO_X(pin))
		return C5_GPIO_X_OFFSET;
	if (C5_IS_GPIO_D(pin))
		return C5_GPIO_D_OFFSET;
	if (C5_IS_GPIO_H(pin))
		return C5_GPIO_H_OFFSET;
	if (C5_IS_GPIO_DV(pin))
		return C5_GPIO_DV_OFFSET;
	return -1;
}

/**
 * Offset to the GPIO output value register
 * @param pin gpio number
 * @return Output value register or -1
 */
static int gpioToOutputReg (int pin)
{
	int ret = gpioToOffset(pin);
	if (ret == -1) return -1;

	return ret + C5_GPIO_O_OFFSET;
}

/**
 * Offset to the GPIO input value register
 * @param pin gpio number
 * @return Input value register or -1
 */
static int gpioToInputReg (int pin)
{
	int ret = gpioToOffset(pin);
	if (ret == -1) return -1;

	return ret + C5_GPIO_I_OFFSET;
}

/**
 * Offset to the GPIO Pull up/down enable register
 * @param pin gpio number
 * @return Pull enable register or -1
 */
static int gpioToPullEnReg (int pin)
{
	int ret = gpioToOffset(pin);
	if (ret == -1) return -1;

	return ret + C5_GPIO_PULL_EN_OFFSET;
}

/**
 * Offset to the GPIO Pull up/down direction register
 * @param pin gpio number
 * @return Pull direction register or -1
 */
static int gpioToPullDirReg (int pin)
{
	int ret = gpioToOffset(pin);
	if (ret == -1) return -1;

	return ret + C5_GPIO_PULL_UP_OFFSET;
}

/**
 * Offset to the GPIO Direction register
 * @param pin gpio number
 * @return Direction register offset or -1
 */
static int gpioToDirectionReg (int pin)
{
	int ret = gpioToOffset(pin);
	if (ret == -1) return -1;

	return ret + C5_GPIO_OEN_OFFSET;
}

/**
 * Offset to the GPIO bit
 * @param pin gpio number
 * @return
 */
static int gpioToShiftReg (int pin)
{
	if (C5_IS_GPIO_X(pin))
		return pin - C5_GPIO_X_WPI_START;
	if (C5_IS_GPIO_D(pin))
		return pin - C5_GPIO_D_WPI_START;
	if (C5_IS_GPIO_H(pin))
		return pin - C5_GPIO_H_WPI_START;
	if (C5_IS_GPIO_DV(pin))
		return pin - C5_GPIO_DV_WPI_START;
	return -1;
}

/**
 * Offset to the GPIO DS register
 * @param pin gpio number
 * @return DS register or -1
 */
static int gpioToDSReg (int pin)
{
	int x = gpioToOffset(pin);
	if (x == -1) return -1;

	if (C5_IS_GPIO_X_EXT(pin))
		return x + C5_GPIO_DS_EXT_OFFSET;

	return x + C5_GPIO_DS_OFFSET;
}

/**
 * Offset to the GPIO DS shift
 * @param pin gpio number
 * @return DS register or -1
 */
static int gpioToDSShift (int pin)
{
	if (C5_IS_GPIO_X_EXT(pin))
		return pin - C5_GPIO_X(16);
	return gpioToShiftReg(pin);
}

/**
 * Offset to the pin mux register
 * @param pin gpio number
 * @return Pin mux register or -1
 */
static int gpioToMuxReg (int pin)
{
	if (pin == C5_GPIO_DV(1) || pin == C5_GPIO_DV(2))
		return C5_PIN_MUX_REG2_OFFSET;
	if (pin >= C5_GPIO_X(0) && pin <= C5_GPIO_X(7))
		return C5_PIN_MUX_REG3_OFFSET;
	if ((pin >= C5_GPIO_X(8) && pin <= C5_GPIO_X(15)))
		return C5_PIN_MUX_REG4_OFFSET;
	if (pin >= C5_GPIO_X(17) && pin <= C5_GPIO_X(18))
		return C5_PIN_MUX_REG5_OFFSET;
	if (pin >= C5_GPIO_H(4) && pin <= C5_GPIO_H(5))
		return C5_PIN_MUX_REGB_OFFSET;
	if (pin >= C5_GPIO_D(2) && pin <= C5_GPIO_D(4))
		return C5_PIN_MUX_REGG_OFFSET;

	return	-1;
}

/**
 * Offset to the pin mux shift
 * @param pin gpio number
 * @return Pin mux shift or -1
 */
static int gpioToMuxShift (int pin)
{
	if (pin == C5_GPIO_DV(1) || pin == C5_GPIO_DV(2))
		return pin - C5_GPIO_DV(0);
	if (pin >= C5_GPIO_X(0) && pin <= C5_GPIO_X(7))
		return pin - C5_GPIO_X(0);
	if ((pin >= C5_GPIO_X(8) && pin <= C5_GPIO_X(15)))
		return pin - C5_GPIO_X(8);
	if (pin >= C5_GPIO_X(17) && pin <= C5_GPIO_X(18))
		return pin - C5_GPIO_X(16);
	if (pin >= C5_GPIO_H(4) && pin <= C5_GPIO_H(5))
		return pin - C5_GPIO_H(0);
	if (pin >= C5_GPIO_D(2) && pin <= C5_GPIO_D(4))
		return pin - C5_GPIO_D(0);

	return	-1;
}

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
	pwmClock = C5_PWM_INTERNAL_CLK;
	sprintf(pwmExport, "%d", 0);
	sprintf(pwmPinPath[pwmPin], "%s/pwm%d", sysPwmPath, 0);
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
	sprintf(pwmUnexport, "%d", 0);
	sprintf(pwmPinPath[pwmPin], "%s/pwm%d", sysPwmPath, 0);
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
	shift = gpioToDSShift(pin);

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
	shift = gpioToDSShift(pin);

	return (*(gpio + ds)	>> shift) & 0b11;
}

/*----------------------------------------------------------------------------*/
static int _pinMode (int pin, int mode)
{
	int dir, shift, origPin = pin;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	pwmRelease (origPin);
	softPwmStop  (origPin);
	softToneStop (origPin);

	dir  = gpioToDirectionReg(pin);
	shift = gpioToShiftReg (pin);

	switch (mode) {
	case	INPUT:
		*(gpio + dir) = (*(gpio + dir) | (1 << shift));
		break;
	case	OUTPUT:
		*(gpio + dir) = (*(gpio + dir) & ~(1 << shift));
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
	int dir, mux, shift, target, mode;

	if (lib->mode == MODE_GPIO_SYS)
		return	-1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return	-1;

	dir   = gpioToDirectionReg(pin);
	mux    = gpioToMuxReg(pin);
	shift = gpioToShiftReg (pin);
	target = gpioToMuxShift(pin);

	while (target >= 8) {
		target -= 8;
	}

	mode = (*(gpio + mux) >> (target * 4)) & 0xF;
	return	mode ? mode + 1 : (*(gpio + dir) & (1 << shift)) ? 0 : 1;
}

/*----------------------------------------------------------------------------*/
static int _getPUPD (int pin)
{
	int puen, pupd, shift;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	puen  = gpioToPullEnReg(pin);
	pupd  = gpioToPullDirReg(pin);
	shift = gpioToShiftReg(pin);

	if (*(gpio + puen) & (1 << shift)) return *(gpio + pupd) & (1 << shift) ? 1 : 2;
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
		*(gpio + gpioToPullEnReg(pin)) =
			(*(gpio + gpioToPullEnReg(pin)) | (1 << shift));

		if (pud == PUD_UP)
			*(gpio + gpioToPullDirReg(pin)) =
				(*(gpio + gpioToPullDirReg(pin)) |  (1 << shift));
		else
			*(gpio + gpioToPullDirReg(pin)) =
				(*(gpio + gpioToPullDirReg(pin)) & ~(1 << shift));
	} else	// Disable Pull/Pull-down resister
		*(gpio + gpioToPullEnReg(pin)) =
			(*(gpio + gpioToPullEnReg(pin)) & ~(1 << shift));

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

	if ((*(gpio + gpioToInputReg(pin)) & (1 << gpioToShiftReg(pin))) != 0)
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
		*(gpio + gpioToOutputReg(pin)) &= ~(1 << gpioToShiftReg(pin));
	else
		*(gpio + gpioToOutputReg(pin)) |=  (1 << gpioToShiftReg(pin));

	return 0;
}

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

static int _analogRead (int pin)
{
	char value[10] = {0,};
	int n;

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
	n = read(adcFds [pin], &value[0], 10);
	if (n < 0) {
		msg(MSG_WARN, "%s: Error occurs when it reads from ADC file descriptor. \n", __func__);
		return -1;
	}

	value[n] = 0;

	return	atoi(value);
}

/*----------------------------------------------------------------------------*/
static int _digitalWriteByte (const unsigned int value)
{
	union	reg_bitfield	gpiox;
	union	reg_bitfield	gpiod;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	gpiox.wvalue = *(gpio + C5_GPIO_X_OFFSET + C5_GPIO_I_OFFSET);
	gpiod.wvalue = *(gpio + C5_GPIO_D_OFFSET + C5_GPIO_I_OFFSET);

	/* Wiring PI GPIO0 = C5 GPIOX.5 */
	gpiox.bits.bit5 = (value & 0x01);
	/* Wiring PI GPIO1 = C5 GPIOX.16 */
	gpiox.bits.bit16 = (value & 0x02);
	/* Wiring PI GPIO2 = C5 GPIOX.15 */
	gpiox.bits.bit15 = (value & 0x04);
	/* Wiring PI GPIO3 = C5 GPIOX.4 */
	gpiox.bits.bit4 = (value & 0x08);
	/* Wiring PI GPIO4 = C5 GPIOD.2 */
	gpiod.bits.bit2 = (value & 0x10);
	/* Wiring PI GPIO5 = C5 GPIOD.3 */
	gpiod.bits.bit3 = (value & 0x20);
	/* Wiring PI GPIO6 = C5 GPIOX.6 */
	gpiox.bits.bit6 = (value & 0x40);
	/* Wiring PI GPIO7 = C5 GPIOD.4 */
	gpiod.bits.bit4 = (value & 0x80);

	*(gpio + C5_GPIO_X_OFFSET + C5_GPIO_O_OFFSET) = gpiox.wvalue;
	*(gpio + C5_GPIO_D_OFFSET + C5_GPIO_O_OFFSET) = gpiod.wvalue;

	return 0;
}

static unsigned int _digitalReadByte (void)
{
	union	reg_bitfield	gpiox;
	union	reg_bitfield	gpiod;

	unsigned int		value = 0;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	gpiox.wvalue = *(gpio + C5_GPIO_X_OFFSET + C5_GPIO_I_OFFSET);
	gpiod.wvalue = *(gpio + C5_GPIO_D_OFFSET + C5_GPIO_I_OFFSET);

	/* Wiring PI GPIO0 = C5 GPIOX.5 */
	if (gpiox.bits.bit5)
		value |= 0x01;
	/* Wiring PI GPIO1 = C5 GPIOX.16 */
	if (gpiox.bits.bit16)
		value |= 0x02;
	/* Wiring PI GPIO2 = C5 GPIOX.15 */
	if (gpiox.bits.bit15)
		value |= 0x04;
	/* Wiring PI GPIO3 = C5 GPIOX.4 */
	if (gpiox.bits.bit4)
		value |= 0x08;
	/* Wiring PI GPIO4 = C5 GPIOD.2 */
	if (gpiod.bits.bit2)
		value |= 0x10;
	/* Wiring PI GPIO5 = C5 GPIOD.3 */
	if (gpiod.bits.bit3)
		value |= 0x20;
	/* Wiring PI GPIO6 = C5 GPIOX.6 */
	if (gpiox.bits.bit6)
		value |= 0x40;
	/* Wiring PI GPIO7 = C5 GPIOD.4 */
	if (gpiod.bits.bit4)
		value |= 0x80;

	return	value;
}

static void _pwmSetRange (unsigned int range)
{
	unsigned int freq, period;

	memset(pwmPeriod, 0, sizeof(pwmPeriod));

	if (lib->mode == MODE_GPIO_SYS)
		return;

	if (pwmClock < 2) {
		printf("error : pwm freq: %dMHz / (pwmSetClock's value) >= 2\n",
				(C5_PWM_INTERNAL_CLK / 1000000));
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

static void _pwmSetClock (int divisor)
{
	if (pwmClock > 0)
		pwmClock = (pwmClock / divisor);
	else {
		printf("error : pwm mode error\n");
		return;
	}
}

static void init_gpio_mmap (void)
{
	int fd = -1;
	void *mapped;

	if (access("/dev/gpiomem",0) == 0) {
		if ((fd = open ("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
			msg (MSG_ERR,
				"wiringPiSetup: Unable to open /dev/gpiomem: %s\n",
				strerror (errno));
		setUsingGpiomem(TRUE);
	} else
		msg (MSG_ERR,
			"wiringPiSetup: /dev/gpiomem doesn't exist. Please try again with sudo.\n");

	if (fd < 0) {
		msg(MSG_ERR, "wiringPiSetup: Cannot open memory area for GPIO use. \n");
	} else {
		// #define C4_GPIO_BASE		0xff634000
#ifdef ANDROID
#if defined(__aarch64__)
		mapped = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, C5_GPIO_BASE);
#else
		mapped = mmap64(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, (off64_t)C5_GPIO_BASE);
#endif
#else
		mapped = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, C5_GPIO_BASE);
#endif

		if (mapped == MAP_FAILED)
			msg(MSG_ERR, "wiringPiSetup: mmap (GPIO) failed: %s \n", strerror (errno));
		else
			gpio = (uint32_t *) mapped;
	}
}

static void init_adc_fds (void)
{
	const char *AIN25_NODE, *AIN29_NODE;

	/* ADC node setup */
	AIN25_NODE = "/sys/bus/iio/devices/iio:device0/in_voltage0_raw";
	AIN29_NODE = "/sys/bus/iio/devices/iio:device0/in_voltage1_raw";

	adcFds[0] = open(AIN25_NODE, O_RDONLY);
	adcFds[1] = open(AIN29_NODE, O_RDONLY);
}

/*----------------------------------------------------------------------------*/
void init_odroidc5 (struct libodroid *libwiring)
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
	libwiring->pinBase		= C5_GPIO_D_WPI_START;

	/* global variable setup */
	lib = libwiring;
}
