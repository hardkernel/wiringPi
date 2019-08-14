/*----------------------------------------------------------------------------*/
//
//
//	WiringPi ODROID-N1 Board Control file (ROCKCHIP 64Bits Platform)
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
#include "odroidn1.h"

/*----------------------------------------------------------------------------*/
// wiringPi gpio map define
/*----------------------------------------------------------------------------*/
static const int pinToGpio[64] = {
	// wiringPi number to native gpio number
	33,  34,	//  0 |  1 : GPIO1_A.1, GPIO1_A.2
	35,  36,	//  2 |  3 : GPIO1_A.3, GPIO1_A.4
	45,  50,	//  4 |  5 : GPIO1_B.5, GPIO1_C.2
	56,  32,	//  6 |  7 : GPIO1_D.0, GPIO1_A.0
	43,  44,	//  8 |  9 : GPIO1_B.3(I2C4_SDA), GPIO1_B.4(I2C4_SCL)
	42,  54,	// 10 | 11 : GPIO1_B.2(SPI1_CSN), GPIO1_C.6
	40,  39,	// 12 | 13 : GPIO1_B.0(SPI1_TXD), GPIO1_A.7(SPI1_RXD)
	41,  81,	// 14 | 15 : GPIO1_B.1(SPI1_CLK), GPIO2_C.1_B(UART0_TX)
	80,  -1,	// 16 | 17 : GPIO2_C.0_B(UART0_RX)
	-1,  -1,	// 18 | 19 :
	-1, 149,	// 20 | 21 :  , GPIO4_C.5(SPDIF_TX)
	150,152,	// 22 | 23 : GPIO4_C.6(PWM1), CPIO4_D.0
	157, -1,	// 24 | 25 : GPIO4_D.5, ADC.AIN1
	156,158,	// 26 | 27 : GPIO4_D.4, GPIO4_D.6
	-1,  -1,	// 28 | 29 : 18.V REF, ADC.AIN0
	 52, 53,	// 30 | 31 : GPIO1_C.4(I2C8_SDA), GPIO1_C.5(I2C8_SCL)

	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 32...47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 48...63
};

static const int phyToGpio[64] = {
	// physical header pin number to native gpio number
	-1,		//  0
	-1,  -1,	//  1 |  2 : 3.0V, 5.0V
	43,  -1,	//  3 |  4 : GPIO1_B.3(I2C4_SDA), 5.0V
	44,  -1,	//  5 |  6 : GPIO1_B.4(I2C4_SCL), GND
	32,  81,	//  7 |  8 : GPIO1_A.0, GPIO2_C.1_B(UART0_TX)
	-1,  80,	//  9 | 10 : GND, GPIO2_C.0_B(UART0_RX)
	33,  34,	// 11 | 12 : GPIO1_A.1, GPIO1_A.2
	35,  -1,	// 13 | 14 : GPIO1_A.3, GND
	36,  45,	// 15 | 16 : GPIO1_A.4, GPIO1_B.5
	-1,  50,	// 17 | 18 : 3.0V, GPIO1_C.2
	40,  -1,	// 19 | 20 : GPIO1_B.0(SPI1_TXD), GND
	39,  56,	// 21 | 22 : GPIO1_A.7(SPI1_RXD), GPIO1_D.0
	41,  42,	// 23 | 24 : GPIO1_B.1(SPI1_CLK), GPIO1_B.2(SPI1_CSN0)
	-1,  54,	// 25 | 26 : GND, GPIO1_C.6
	52,  53,	// 27 | 28 : GPIO1_C.4(I2C8_SDA), GPIO1_C.5(I2C8_SCL)
	149, -1,	// 29 | 30 : GPIO4_C.5(SPDIF_TX), GND
	150,156,	// 31 | 32 : GPIO4_C.6(PWM1), GPIO4_D.4
	152, -1,	// 33 | 34 : GPIO4_D.0, GND
	157,158,	// 35 | 36 : GPIO4_D.5, GPIO4_D.6
	-1,  -1,	// 37 | 38 : ADC.AIN1, 1.8V REF
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

/* GPIO mmap control. Actual GPIO bank number. */
static volatile uint32_t *gpio[5];

/* GRF(General Register Files) base addresses to control GPIO modes */
static volatile uint32_t *grf[2];

/* CRU(Clock & Reset Unit) base addresses to control CLK mode */
static volatile uint32_t *cru[2];

/* wiringPi Global library */
static struct libodroid	*lib = NULL;

/*----------------------------------------------------------------------------*/
// Function prototype define
/*----------------------------------------------------------------------------*/
static int	gpioToShiftReg	(int pin);
static int	gpioToShiftGReg	(int pin);
static void	setClkState	(int pin, int state);
static void	setIomuxMode 	(int pin, int mode);

/*----------------------------------------------------------------------------*/
// wiringPi core function
/*----------------------------------------------------------------------------*/
static int		_getModeToGpio		(int mode, int pin);
static int		_pinMode		(int pin, int mode);
static int		_getAlt			(int pin);
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

	void init_odroidn1 	(struct libodroid *libwiring);

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
//
// offset to the GPIO bit
//
/*----------------------------------------------------------------------------*/
static int gpioToShiftReg (int pin)
{
	return pin % 32;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO bit at GRF address
//
/*----------------------------------------------------------------------------*/
static int gpioToShiftGReg (int pin)
{
	return pin % 8;
}

/*----------------------------------------------------------------------------*/
static int _getModeToGpio (int mode, int pin)
{
	if (pin > 255)
		return msg(MSG_ERR, "%s : Invalid pin number %d\n", __func__, pin);

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
		msg(MSG_WARN, "%s : Unknown Mode %d\n", __func__, mode);
		return	-1;
	}
}

/*----------------------------------------------------------------------------*/
//
// set GPIO clock state
//
/*----------------------------------------------------------------------------*/
static void setClkState (int pin, int state)
{
	uint32_t target = 0;
	uint8_t bank = pin / 32;
	uint8_t shift = bank < 2 ? bank + 2 : bank + 1;

	target |= (1 << (shift + 16));

	switch (state) {
	case N1_CLK_ENABLE:
		if (bank < 2) {
			target |= *(cru[0] + (N1_PMUCRU_GPIO_CLK_OFFSET >> 2));
			target &= ~(1 << shift);
			*(cru[0] + (N1_PMUCRU_GPIO_CLK_OFFSET >> 2)) = target;
		} else {
			target |= *(cru[1] + (N1_CRU_GPIO_CLK_OFFSET >> 2));
			target &= ~(1 << shift);
			*(cru[1] + (N1_CRU_GPIO_CLK_OFFSET >> 2)) = target;
		}
		break;
	case N1_CLK_DISABLE:
		if (bank < 2) {
			target |= *(cru[0] + (N1_PMUCRU_GPIO_CLK_OFFSET >> 2));
			target |=  (1 << shift);
			*(cru[0] + (N1_PMUCRU_GPIO_CLK_OFFSET >> 2)) = target;
		} else {
			target |= *(cru[1] + (N1_CRU_GPIO_CLK_OFFSET >> 2));
			target |=  (1 << shift);
			*(cru[1] + (N1_CRU_GPIO_CLK_OFFSET >> 2)) = target;
		}
		break;
	default:
		break;
	}
}

/*----------------------------------------------------------------------------*/
//
// set IOMUX mode
//
/*----------------------------------------------------------------------------*/
static void setIomuxMode (int pin, int mode)
{
	uint32_t offset, target;
	uint8_t	bank, group;

	bank	= pin / 32;
	group	= (pin - bank * 32) / 8;
	offset	= 0x10 * (bank > 1 ? bank - 2 : bank) + 0x4 * group;
	target	= 0;

	target |= (1 << (gpioToShiftGReg(pin) * 2 + 17));
	target |= (1 << (gpioToShiftGReg(pin) * 2 + 16));

	switch (mode) {
	case N1_FUNC_GPIO:
		// Common IOMUX Funtion 1 : GPIO (0b00)
		if (bank < 2) {
			offset += N1_PMUGRF_IOMUX_OFFSET;

			target |= *(grf[0] + (offset >> 2));
			target &= ~(1 << (gpioToShiftGReg(pin) * 2 + 1));
			target &= ~(1 << (gpioToShiftGReg(pin) * 2));

			*(grf[0] + (offset >> 2)) = target;
		} else {
			offset += N1_GRF_IOMUX_OFFSET;

			target |= *(grf[1] + (offset >> 2));
			target &= ~(1 << (gpioToShiftGReg(pin) * 2 + 1));
			target &= ~(1 << (gpioToShiftGReg(pin) * 2));

			*(grf[1] + (offset >> 2)) = target;
		}
		break;
	default:
		break;
	}
}

/*----------------------------------------------------------------------------*/
static int _pinMode (int pin, int mode)
{
	int origPin, bank;
	unsigned long UNU flags;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	origPin = pin;
	bank = pin / 32;

	softPwmStop (origPin);
	softToneStop(origPin);

	setClkState (pin, N1_CLK_ENABLE);
	setIomuxMode(pin, N1_FUNC_GPIO);

	switch (mode) {
	case INPUT:
		*(gpio[bank] + (N1_GPIO_CON_OFFSET >> 2)) &= ~(1 << gpioToShiftReg(pin));
		_pullUpDnControl(origPin, PUD_OFF);
		break;
	case OUTPUT:
		*(gpio[bank] + (N1_GPIO_CON_OFFSET >> 2)) |=  (1 << gpioToShiftReg(pin));
		break;
	case INPUT_PULLUP:
		*(gpio[bank] + (N1_GPIO_CON_OFFSET >> 2)) &= ~(1 << gpioToShiftReg(pin));
		_pullUpDnControl(origPin, PUD_UP);
		break;
	case INPUT_PULLDOWN:
		*(gpio[bank] + (N1_GPIO_CON_OFFSET >> 2)) &= ~(1 << gpioToShiftReg(pin));
		_pullUpDnControl(origPin, PUD_DOWN);
		break;
	case SOFT_PWM_OUTPUT:
		softPwmCreate (origPin, 0, 100);
		break;
	case SOFT_TONE_OUTPUT:
		softToneCreate (origPin);
		break;
	default:
		msg(MSG_WARN, "%s : Unknown Mode %d\n", __func__, mode);
		return -1;
	}

	setClkState (pin, N1_CLK_DISABLE);

	return 0;
}

/*----------------------------------------------------------------------------*/
static int _getAlt (int pin)
{
	uint32_t offset;
	uint8_t	bank, group, shift;
	uint8_t ret = 0;

	if (lib->mode == MODE_GPIO_SYS)
		return	-1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return	-1;

	bank	= pin / 32;
	group	= (pin - bank * 32) / 8;
	offset	= 0x10 * (bank > 1 ? bank - 2 : bank) + 0x4 * group;
	shift	= gpioToShiftGReg(pin) << 1;

	setClkState(pin, N1_CLK_ENABLE);

	// Check if the pin is GPIO mode on GRF register
	if (bank < 2) {
		offset += N1_PMUGRF_IOMUX_OFFSET;
		ret = (*(grf[0] + (offset >> 2)) >> shift) & 0b11;
	} else {
		offset += N1_GRF_IOMUX_OFFSET;
		ret = (*(grf[1] + (offset >> 2)) >> shift) & 0b11;
	}

	// If it is GPIO mode, check it's direction
	if (ret == 0)
		ret = *(gpio[bank] + (N1_GPIO_CON_OFFSET >> 2)) & (1 << gpioToShiftReg(pin)) ? 1 : 0;
	else {
		// ALT1 is GPIO mode(0b00) on this SoC
		ret++;
	}

	setClkState(pin, N1_CLK_DISABLE);

	return ret;
}

/*----------------------------------------------------------------------------*/
static int _pullUpDnControl (int pin, int pud)
{
	uint32_t offset, target;
	uint8_t	bank, group;

	if (lib->mode == MODE_GPIO_SYS)
		return	-1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return	-1;

	bank	= pin / 32;
	group	= (pin - bank * 32) / 8;
	offset	= 0x10 * (bank > 1 ? bank - 2 : bank) + 0x4 * group;
	target	= 0;

	target |= (1 << (gpioToShiftGReg(pin) * 2 + 17));
	target |= (1 << (gpioToShiftGReg(pin) * 2 + 16));

	setClkState(pin, N1_CLK_ENABLE);

	switch (pud) {
	case PUD_UP:
		if (bank < 2) {
			offset += N1_PMUGRF_PUPD_OFFSET;

			target |= *(grf[0] + (offset >> 2));
			target &= ~(1 << (gpioToShiftGReg(pin) * 2 + 1));
			target |=  (1 << (gpioToShiftGReg(pin) * 2));

			*(grf[0] + (offset >> 2)) = target;
		} else {
			offset += N1_GRF_PUPD_OFFSET;

			target |= *(grf[1] + (offset >> 2));
			if (bank == 2 && group >= 2) {
				target |=  (1 << (gpioToShiftGReg(pin) * 2 + 1));
				target |=  (1 << (gpioToShiftGReg(pin) * 2));
			} else {
				target &= ~(1 << (gpioToShiftGReg(pin) * 2 + 1));
				target |=  (1 << (gpioToShiftGReg(pin) * 2));
			}

			*(grf[1] + (offset >> 2)) = target;
		}
		break;
	case PUD_DOWN:
		if (bank < 2) {
			offset += N1_PMUGRF_PUPD_OFFSET;

			target |= *(grf[0] + (offset >> 2));
			target |=  (1 << (gpioToShiftGReg(pin) * 2 + 1));
			target &= ~(1 << (gpioToShiftGReg(pin) * 2));

			*(grf[0] + (offset >> 2)) = target;
		} else {
			offset += N1_GRF_PUPD_OFFSET;

			target |= *(grf[1] + (offset >> 2));
			if (bank == 2 && group >= 2) {
				target &= ~(1 << (gpioToShiftGReg(pin) * 2 + 1));
				target |=  (1 << (gpioToShiftGReg(pin) * 2));
			} else {
				target |=  (1 << (gpioToShiftGReg(pin) * 2 + 1));
				target &= ~(1 << (gpioToShiftGReg(pin) * 2));
			}

			*(grf[1] + (offset >> 2)) = target;
		}
		break;
	case PUD_OFF:
		if (bank < 2) {
			offset += N1_PMUGRF_PUPD_OFFSET;

			target |= *(grf[0] + (offset >> 2));
			target &= ~(1 << (gpioToShiftGReg(pin) * 2 + 1));
			target &= ~(1 << (gpioToShiftGReg(pin) * 2));

			*(grf[0] + (offset >> 2)) = target;
		} else {
			offset += N1_GRF_PUPD_OFFSET;

			target |= *(grf[1] + (offset >> 2));
			target &= ~(1 << (gpioToShiftGReg(pin) * 2 + 1));
			target &= ~(1 << (gpioToShiftGReg(pin) * 2));

			*(grf[1] + (offset >> 2)) = target;
		}
		break;
	default:
		break;
	}

	setClkState(pin, N1_CLK_DISABLE);

	return 0;
}

/*----------------------------------------------------------------------------*/
static int _digitalRead (int pin)
{
	int bank, ret;
	char c;

	if (lib->mode == MODE_GPIO_SYS) {
		if (lib->sysFds[pin] == -1)
			return -1;

		lseek	(lib->sysFds[pin], 0L, SEEK_SET);
		if (read(lib->sysFds[pin], &c, 1) < 0) {
			msg(MSG_WARN, "%s: Failed with reading from sysfs GPIO node. \n", __func__);
			return -1;
		}

		return (c == '0') ? LOW : HIGH;
	}

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return	-1;

	bank = pin / 32;
	setClkState(pin, N1_CLK_ENABLE);

	ret = *(gpio[bank] + (N1_GPIO_GET_OFFSET >> 2)) & (1 << gpioToShiftReg(pin)) ? HIGH : LOW;

	setClkState(pin, N1_CLK_DISABLE);
	return ret;
}

/*----------------------------------------------------------------------------*/
static int _digitalWrite (int pin, int value)
{
	int bank;

	if (lib->mode == MODE_GPIO_SYS) {
		if (lib->sysFds[pin] != -1) {
			if (value == LOW) {
				if (write (lib->sysFds[pin], "0\n", 2) < 0)
					msg(MSG_ERR,
					"%s : %s\nEdit direction file to output mode for\n\t/sys/class/gpio/gpio%d/direction\n",
					__func__, strerror(errno), pin + N1_GPIO_PIN_BASE);
			} else {
				if (write (lib->sysFds[pin], "1\n", 2) < 0)
					msg(MSG_ERR,
					"%s : %s\nEdit direction file to output mode for\n\t/sys/class/gpio/gpio%d/direction\n",
					__func__, strerror(errno), pin + N1_GPIO_PIN_BASE);
			}
		}
		return -1;
	}

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	bank = pin / 32;
	setClkState(pin, N1_CLK_ENABLE);

	switch (value) {
	case LOW:
		*(gpio[bank] + (N1_GPIO_SET_OFFSET >> 2)) &= ~(1 << gpioToShiftReg(pin));
		break;
	case HIGH:
		*(gpio[bank] + (N1_GPIO_SET_OFFSET >> 2)) |=  (1 << gpioToShiftReg(pin));
		break;
	default:
		break;
	}

	setClkState(pin, N1_CLK_DISABLE);

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
	union	reg_bitfield	gpioBits1;

	if (lib->mode == MODE_GPIO_SYS) {
		return -1;
	}

	// Enable clock for GPIO 1 bank
	setClkState(32, N1_CLK_ENABLE);

	/* Read data register */
	gpioBits1.wvalue = *(gpio[1] + (N1_GPIO_GET_OFFSET >> 2));

	/* Wiring PI GPIO0 = N1 GPIO1_A.1 */
	gpioBits1.bits.bit1  = (value & 0x01);
	/* Wiring PI GPIO1 = N1 GPIO1_A.2 */
	gpioBits1.bits.bit2  = (value & 0x02);
	/* Wiring PI GPIO2 = N1 GPIO1_A.3 */
	gpioBits1.bits.bit3  = (value & 0x04);
	/* Wiring PI GPIO3 = N1 GPIO1_A.4 */
	gpioBits1.bits.bit4  = (value & 0x08);
	/* Wiring PI GPIO4 = N1 GPIO1_B.5 */
	gpioBits1.bits.bit14 = (value & 0x10);
	/* Wiring PI GPIO5 = N1 GPIO1_C.2 */
	gpioBits1.bits.bit19 = (value & 0x20);
	/* Wiring PI GPIO6 = N1 GPIO1_D.0 */
	gpioBits1.bits.bit25 = (value & 0x40);
	/* Wiring PI GPIO7 = N1 GPIO1_A.0 */
	gpioBits1.bits.bit0  = (value & 0x80);

	/* Update data register */
	*(gpio[1] + (N1_GPIO_SET_OFFSET >> 2)) = gpioBits1.wvalue;

	setClkState(32, N1_CLK_DISABLE);

	return 0;
}

/*----------------------------------------------------------------------------*/
static unsigned int _digitalReadByte (void)
{
	union reg_bitfield	gpioBits1;
	unsigned int		value = 0;

	if (lib->mode == MODE_GPIO_SYS) {
		return	-1;
	}

	// Enable clock for GPIO 1 bank
	setClkState(32, N1_CLK_ENABLE);

	/* Read data register */
	gpioBits1.wvalue = *(gpio[1] + (N1_GPIO_GET_OFFSET >> 2));

	setClkState(32, N1_CLK_DISABLE);

	/* Wiring PI GPIO0 = N1 GPIO1_A.1 */
	if (gpioBits1.bits.bit1)
		value |= 0x01;
	/* Wiring PI GPIO1 = N1 GPIO1_A.2 */
	if (gpioBits1.bits.bit2)
		value |= 0x02;
	/* Wiring PI GPIO2 = N1 GPIO1_A.3 */
	if (gpioBits1.bits.bit3)
		value |= 0x04;
	/* Wiring PI GPIO3 = N1 GPIO1_A.4 */
	if (gpioBits1.bits.bit4)
		value |= 0x08;
	/* Wiring PI GPIO4 = N1 GPIO1_B.5 */
	if (gpioBits1.bits.bit14)
		value |= 0x10;
	/* Wiring PI GPIO5 = N1 GPIO1_C.2 */
	if (gpioBits1.bits.bit19)
		value |= 0x20;
	/* Wiring PI GPIO6 = N1 GPIO1_D.0 */
	if (gpioBits1.bits.bit25)
		value |= 0x40;
	/* Wiring PI GPIO7 = N1 GPIO1_A.0 */
	if (gpioBits1.bits.bit0)
		value |= 0x80;

	return value;
}

/*----------------------------------------------------------------------------*/
static void init_gpio_mmap (void)
{
	int fd = -1;
	void *mapped_cru[2], *mapped_grf[2], *mapped_gpio[5];

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
		// GPIO{0, 1}
		//#define ODROIDN1_PMUCRU_BASE	0xFF750000
		mapped_cru[0] = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, N1_PMUCRU_BASE);

		// GPIO{2, 3, 4}
		//#define ODROIDN1_CRU_BASE	0xFF760000
		mapped_cru[1] = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, N1_CRU_BASE);

		// GPIO{0, 1}
		//#define ODROIDN1_PMU_BASE	0xFF320000
		mapped_grf[0] = mmap(0, N1_GRF_BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, N1_PMUGRF_BASE);

		// GPIO{2, 3, 4}
		//#define ODROIDN1_GRF_BASE	0xFF770000
		mapped_grf[1] = mmap(0, N1_GRF_BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, N1_GRF_BASE);

		// GPIO1_A.	0,1,2,3,4,7
		// GPIO1_B.	0,1,2,3,4,5
		// GPIO1_C.	2,4,5,6
		// GPIO1_D.	0
		//#define ODROIDN1_GPIO1_BASE	0xFF730000
		mapped_gpio[1] = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, N1_GPIO_1_BASE);

		// GPIO2_C.	0_B,1_B
		//#define ODROIDN1_GPIO2_BASE	0xFF780000
		mapped_gpio[2] = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, N1_GPIO_2_BASE);

		// GPIO4_C.	5,6
		// GPIO4_D.	0,4,5,6
		//#define ODROIDN1_GPIO4_BASE	0xFF790000
		mapped_gpio[4] = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, N1_GPIO_4_BASE);

		// Reserved
		mapped_gpio[0] = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, N1_GPIO_0_BASE);
		mapped_gpio[3] = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, N1_GPIO_3_BASE);

		if ((mapped_cru[0] == MAP_FAILED) || (mapped_cru[1] == MAP_FAILED)) {
			msg (MSG_ERR,
				"wiringPiSetup: mmap (CRU) failed: %s\n",
				strerror (errno));
		} else {
			cru[0] = (uint32_t *) mapped_cru[0];
			cru[1] = (uint32_t *) mapped_cru[1];
		}

		if ((mapped_grf[0] == MAP_FAILED) || (mapped_grf[1] == MAP_FAILED)) {
			msg (MSG_ERR,
				"wiringPiSetup: mmap (GRF) failed: %s\n",
				strerror (errno));
		} else {
			grf[0] = (uint32_t *) mapped_grf[0];
			grf[1] = (uint32_t *) mapped_grf[1];
		}

		if (	(mapped_gpio[0] == MAP_FAILED) ||
			(mapped_gpio[1] == MAP_FAILED) ||
			(mapped_gpio[2] == MAP_FAILED) ||
			(mapped_gpio[3] == MAP_FAILED) ||
			(mapped_gpio[4] == MAP_FAILED)) {
			msg (MSG_ERR,
				"wiringPiSetup: mmap (GPIO) failed: %s\n",
				strerror (errno));
		} else {
			gpio[0] = (uint32_t *) mapped_gpio[0];
			gpio[1] = (uint32_t *) mapped_gpio[1];
			gpio[2] = (uint32_t *) mapped_gpio[2];
			gpio[3] = (uint32_t *) mapped_gpio[3];
			gpio[4] = (uint32_t *) mapped_gpio[4];
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

	AIN0_NODE = "/sys/devices/platform/ff100000.saradc/iio:device0/in_voltage1_raw";
	AIN1_NODE = "/sys/devices/platform/ff100000.saradc/iio:device0/in_voltage0_raw";

	adcFds[0] = open(AIN0_NODE, O_RDONLY);
	adcFds[1] = open(AIN1_NODE, O_RDONLY);
}

/*----------------------------------------------------------------------------*/
void init_odroidn1 (struct libodroid *libwiring)
{
	init_gpio_mmap();

	init_adc_fds();

	/* wiringPi Core function initialize */
	libwiring->getModeToGpio	= _getModeToGpio;
	libwiring->pinMode		= _pinMode;
	libwiring->getAlt		= _getAlt;
	libwiring->pullUpDnControl	= _pullUpDnControl;
	libwiring->digitalRead		= _digitalRead;
	libwiring->digitalWrite		= _digitalWrite;
	libwiring->analogRead		= _analogRead;
	libwiring->digitalWriteByte	= _digitalWriteByte;
	libwiring->digitalReadByte	= _digitalReadByte;

	/* specify pin base number */
	libwiring->pinBase		= N1_GPIO_PIN_BASE;

	/* global variable setup */
	lib = libwiring;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
