/*----------------------------------------------------------------------------*/
/*

	WiringPi ODROID-N2 Board Header file

 */
/*----------------------------------------------------------------------------*/
#ifndef	__ODROID_N2_H__
#define	__ODROID_N2_H__

/*----------------------------------------------------------------------------*/
#define N2_GPIO_MASK			(0xFFFFFF00)
#define N2_GPIO_BASE			0xff634000
#define N2_GPIO_PWM_BASE		0xFFD19000

#define N2_GPIO_PIN_BASE		410

#define N2_GPIOA_PIN_START		(N2_GPIO_PIN_BASE + 50) // GPIOA_0
#define N2_GPIOA_PIN_END		(N2_GPIO_PIN_BASE + 65) // GPIOA_15
#define N2_GPIOX_PIN_START		(N2_GPIO_PIN_BASE + 66) // GPIOX_0
#define N2_GPIOX_PIN_MID		(N2_GPIO_PIN_BASE + 81) // GPIOX_15
#define N2_GPIOX_PIN_END		(N2_GPIO_PIN_BASE + 85) // GPIOX_19

#define N2_GPIOX_FSEL_REG_OFFSET	0x116
#define N2_GPIOX_OUTP_REG_OFFSET	0x117
#define N2_GPIOX_INP_REG_OFFSET		0x118
#define N2_GPIOX_PUPD_REG_OFFSET	0x13C
#define N2_GPIOX_PUEN_REG_OFFSET	0x14A
#define N2_GPIOX_DS_REG_2A_OFFSET	0x1D2
#define N2_GPIOX_DS_REG_2B_OFFSET	0x1D3
#define N2_GPIOX_MUX_3_REG_OFFSET	0x1B3
#define N2_GPIOX_MUX_4_REG_OFFSET	0x1B4
#define N2_GPIOX_MUX_5_REG_OFFSET	0x1B5

#define N2_GPIOA_FSEL_REG_OFFSET	0x120
#define N2_GPIOA_OUTP_REG_OFFSET	0x121
#define N2_GPIOA_INP_REG_OFFSET		0x122
#define N2_GPIOA_PUPD_REG_OFFSET	0x13F
#define N2_GPIOA_PUEN_REG_OFFSET	0x14D
#define N2_GPIOA_DS_REG_5A_OFFSET	0x1D6
#define N2_GPIOA_MUX_D_REG_OFFSET	0x1BD
#define N2_GPIOA_MUX_E_REG_OFFSET	0x1BE

/// S922X datasheet p.1075
#define N2_PWM_CD_OFFSET		0x1000
#define N2_PWM_EF_OFFSET		0
#define N2_PWM_0_DUTY_CYCLE_OFFSET	0x00
#define N2_PWM_1_DUTY_CYCLE_OFFSET	0x01
#define N2_PWM_MISC_REG_01_OFFSET	0x02

/// PWM_MISC_REG_CD
#define N2_PWM_1_INV_EN		( 27 )
#define N2_PWM_0_INV_EN		( 26 )
#define N2_PWM_1_CLK_EN		( 23 )
#define N2_PWM_1_CLK_DIV0	( 16 )	/// 22 ~ 16
#define N2_PWM_0_CLK_EN		( 15 )
#define N2_PWM_0_CLK_DIV0	( 8 )	/// 14 ~ 8
#define N2_PWM_1_CLK_SEL0	( 6 )	/// 7 ~ 6
#define N2_PWM_0_CLK_SEL0	( 4 )	/// 5 ~ 4
#define N2_PWM_1_DS_EN		( 3 )
#define N2_PWM_0_DS_EN		( 2 )
#define N2_PWM_1_EN		( 1 )
#define N2_PWM_0_EN		( 0 )

#ifdef __cplusplus
extern "C" {
#endif

extern void init_odroidn2 (struct libodroid *libwiring);

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------*/
#endif	/* __ODROID_N2_H__ */
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
