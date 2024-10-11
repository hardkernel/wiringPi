/*----------------------------------------------------------------------------*/
/*

	WiringPi ODROID-C5 Board Header file

 */
/*----------------------------------------------------------------------------*/
#ifndef	__ODROID_C5_H__
#define	__ODROID_C5_H__

/*----------------------------------------------------------------------------*/

#define C5_PWM_INTERNAL_CLK			24000000

#define C5_GPIO_BASE			0xfe004000

#define C5_GPIO_D_WPI_START  452
#define C5_GPIO_DV_WPI_START 457
#define C5_GPIO_H_WPI_START  464
#define C5_GPIO_X_WPI_START  476

#define C5_GPIO_D(REG)  (REG + C5_GPIO_D_WPI_START)
#define C5_GPIO_DV(REG) (REG + C5_GPIO_DV_WPI_START)
#define C5_GPIO_H(REG)  (REG + C5_GPIO_H_WPI_START)
#define C5_GPIO_X(REG)  (REG + C5_GPIO_X_WPI_START)

#define C5_IS_GPIO_D(W)      (W >= C5_GPIO_D(2) && W <= C5_GPIO_D(4))
#define C5_IS_GPIO_DV(W)     (W >= C5_GPIO_DV(1) && W <= C5_GPIO_DV(2))
#define C5_IS_GPIO_H(W)      (W >= C5_GPIO_H(4) && W <= C5_GPIO_H(5))
#define C5_IS_GPIO_X(W)      (W >= C5_GPIO_X(0) && W <= C5_GPIO_X(18) && W != C5_GPIO_X(16))
#define C5_IS_GPIO_X_EXT(W)  (W >= C5_GPIO_X(16) && W <= C5_GPIO_X(18))

#define C5_PIN_MUX_REG2_OFFSET 0x02
#define C5_PIN_MUX_REG3_OFFSET 0x03
#define C5_PIN_MUX_REG4_OFFSET 0x04
#define C5_PIN_MUX_REG5_OFFSET 0x05
#define C5_PIN_MUX_REGB_OFFSET 0x0b
#define C5_PIN_MUX_REGG_OFFSET 0x10

#define C5_GPIO_X_OFFSET  0x40
#define C5_GPIO_H_OFFSET  0x50
#define C5_GPIO_D_OFFSET  0x60
#define C5_GPIO_DV_OFFSET 0xa0

#define C5_GPIO_I_OFFSET 		0x00
#define C5_GPIO_O_OFFSET 		0x01
#define C5_GPIO_OEN_OFFSET 		0x02
#define C5_GPIO_PULL_EN_OFFSET 	0x03
#define C5_GPIO_PULL_UP_OFFSET 	0x04
#define C5_GPIO_DS_OFFSET 		0x07
#define C5_GPIO_DS_EXT_OFFSET 	0x08



#ifdef __cplusplus
extern "C" {
#endif

extern void init_odroidc5 (struct libodroid *libwiring);

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------*/
#endif	/* __ODROID_C5_H__ */
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
