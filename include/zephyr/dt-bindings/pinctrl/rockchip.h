/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_DT_BINDINGS_PINCTRL_RK3588_H_
#define ZEPHYR_DT_BINDINGS_PINCTRL_RK3588_H_

/* Porta: A=0, B=1, C=2, D=3  (8 pin per porta -> 32 pin per bank) */
#define RK_PA 0
#define RK_PB 1
#define RK_PC 2
#define RK_PD 3

/* Flags generici (bit di config pin-conf) */
#define RK_PIN_PULL_MASK      GENMASK(1,0)
#define RK_PIN_PULL_NONE      0
#define RK_PIN_PULL_UP        1
#define RK_PIN_PULL_DOWN      2

#define RK_PIN_SCHMITT        BIT(2)      /* abilita schmitt trigger */

#define RK_PIN_DRV_SHIFT      3           /* 3..5: drive level 0..7 (mappa TRM) */
#define RK_PIN_DRV(level)     (((level) & 0x7) << RK_PIN_DRV_SHIFT)

/*
 * Codifica pin per Zephyr pinctrl (32-bit):
 * [31:28] bank (0..4)
 * [27:26] port (A..D)
 * [25:23] index (0..7)
 * [22:19] function (0..15)
 * [18:0]  flags (pull/schmitt/drive)
 */
#define RK_PINMUX(bank, port, index, func, flags) \
	(((bank)  & 0xF)  << 28 | \
	 ((port)  & 0x3)  << 26 | \
	 ((index) & 0x7)  << 23 | \
	 ((func)  & 0xF)  << 19 | \
	 ((flags) & 0x7FFFF))

#define RK_PIN_BANK(val)   (((val) >> 28) & 0xF)
#define RK_PIN_PORT(val)   (((val) >> 26) & 0x3)
#define RK_PIN_INDEX(val)  (((val) >> 23) & 0x7)
#define RK_PIN_FUNC(val)   (((val) >> 19) & 0xF)
#define RK_PIN_FLAGS(val)  ((val) & 0x7FFFF)

#endif /* ZEPHYR_DT_BINDINGS_PINCTRL_RK3588_H_ */

