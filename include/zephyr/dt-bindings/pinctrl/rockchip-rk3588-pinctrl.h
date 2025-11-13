/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_DT_BINDINGS_PINCTRL_RK3588_H_
#define ZEPHYR_DT_BINDINGS_PINCTRL_RK3588_H_

/* Ports */
#define RK3588_PORT_A 0
#define RK3588_PORT_B 1
#define RK3588_PORT_C 2
#define RK3588_PORT_D 3

/* Functions */
#define RK3588_FUNC_GPIO 0

/* pinmux encoding:
 * [31:28] bank (0..4)
 * [27:24] port (A=0..D=3)
 * [23:20] pin  (0..7)
 * [19:16] func (0=GPIO, others per SoC mux)
 * [15:0 ] per-pin electrical configuration flags (pull/drive/schmitt/..)
 */
#define RK3588_PINMUX(bank, port, pin, func) \
	((((bank) & 0xF) << 28) | (((port) & 0xF) << 24) | (((pin) & 0xF) << 20) | (((func) & 0xF) << 16))

/* Electrical configuration flag bitmap */
#define RK3588_PINCFG_FLAGS_MASK	0xFFFFU

/* Pull select */
#define RK3588_PINCFG_PULL_SHIFT	0
#define RK3588_PINCFG_PULL_MASK		0x3U
#define RK3588_PINCFG_PULL_NONE		(0U << RK3588_PINCFG_PULL_SHIFT)
#define RK3588_PINCFG_PULL_UP		(1U << RK3588_PINCFG_PULL_SHIFT)
#define RK3588_PINCFG_PULL_DOWN		(2U << RK3588_PINCFG_PULL_SHIFT)
#define RK3588_PINCFG_PULL_KEEP		(3U << RK3588_PINCFG_PULL_SHIFT)

/* Drive strength (4-bit value, valid flag in bit 6) */
#define RK3588_PINCFG_DRIVE_SHIFT	2
#define RK3588_PINCFG_DRIVE_MASK	(0xFU << RK3588_PINCFG_DRIVE_SHIFT)
#define RK3588_PINCFG_DRIVE_FLAG	(1U << 6)
#define RK3588_PINCFG_DRIVE(val)	((((val) & 0xFU) << RK3588_PINCFG_DRIVE_SHIFT) | RK3588_PINCFG_DRIVE_FLAG)

/* Schmitt trigger configuration (2-bit selector starting at bit 7) */
#define RK3588_PINCFG_SCHMITT_SHIFT	7
#define RK3588_PINCFG_SCHMITT_MASK	(0x3U << RK3588_PINCFG_SCHMITT_SHIFT)
#define RK3588_PINCFG_SCHMITT_KEEP	(0U << RK3588_PINCFG_SCHMITT_SHIFT)
#define RK3588_PINCFG_SCHMITT_DISABLE	(1U << RK3588_PINCFG_SCHMITT_SHIFT)
#define RK3588_PINCFG_SCHMITT_ENABLE	(2U << RK3588_PINCFG_SCHMITT_SHIFT)

/* Helper macro to compose mux + electrical flags */
#define RK3588_PINCFG(bank, port, pin, func, flags) \
	(RK3588_PINMUX(bank, port, pin, func) | ((flags) & RK3588_PINCFG_FLAGS_MASK))

/* helpers to extract fields */
#define RK3588_PINMUX_BANK(x) (((x) >> 28) & 0xF)
#define RK3588_PINMUX_PORT(x) (((x) >> 24) & 0xF)
#define RK3588_PINMUX_PIN(x)  (((x) >> 20) & 0xF)
#define RK3588_PINMUX_FUNC(x) (((x) >> 16) & 0xF)
#define RK3588_PINMUX_FLAGS(x) ((x) & RK3588_PINCFG_FLAGS_MASK)

#endif /* ZEPHYR_DT_BINDINGS_PINCTRL_RK3588_H_ */
