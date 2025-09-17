/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_DT_BINDINGS_PINCTRL_RK3588_H_
#define ZEPHYR_DT_BINDINGS_PINCTRL_RK3588_H_

/* Ports */
#define RK3588_PORT_A 0
#define RK3588_PORT_B 1
#define RK3588_PORT_C 2
#define RK3588_PORT_D 3

/* Functions (we start with only GPIO = 0) */
#define RK3588_FUNC_GPIO 0

/* pinmux encoding:
 * [31:28] bank (0..4)
 * [27:24] port (A=0..D=3)
 * [23:20] pin  (0..7)
 * [19:16] func (0=GPIO, others TBA)
 * [15:0 ] reserved
 */
#define RK3588_PINMUX(bank, port, pin, func) \
  ((((bank) & 0xF) << 28) | (((port) & 0xF) << 24) | (((pin) & 0xF) << 20) | (((func) & 0xF) << 16))

/* helpers to extract fields */
#define RK3588_PINMUX_BANK(x) (((x) >> 28) & 0xF)
#define RK3588_PINMUX_PORT(x) (((x) >> 24) & 0xF)
#define RK3588_PINMUX_PIN(x)  (((x) >> 20) & 0xF)
#define RK3588_PINMUX_FUNC(x) (((x) >> 16) & 0xF)

#endif /* ZEPHYR_DT_BINDINGS_PINCTRL_RK3588_H_ */