// SPDX-License-Identifier: Apache-2.0

#include <zephyr/init.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <pinctrl_soc.h>

LOG_MODULE_REGISTER(board_rk3588, CONFIG_LOG_DEFAULT_LEVEL);


/* GPIO header pins we want available as plain GPIOs at boot */
static const pinctrl_soc_pin_t header_pins[] = {
	DT_PROP_BY_IDX(DT_NODELABEL(gpio1a4_as_gpio), pinmux, 0),
	DT_PROP_BY_IDX(DT_NODELABEL(gpio3a1_as_gpio), pinmux, 0),
	DT_PROP_BY_IDX(DT_NODELABEL(gpio1d6_as_gpio), pinmux, 0),
};

static int pi5_plus_header_pins_init(void)
{
	LOG_DBG("[pi5_plus_header_pins_init] Configuring GPIO header pins at boot (count=%zu)",
		ARRAY_SIZE(header_pins));
	printk("[board] pi5_plus_header_pins_init start (pins=%zu)\n", ARRAY_SIZE(header_pins));
	int ret = pinctrl_configure_pins(header_pins, ARRAY_SIZE(header_pins),
					 PINCTRL_REG_NONE);
	printk("[board] pi5_plus_header_pins_init done ret=%d\n", ret);
	if (ret < 0) {
		LOG_ERR("pinctrl_configure_pins failed (%d) - continuing without board mux", ret);
		return 0;
	}

	return 0;
}

/*
 * Require POST_KERNEL so that the pinctrl driver, clocks and resets have
 * finished initializing; invoking the routine in PRE_KERNEL_1 was racing with
 * those subsystems and caused the boot hang.
 */
SYS_INIT(pi5_plus_header_pins_init, POST_KERNEL, 50);
