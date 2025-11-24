
/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control/rk3588_cru.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);


static const struct gpio_dt_spec user_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

#define BLINK_PERIOD_MS     1000U

int main(void)
{
	if (!device_is_ready(user_led.port)) {
		LOG_ERR("LED GPIO controller not ready");
		return 0;
	}

	int ret = gpio_pin_configure_dt(&user_led, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		LOG_ERR("Failed to configure user LED (err=%d)", ret);
		return 0;
	}

	LOG_INF("Blink su LED0 (header pin 11) ogni %u ms",
		BLINK_PERIOD_MS);

	rk3588_cru_dump_state();

	while (true) {
		gpio_pin_set_dt(&user_led, 1);
		k_msleep(BLINK_PERIOD_MS);
		gpio_pin_set_dt(&user_led, 0);
		k_msleep(BLINK_PERIOD_MS);
	}
}
