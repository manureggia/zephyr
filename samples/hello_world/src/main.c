
/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>

#define MY_PINCTRL_NODE DT_NODELABEL(my_pinctrl)
PINCTRL_DT_DEFINE(MY_PINCTRL_NODE);
static const struct pinctrl_dev_config *pcfg =
        PINCTRL_DT_DEV_CONFIG_GET(MY_PINCTRL_NODE);

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/* 40-pin header mapping provided: pin 11 = GPIO1_A4, pin 12 = GPIO3_A1 */
#define GPIO1_NODE     DT_NODELABEL(gpio1)
#define GPIO3_NODE     DT_NODELABEL(gpio3)
#define TEST_PIN_OUT   4   /* GPIO1_A4 */
#define TEST_PIN_IN    1   /* GPIO3_A1 */

static const struct device *gpio1_dev;
static const struct device *gpio3_dev;

int main(void)
{
    int ret = pinctrl_apply_state(pcfg, PINCTRL_STATE_DEFAULT);
    LOG_INF("pinctrl_apply_state -> %d", ret);
    
    gpio1_dev = DEVICE_DT_GET(GPIO1_NODE);
    if (!device_is_ready(gpio1_dev)) {
        LOG_ERR("GPIO1 device not ready");
        return 0;
    }

    gpio3_dev = DEVICE_DT_GET(GPIO3_NODE);
    if (!device_is_ready(gpio3_dev)) {
        LOG_ERR("GPIO3 device not ready");
        return 0;
    }

    /* Configura pin di test: uno in uscita, uno in ingresso */
    ret = gpio_pin_configure(gpio1_dev, TEST_PIN_OUT, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure OUT pin %d", TEST_PIN_OUT);
        return 0;
    }

    ret = gpio_pin_configure(gpio3_dev, TEST_PIN_IN, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure IN pin %d", TEST_PIN_IN);
        return 0;
    }

    LOG_INF("Loopback: GPIO1 pin %d -> GPIO3 pin %d", TEST_PIN_OUT, TEST_PIN_IN);

    while (1) {
        gpio_pin_toggle(gpio1_dev, TEST_PIN_OUT);
        int out_val = gpio_pin_get(gpio1_dev, TEST_PIN_OUT);
        int in_val  = gpio_pin_get(gpio3_dev, TEST_PIN_IN);
        LOG_INF("OUT=%d IN=%d", out_val, in_val);
        k_msleep(500);
    }

    return 0;
}
