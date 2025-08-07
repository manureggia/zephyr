/*
 * Copyright (c) 2025 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT rockchip_rk3588_gpio

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/irq.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/logging/log.h>
#include <zephyr/dt-bindings/clock/rk3588-cru.h>

LOG_MODULE_REGISTER(gpio_rockchip, CONFIG_GPIO_LOG_LEVEL);

#define GPIO_SWPORT_DR          0x0000
#define GPIO_SWPORT_DDR         0x0004

struct gpio_rockchip_config {
	struct gpio_driver_config common;
	uintptr_t reg_base;
	const struct device *clock_dev;
	uint32_t pclk_id;
	uint32_t dbclk_id;
	const struct reset_dt_spec reset;
	void (*irq_config_func)(const struct device *dev);
};

struct gpio_rockchip_data {
	struct gpio_driver_data common;
	sys_slist_t callbacks;
};

static int gpio_rockchip_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_rockchip_config *config = dev->config;

	if ((flags & (GPIO_PULL_UP | GPIO_PULL_DOWN | GPIO_OPEN_DRAIN)) != 0) {
		return -ENOTSUP;
	}

	if ((flags & GPIO_OUTPUT) != 0) {
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			sys_write32(sys_read32(config->reg_base + GPIO_SWPORT_DR) | BIT(pin),
				    config->reg_base + GPIO_SWPORT_DR);
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
			sys_write32(sys_read32(config->reg_base + GPIO_SWPORT_DR) & ~BIT(pin),
				    config->reg_base + GPIO_SWPORT_DR);
		}
		sys_write32(sys_read32(config->reg_base + GPIO_SWPORT_DDR) | BIT(pin),
			    config->reg_base + GPIO_SWPORT_DDR);
	} else { /* Input */
		sys_write32(sys_read32(config->reg_base + GPIO_SWPORT_DDR) & ~BIT(pin),
			    config->reg_base + GPIO_SWPORT_DDR);
	}

	return 0;
}

static int gpio_rockchip_manage_callback(const struct device *dev,
					 struct gpio_callback *callback, bool set)
{
	struct gpio_rockchip_data *data = dev->data;
	return gpio_manage_callback(&data->callbacks, callback, set);
}

static void gpio_rockchip_isr(const struct device *dev) { /* Da implementare */ }

static const struct gpio_driver_api gpio_rockchip_api = {
	.pin_configure = gpio_rockchip_pin_configure,
	.manage_callback = gpio_rockchip_manage_callback,
};

static int gpio_rockchip_init(const struct device *dev)
{
	const struct gpio_rockchip_config *config = dev->config;

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("Clock controller per %s non pronto", dev->name);
		return -ENOTSUP;
	}

	clock_control_on(config->clock_dev, (clock_control_subsys_t)config->pclk_id);
	clock_control_on(config->clock_dev, (clock_control_subsys_t)config->dbclk_id);

	if (config->reset.dev) {
		if (!device_is_ready(config->reset.dev)) {
			LOG_ERR("Reset controller non pronto");
			return -ENODEV;
		}
		reset_line_toggle_dt(&config->reset);
	}

	if (config->irq_config_func) {
		config->irq_config_func(dev);
	}

	LOG_INF("Driver GPIO %s inizializzato", dev->name);
	return 0;
}

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio0), okay)
static void gpio_rockchip_irq_config_0(const struct device *dev)
{
	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(gpio0)), DT_IRQ(DT_NODELABEL(gpio0), priority),
		    gpio_rockchip_isr, DEVICE_DT_GET(DT_NODELABEL(gpio0)), 0);
	irq_enable(DT_IRQN(DT_NODELABEL(gpio0)));
}
static const struct gpio_rockchip_config gpio_config_0 = {
	.common = {.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_NODE(DT_NODELABEL(gpio0))},
	.reg_base = DT_REG_ADDR(DT_NODELABEL(gpio0)),
	.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_NODELABEL(gpio0))),
	.pclk_id = DT_CLOCKS_CELL_BY_IDX(DT_NODELABEL(gpio0), 0, id),
	.dbclk_id = DT_CLOCKS_CELL_BY_IDX(DT_NODELABEL(gpio0), 1, id),
	.reset = RESET_DT_SPEC_GET_OR(DT_NODELABEL(gpio0), {0}),
	.irq_config_func = gpio_rockchip_irq_config_0,
};
static struct gpio_rockchip_data gpio_data_0;
DEVICE_DT_DEFINE(DT_NODELABEL(gpio0), gpio_rockchip_init, NULL,
		 &gpio_data_0, &gpio_config_0,
		 POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY,
		 &gpio_rockchip_api);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio1), okay)
static void gpio_rockchip_irq_config_1(const struct device *dev)
{
	IRQ_CONNECT(DT_IRQN(DT_NODELABEL(gpio1)), DT_IRQ(DT_NODELABEL(gpio1), priority),
		    gpio_rockchip_isr, DEVICE_DT_GET(DT_NODELABEL(gpio1)), 0);
	irq_enable(DT_IRQN(DT_NODELABEL(gpio1)));
}
static const struct gpio_rockchip_config gpio_config_1 = {
	.common = {.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_NODE(DT_NODELABEL(gpio1))},
	.reg_base = DT_REG_ADDR(DT_NODELABEL(gpio1)),
	.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_NODELABEL(gpio1))),
	.pclk_id = DT_CLOCKS_CELL_BY_IDX(DT_NODELABEL(gpio1), 0, id),
	.dbclk_id = DT_CLOCKS_CELL_BY_IDX(DT_NODELABEL(gpio1), 1, id),
	.reset = RESET_DT_SPEC_GET_OR(DT_NODELABEL(gpio1), {0}),
	.irq_config_func = gpio_rockchip_irq_config_1,
};
static struct gpio_rockchip_data gpio_data_1;
DEVICE_DT_DEFINE(DT_NODELABEL(gpio1), gpio_rockchip_init, NULL,
		 &gpio_data_1, &gpio_config_1,
		 POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY,
		 &gpio_rockchip_api);
#endif

/* Aggiungere blocchi simili per gpio2, gpio3, gpio4 se necessario */
