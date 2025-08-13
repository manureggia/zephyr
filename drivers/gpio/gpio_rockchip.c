/* SPDX-License-Identifier: Apache-2.0 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>

/* Compat del DT per questo driver */
#define DT_DRV_COMPAT rockchip_rk3588_gpio

/* ---- Config & Data ------------------------------------------------------ */

struct rk_gpio_config {
	/* Base regs e IRQ dal DT (stub per ora) */
	uintptr_t regs;
	int irq;
};

struct rk_gpio_data {
	sys_slist_t callbacks;
	uint32_t out_shadow;
	uint32_t dir_shadow;
};

/* ---- Prototipi funzioni API -------------------------------------------- */

static int  rk_gpio_init(const struct device *dev);
static int  rk_gpio_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags);
static int  rk_gpio_port_get_raw(const struct device *dev, uint32_t *value);
static int  rk_gpio_port_set_masked_raw(const struct device *dev, uint32_t mask, uint32_t value);
static int  rk_gpio_port_set_bits_raw(const struct device *dev, uint32_t pins);
static int  rk_gpio_port_clear_bits_raw(const struct device *dev, uint32_t pins);
static int  rk_gpio_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
					    enum gpio_int_mode mode, enum gpio_int_trig trig);
static int  rk_gpio_manage_callback(const struct device *dev, struct gpio_callback *cb, bool set);

/* ---- API structure ------------------------------------------------------ */

static const struct gpio_driver_api rk_gpio_api = {
	.pin_configure = rk_gpio_pin_configure,
	.port_get_raw = rk_gpio_port_get_raw,
	.port_set_masked_raw = rk_gpio_port_set_masked_raw,
	.port_set_bits_raw = rk_gpio_port_set_bits_raw,
	.port_clear_bits_raw = rk_gpio_port_clear_bits_raw,
	.pin_interrupt_configure = rk_gpio_pin_interrupt_configure,
	.manage_callback = rk_gpio_manage_callback,
};

/* ---- Init --------------------------------------------------------------- */

static int rk_gpio_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	/* Quando avrai l’IRQ:
	 * const struct rk_gpio_config *cfg = dev->config;
	 * if (cfg->irq >= 0) { irq_connect_dynamic(...); irq_enable(cfg->irq); }
	 */
	return 0;
}

/* ---- Callbacks helpers -------------------------------------------------- */

static int rk_gpio_manage_callback(const struct device *dev, struct gpio_callback *cb, bool set)
{
	struct rk_gpio_data *d = dev->data;
	return gpio_manage_callback(&d->callbacks, cb, set);
}

/* ---- Funzioni base (stub: compila, poi metti i registri reali) ---------- */

static int rk_gpio_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(pin);
	ARG_UNUSED(flags);
	/* TODO: programma direzione/pull/drive su HW */
	return 0;
}

static int rk_gpio_port_get_raw(const struct device *dev, uint32_t *value)
{
	const struct rk_gpio_data *d = dev->data;
	/* TODO: leggi HW; per ora shadow */
	*value = d->out_shadow;
	return 0;
}

static int rk_gpio_port_set_masked_raw(const struct device *dev, uint32_t mask, uint32_t value)
{
	struct rk_gpio_data *d = dev->data;
	/* TODO: scrivi HW; per ora shadow */
	d->out_shadow = (d->out_shadow & ~mask) | (value & mask);
	return 0;
}

static int rk_gpio_port_set_bits_raw(const struct device *dev, uint32_t pins)
{
	struct rk_gpio_data *d = dev->data;
	d->out_shadow |= pins;
	return 0;
}

static int rk_gpio_port_clear_bits_raw(const struct device *dev, uint32_t pins)
{
	struct rk_gpio_data *d = dev->data;
	d->out_shadow &= ~pins;
	return 0;
}

static int rk_gpio_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
					   enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(pin);
	ARG_UNUSED(mode);
	ARG_UNUSED(trig);
	/* TODO: configura registri di interrupt */
	return 0;
}

/* ---- Instanziazione da Devicetree (senza PINCTRL) ---------------------- */

#define RK_GPIO_INIT(inst)                                                       \
	static const struct rk_gpio_config rk_gpio_cfg_##inst = {                \
		.regs = DT_REG_ADDR(DT_DRV_INST(inst)),                           \
		.irq  = COND_CODE_1(DT_IRQ_HAS_IDX(DT_DRV_INST(inst), 0),         \
		                    (DT_IRQN(DT_DRV_INST(inst))), (-1)),        \
	};                                                                        \
	static struct rk_gpio_data rk_gpio_data_##inst;                           \
	DEVICE_DT_INST_DEFINE(inst,                                               \
			      rk_gpio_init,                                         \
			      NULL,                                                  \
			      &rk_gpio_data_##inst,                                  \
			      &rk_gpio_cfg_##inst,                                   \
			      POST_KERNEL,                                           \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                    \
			      &rk_gpio_api);

DT_INST_FOREACH_STATUS_OKAY(RK_GPIO_INIT)