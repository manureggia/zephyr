/* SPDX-License-Identifier: Apache-2.0 */
/* Zephyr 4.2.0-rc2 - Rockchip RK3588 GPIO (DW APB GPIO-like) */
#define DT_DRV_COMPAT rockchip_rk3588_gpio

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree/clocks.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/rockchip_clock_control.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/mem_manage.h>
#include <zephyr/kernel/internal/mm.h>
#include <zephyr/irq.h>

LOG_MODULE_REGISTER(gpio_rk3588, CONFIG_GPIO_LOG_LEVEL);

/* Attributi MMU per periferiche su ARM64: Device-nGnRE (come DEVICE_MMIO_MAP) */
#ifndef RK_MMIO_FLAGS_DEVICE
#define RK_MMIO_FLAGS_DEVICE 0xA
#endif

/* --- Register map (DW APB GPIO single-port) --- */
/* Riferimenti: Rockchip TRM (famiglia), DW APB GPIO databook */
#define RK_GPIO_SWPORTA_DR        0x00
#define RK_GPIO_SWPORTA_DDR       0x04
#define RK_GPIO_INTEN             0x30
#define RK_GPIO_INTMASK           0x34
#define RK_GPIO_INTTYPE_LEVEL     0x38
#define RK_GPIO_INT_POLARITY      0x3C
#define RK_GPIO_INT_STATUS        0x40
#define RK_GPIO_RAW_INT_STATUS    0x44
#define RK_GPIO_DEBOUNCE          0x48
#define RK_GPIO_PORTS_EOI         0x4C
#define RK_GPIO_EXT_PORTA         0x50
#define RK_GPIO_LS_SYNC           0x60

struct rk_gpio_cfg {
    struct gpio_driver_config common; /* MUST be first for Zephyr GPIO API */
    uintptr_t mmio_phys;              /* base fisico dal DT */
    size_t    mmio_size;              /* dimensione finestra MMIO */
    uint8_t   bank_idx;               /* 0..4 */
    /* TODO: clock handles quando avremo driver CRU */
    int       irq;                    /* opzionale: linea GIC */
    const char *label;
    /* CRU clocks/resets (opzionali) */
    const struct device *clk_dev;
    struct rockchip_clk_subsys pclk;
    struct rockchip_clk_subsys dbclk;
};

struct rk_gpio_data {
	struct gpio_driver_data common;   /* MUST be first for Zephyr GPIO API */
	struct k_spinlock lock;
	sys_slist_t callbacks;
	mm_reg_t mmio_base;               /* base virtuale mappata con k_mem_map_phys_bare */
	/* Shadow del registro DR per piattaforme senza HW effettivo o letture non riflettenti */
	uint32_t dr_shadow;
	/* Shadow della direzione (DDR): 1=output, 0=input */
	uint32_t ddr_shadow;
};

static inline uint32_t reg_read(const struct device *dev, uint32_t off)
{
	mm_reg_t base = ((const struct rk_gpio_data *)dev->data)->mmio_base;
	return sys_read32(base + off);
}

static inline void reg_write(const struct device *dev, uint32_t off, uint32_t v)
{
	mm_reg_t base = ((const struct rk_gpio_data *)dev->data)->mmio_base;
	sys_write32(v, base + off);
}

/* --- Debug helper: dump registri critici del bank --- */
static void rk_gpio_dump_regs(const struct device *dev, const char *tag)
{
	const struct rk_gpio_cfg *cfg = dev->config;
	uint32_t dr = reg_read(dev, RK_GPIO_SWPORTA_DR);
	uint32_t ddr = reg_read(dev, RK_GPIO_SWPORTA_DDR);
	uint32_t ext = reg_read(dev, RK_GPIO_EXT_PORTA);
	uint32_t ien = reg_read(dev, RK_GPIO_INTEN);
	uint32_t imask = reg_read(dev, RK_GPIO_INTMASK);
	uint32_t itype = reg_read(dev, RK_GPIO_INTTYPE_LEVEL);
	uint32_t ipol = reg_read(dev, RK_GPIO_INT_POLARITY);
	uint32_t ist = reg_read(dev, RK_GPIO_INT_STATUS);

	LOG_DBG("[%s] %s: DR=0x%08x DDR=0x%08x EXT=0x%08x",
		cfg->label, tag, dr, ddr, ext);
	LOG_DBG("[%s] %s: INTEN=0x%08x MASK=0x%08x TYPE=0x%08x POL=0x%08x IST=0x%08x",
		cfg->label, tag, ien, imask, itype, ipol, ist);
}

/* --- API Zephyr: port get/set --- */
static int rk_gpio_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	const struct rk_gpio_cfg *cfg = dev->config;
	/* Per i pin configurati in output, riportiamo lo stato dell'output latch
	 * (dr_shadow). Per gli input, leggiamo EXT.
	 */
	uint32_t ext = reg_read(dev, RK_GPIO_EXT_PORTA);
	uint32_t ddr = ((const struct rk_gpio_data *)dev->data)->ddr_shadow;
	uint32_t drs = ((const struct rk_gpio_data *)dev->data)->dr_shadow;
	uint32_t out = (ext & ~ddr) | (drs & ddr);
	*value = out;
	LOG_DBG("[%s] port_get_raw -> 0x%08x", cfg->label, (uint32_t)out);
	return 0;
}

static int rk_gpio_port_set_masked_raw(const struct device *dev,
				       gpio_port_pins_t mask,
				       gpio_port_value_t value)
{
	const struct rk_gpio_cfg *cfg = dev->config;
	k_spinlock_key_t key = k_spin_lock(&((struct rk_gpio_data *)dev->data)->lock);

	uint32_t dr = reg_read(dev, RK_GPIO_SWPORTA_DR);
	dr = (dr & ~mask) | (value & mask);
	reg_write(dev, RK_GPIO_SWPORTA_DR, dr);
	/* Aggiorna shadow per riflettere lo stato voluto */
	((struct rk_gpio_data *)dev->data)->dr_shadow =
		((((struct rk_gpio_data *)dev->data)->dr_shadow & ~mask) | (value & mask));

	k_spin_unlock(&((struct rk_gpio_data *)dev->data)->lock, key);
	uint32_t ext = reg_read(dev, RK_GPIO_EXT_PORTA);
	LOG_DBG("[%s] port_set_masked_raw: mask=0x%08x value=0x%08x -> DR=0x%08x EXT=0x%08x",
		cfg->label, (uint32_t)mask, (uint32_t)value, dr, ext);
	return 0;
}

static int rk_gpio_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	return rk_gpio_port_set_masked_raw(dev, pins, pins);
}

static int rk_gpio_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	return rk_gpio_port_set_masked_raw(dev, pins, 0);
}

static int rk_gpio_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	const struct rk_gpio_cfg *cfg = dev->config;
	k_spinlock_key_t key = k_spin_lock(&((struct rk_gpio_data *)dev->data)->lock);
	uint32_t dr = reg_read(dev, RK_GPIO_SWPORTA_DR);
	dr ^= pins;
	reg_write(dev, RK_GPIO_SWPORTA_DR, dr);
	/* Aggiorna shadow */
	((struct rk_gpio_data *)dev->data)->dr_shadow ^= pins;
	k_spin_unlock(&((struct rk_gpio_data *)dev->data)->lock, key);
	uint32_t ext = reg_read(dev, RK_GPIO_EXT_PORTA);
	LOG_DBG("[%s] toggle_bits: pins=0x%08x -> DR=0x%08x EXT=0x%08x", cfg->label, (uint32_t)pins, dr, ext);
	return 0;
}

/* --- API Zephyr: configure pin --- */
static int rk_gpio_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct rk_gpio_cfg *cfg = dev->config;

	if (pin >= 32) {
		return -EINVAL;
	}

	/* Direction */
	k_spinlock_key_t key = k_spin_lock(&((struct rk_gpio_data *)dev->data)->lock);
	uint32_t ddr = reg_read(dev, RK_GPIO_SWPORTA_DDR);

	if (flags & GPIO_OUTPUT) {
		ddr |= BIT(pin);
		/* output init value */
		uint32_t dr = reg_read(dev, RK_GPIO_SWPORTA_DR);
		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			dr |= BIT(pin);
		} else if (flags & GPIO_OUTPUT_INIT_LOW) {
			dr &= ~BIT(pin);
		}
		reg_write(dev, RK_GPIO_SWPORTA_DR, dr);
		/* Aggiorna shadow in base all'inizializzazione */
		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			((struct rk_gpio_data *)dev->data)->dr_shadow |= BIT(pin);
		} else if (flags & GPIO_OUTPUT_INIT_LOW) {
			((struct rk_gpio_data *)dev->data)->dr_shadow &= ~BIT(pin);
		}
	} else { /* input */
		ddr &= ~BIT(pin);
	}

	reg_write(dev, RK_GPIO_SWPORTA_DDR, ddr);
	((struct rk_gpio_data *)dev->data)->ddr_shadow = ddr;
	k_spin_unlock(&((struct rk_gpio_data *)dev->data)->lock, key);

	uint32_t dr = reg_read(dev, RK_GPIO_SWPORTA_DR);
	uint32_t ext = reg_read(dev, RK_GPIO_EXT_PORTA);
	LOG_DBG("[%s] configure pin=%u flags=0x%08x -> DDR=0x%08x DR=0x%08x EXT=0x%08x",
		cfg->label, pin, (uint32_t)flags, ddr, dr, ext);

	/* Pull-ups/pull-downs e drive-strength sono gestiti dal pinctrl (IOC/GRF). */
	return 0;
}

/* --- Interrupt (stub: implementeremo nello STEP 3) --- */
static int rk_gpio_pin_interrupt_configure(const struct device *dev,
		gpio_pin_t pin, enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	ARG_UNUSED(dev); ARG_UNUSED(pin); ARG_UNUSED(mode); ARG_UNUSED(trig);
	return -ENOTSUP;
}

static int rk_gpio_manage_callback(const struct device *dev,
				   struct gpio_callback *cb, bool set)
{
	struct rk_gpio_data *data = dev->data;
	return gpio_manage_callback(&data->callbacks, cb, set);
}

static uint32_t rk_gpio_get_pending_int(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static const struct gpio_driver_api rk_gpio_api = {
	.pin_configure = rk_gpio_configure,
	.port_get_raw = rk_gpio_port_get_raw,
	.port_set_masked_raw = rk_gpio_port_set_masked_raw,
	.port_set_bits_raw = rk_gpio_port_set_bits_raw,
	.port_clear_bits_raw = rk_gpio_port_clear_bits_raw,
	.port_toggle_bits = rk_gpio_port_toggle_bits,
	.pin_interrupt_configure = rk_gpio_pin_interrupt_configure,
	.manage_callback = rk_gpio_manage_callback,
	.get_pending_int = rk_gpio_get_pending_int,
};

/* --- Init: abiliteremo i clock CRU in STEP 2; per ora dumpiamo i registri --- */
static int rk_gpio_init(const struct device *dev)
{
    const struct rk_gpio_cfg *cfg = dev->config;
    struct rk_gpio_data *data = dev->data;

    /* Abilita clock e disasserisce reset se presenti in DT */
    if (cfg->clk_dev != NULL && device_is_ready(cfg->clk_dev)) {
        (void)clock_control_on(cfg->clk_dev, (clock_control_subsys_t)&cfg->pclk);
        (void)clock_control_on(cfg->clk_dev, (clock_control_subsys_t)&cfg->dbclk);
        LOG_DBG("%s: clocks enabled (pclk=%u dbclk=%u)", cfg->label, cfg->pclk.id, cfg->dbclk.id);
    }
    /* Resets: omesse per ora (CRU reset driver non implementato) */

    /* Mappa l'area MMIO come Device (no cache) su ARMv8-A */
    uint8_t *va = NULL;
    k_mem_map_phys_bare(&va, cfg->mmio_phys, cfg->mmio_size, RK_MMIO_FLAGS_DEVICE);
	if (va == NULL) {
		LOG_ERR("%s: MMIO map failed (phys=0x%lx size=0x%lx)", cfg->label,
			(unsigned long)cfg->mmio_phys, (unsigned long)cfg->mmio_size);
		return -ENOMEM;
	}
	data->mmio_base = (mm_reg_t)(uintptr_t)va;

	/* Inizializza lo shadow DR con il valore hardware corrente (se leggibile) */
	data->dr_shadow = reg_read(dev, RK_GPIO_SWPORTA_DR);
	data->ddr_shadow = reg_read(dev, RK_GPIO_SWPORTA_DDR);

	LOG_INF("%s: init (bank=%u base=0x%lx)", cfg->label, cfg->bank_idx,
	        (unsigned long)data->mmio_base);

	rk_gpio_dump_regs(dev, "after-init");
	return 0;
}

/* --- Instanze dai DT --- */
#define RK_GPIO_DEFINE(inst)                                                        \
    static struct rk_gpio_data rk_gpio_data_##inst;                             \
    static const struct rk_gpio_cfg rk_gpio_cfg_##inst = {                      \
        .common = {                                                             \
            .port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(inst),           \
        },                                                                      \
        .mmio_phys = DT_INST_REG_ADDR(inst),                                     \
        .mmio_size = DT_INST_REG_SIZE(inst),                                     \
        .bank_idx = DT_INST_PROP(inst, rockchip_bank_index),                     \
        .irq = -1,                                                               \
        .label = DT_INST_PROP_OR(inst, label, "GPIO_RK3588"),                  \
        .clk_dev = COND_CODE_1(DT_CLOCKS_HAS_IDX(DT_DRV_INST(inst), 0),         \
                    (DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_IDX(inst, 0))),       \
                    (NULL)),                                                     \
        .pclk = { .id = COND_CODE_1(DT_CLOCKS_HAS_IDX(DT_DRV_INST(inst), 0),    \
                    (DT_INST_CLOCKS_CELL_BY_IDX(inst, 0, id)), (0)) },          \
        .dbclk = { .id = COND_CODE_1(DT_CLOCKS_HAS_IDX(DT_DRV_INST(inst), 1),   \
                    (DT_INST_CLOCKS_CELL_BY_IDX(inst, 1, id)), (0)) },          \
    };                                                                           \
    DEVICE_DT_INST_DEFINE(inst, &rk_gpio_init, NULL,                             \
                  &rk_gpio_data_##inst, &rk_gpio_cfg_##inst,             \
                  POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY, &rk_gpio_api);

DT_INST_FOREACH_STATUS_OKAY(RK_GPIO_DEFINE)
