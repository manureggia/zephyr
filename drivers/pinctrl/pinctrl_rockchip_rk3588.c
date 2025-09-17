/* SPDX-License-Identifier: Apache-2.0 */
/* Zephyr 4.2.0-rc2 — RK3588 pinctrl (BUS_IOC mux only). */
#define DT_DRV_COMPAT rockchip_rk3588_pinctrl

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/mem_manage.h>
#include <zephyr/drivers/pinctrl.h>
#include <pinctrl_soc.h>
#include <zephyr/kernel/internal/mm.h>
#include <zephyr/dt-bindings/pinctrl/rockchip-rk3588-pinctrl.h>

LOG_MODULE_REGISTER(pinctrl_rk3588, CONFIG_PINCTRL_LOG_LEVEL);

/* Mappa attributi per regioni Device (nGnRE), come per GPIO */
#ifndef RK_MMIO_FLAGS_DEVICE
#define RK_MMIO_FLAGS_DEVICE 0xA
#endif

/* BUS_IOC layout (TRM RK3588, semplificato):
 * Per ogni bank 1..4 e port (A..D=0..3):
 *   IOMUX_SEL_L (pin 0..3), IOMUX_SEL_H (pin 4..7)
 * Pattern offset parametrico:
 *   base_off = 0x20
 *   block per bank: 0x40  (bank=1 -> +0x00, bank=2 -> +0x40, ...)
 *   sub-block per port: 0x08 (A=0..D=3)
 *   +4 se H (pin>=4)
 */
static inline uint32_t rk_ioc_iomux_off(uint8_t bank, uint8_t port, uint8_t pin)
{
	/* NB: bank==0 (GPIO0) non è in BUS_IOC ma in PMU_IOC: non supportato qui */
	uint32_t bank_idx = (bank == 0) ? 0 : (bank - 1);
	uint32_t off = 0x20 + (bank_idx * 0x40) + (port * 0x8);
	if (pin >= 4U) {
		off += 4U; /* SEL_H */
	}
	return off;
}

struct rk_pinctrl_cfg {
	/* base fisica BUS_IOC dal DT */
	uintptr_t phys;
	size_t    size;
};

struct rk_pinctrl_data {
	/* base virtuale mappata */
	mm_reg_t base;
};

/* Puntatore globale alla base mappata del primo (e unico) controller.
 * Il core pinctrl chiama pinctrl_configure_pins() senza device; qui usiamo questa
 * base (settata in init) per programmare i registri.
 */
static mm_reg_t g_bus_ioc_base;

static int rk3588_pinctrl_init(const struct device *dev)
{
    const struct rk_pinctrl_cfg *cfg = dev->config;
    struct rk_pinctrl_data *data = dev->data;

	uint8_t *va = NULL;
	k_mem_map_phys_bare(&va, cfg->phys, cfg->size, RK_MMIO_FLAGS_DEVICE);
	if (!va) {
		LOG_ERR("BUS_IOC map failed (phys=0x%lx size=0x%lx)",
			(unsigned long)cfg->phys, (unsigned long)cfg->size);
		return -ENOMEM;
	}
    data->base = (mm_reg_t)(uintptr_t)va;

    /* rende disponibile la base per pinctrl_configure_pins() */
    g_bus_ioc_base = data->base;

    LOG_INF("BUS_IOC mapped: phys=0x%lx -> virt=0x%lx size=0x%lx",
        (unsigned long)cfg->phys, (unsigned long)data->base, (unsigned long)cfg->size);
    /* stampa grezza anche se LOG backend non è pronto */
    printk("[pinctrl] BUS_IOC mapped: phys=%lx virt=%lx size=%lx\n",
           (unsigned long)cfg->phys, (unsigned long)data->base, (unsigned long)cfg->size);
    return 0;
}


/* API richiesta dal core Zephyr: configura i pin di una state */
int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pins_cnt, uintptr_t reg_base_unused)
{
    ARG_UNUSED(reg_base_unused);

    if (g_bus_ioc_base == 0) {
        LOG_ERR("BUS_IOC not mapped yet");
        printk("[pinctrl] ERROR: BUS_IOC not mapped yet\n");
        return -ENXIO;
    }

    for (uint8_t i = 0; i < pins_cnt; i++) {
        uint32_t mux = pins[i];
		uint8_t bank = RK3588_PINMUX_BANK(mux);
		uint8_t port = RK3588_PINMUX_PORT(mux);
		uint8_t pin  = RK3588_PINMUX_PIN(mux);
		uint8_t func = RK3588_PINMUX_FUNC(mux);

        if (bank == 0U) {
            LOG_ERR("PMU_IOC/GPIO0 not supported yet (bank=0)");
            printk("[pinctrl] bank0 not supported\n");
            return -ENOTSUP;
        }
        if (bank > 4U || port > 3U || pin > 7U || func > 0xF) {
            LOG_ERR("Invalid pinmux entry: 0x%08x", mux);
            printk("[pinctrl] invalid pinmux=0x%08x\n", mux);
            return -EINVAL;
        }

        uint32_t off   = rk_ioc_iomux_off(bank, port, pin);
        uint32_t shift = 4U * (pin & 0x3U); /* 0..12 */
        uint32_t field_mask  = 0xFU << shift;         /* mask in low 16 bits */
        uint32_t field_value = ((uint32_t)func << shift) & field_mask;

        /*
         * Rockchip IOC/GRF registri usano write-mask: i bit [31:16] selezionano
         * quali bit dei [15:0] vengono aggiornati. Scriviamo quindi
         *   write_val = (field_mask << 16) | field_value
         * per aggiornare solo il campo di interesse.
         */
        uint32_t write_val = (field_mask << 16) | field_value;

        uint32_t before = sys_read32(g_bus_ioc_base + off);

        LOG_DBG("apply: bank=%u port=%u pin=%u func=%u -> off=0x%04x shift=%u fmask=0x%08x before=0x%08x w=0x%08x",
                bank, port, pin, func, off, shift, field_mask, before, write_val);

        sys_write32(write_val, g_bus_ioc_base + off);

        /* rilettura per debug */
        uint32_t verify = sys_read32(g_bus_ioc_base + off);
        LOG_INF("muxed: bank%u %c%u -> func=%u @off 0x%04x: 0x%08x",
                bank, 'A' + port, pin, func, off, verify);
        printk("[pinctrl] muxed bank%u %c%u func=%u off=0x%04x reg=0x%08x write=0x%08x\n",
               bank, 'A' + port, pin, func, off, verify, write_val);
    }

    return 0;
}

/* Instanzia esattamente un controller (BUS_IOC) dal DT */
#define RK_PINCTRL_DEFINE(inst)                                                      \
    static struct rk_pinctrl_data rk_pinctrl_data_##inst;                        \
	static const struct rk_pinctrl_cfg rk_pinctrl_cfg_##inst = {                 \
		.phys = DT_REG_ADDR(DT_DRV_INST(inst)),                              \
		.size = DT_REG_SIZE(DT_DRV_INST(inst)),                              \
	};                                                                             \
    DEVICE_DT_INST_DEFINE(inst, &rk3588_pinctrl_init, NULL,                       \
                          &rk_pinctrl_data_##inst, &rk_pinctrl_cfg_##inst,        \
                          PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);\
    /* no extra SYS_INIT needed */

DT_INST_FOREACH_STATUS_OKAY(RK_PINCTRL_DEFINE)
