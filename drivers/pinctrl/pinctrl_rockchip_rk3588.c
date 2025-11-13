/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Minimal RK3588 pinctrl support for Zephyr.
 */

#define DT_DRV_COMPAT rockchip_rk3588_pinctrl

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/devicetree/clocks.h>
#include <zephyr/devicetree/reset.h>
#include <zephyr/spinlock.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/rockchip_clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/kernel/internal/mm.h>
#include <zephyr/dt-bindings/pinctrl/rockchip-rk3588-pinctrl.h>
#include <zephyr/drivers/clock_control/rk3588_cru.h>

LOG_MODULE_REGISTER(pinctrl_rk3588, CONFIG_PINCTRL_LOG_LEVEL);

#ifndef RK_MMIO_FLAGS_DEVICE
#define RK_MMIO_FLAGS_DEVICE 0xA
#endif

#define RK3588_PINCTRL_NODE DT_NODELABEL(pinctrl)

enum rk3588_ioc_region_id {
	RK3588_IOC_REGION_NONE = 0,
	RK3588_IOC_REGION_BUS,
	RK3588_IOC_REGION_VCCIO14,
	RK3588_IOC_REGION_VCCIO35,
	RK3588_IOC_REGION_EMMC,
	RK3588_IOC_REGION_VCCIO6,
	RK3588_IOC_REGION_MAX
};

struct rk3588_ioc_region {
	const char *dbg_name;
	mm_reg_t base;
	bool ready;
	struct k_spinlock lock;
};

static struct rk3588_ioc_region rk3588_regions[RK3588_IOC_REGION_MAX] = {
	[RK3588_IOC_REGION_BUS] = {
		.dbg_name = "bus",
	},
	[RK3588_IOC_REGION_VCCIO14] = {
		.dbg_name = "vccio1_4",
	},
	[RK3588_IOC_REGION_VCCIO35] = {
		.dbg_name = "vccio3_5",
	},
	[RK3588_IOC_REGION_EMMC] = {
		.dbg_name = "emmc",
	},
	[RK3588_IOC_REGION_VCCIO6] = {
		.dbg_name = "vccio6",
	},
};

static inline const char *rk3588_region_name(enum rk3588_ioc_region_id id)
{
	if (id <= RK3588_IOC_REGION_NONE || id >= RK3588_IOC_REGION_MAX) {
		return "unknown";
	}

	const char *name = rk3588_regions[id].dbg_name;

	return (name == NULL) ? "unknown" : name;
}

/* BUS_IOC layout helper (same pattern used by Linux driver) */
static inline uint32_t rk_ioc_iomux_off(uint8_t bank, uint8_t port, uint8_t pin)
{
	const uint32_t bus_base = 0x8000U;
	uint32_t bank_idx = (bank == 0U) ? 0U : (bank - 1U);
	uint32_t rel = (bank == 0U) ? (port * 0x8U)
				      : (0x20U + bank_idx * 0x20U + port * 0x8U);
	uint32_t off = bus_base + rel;

	if (pin >= 4U) {
		off += 4U; /* SEL_H */
	}

	return off - bus_base;
}

static inline uintptr_t rk3588_region_phys_addr(enum rk3588_ioc_region_id id)
{
	switch (id) {
	case RK3588_IOC_REGION_BUS:
		return DT_REG_ADDR_BY_NAME(RK3588_PINCTRL_NODE, bus);
	case RK3588_IOC_REGION_VCCIO14:
		return DT_REG_ADDR_BY_NAME(RK3588_PINCTRL_NODE, vccio1_4);
	case RK3588_IOC_REGION_VCCIO35:
		return DT_REG_ADDR_BY_NAME(RK3588_PINCTRL_NODE, vccio3_5);
	case RK3588_IOC_REGION_EMMC:
		return DT_REG_ADDR_BY_NAME(RK3588_PINCTRL_NODE, emmc);
	case RK3588_IOC_REGION_VCCIO6:
		return DT_REG_ADDR_BY_NAME(RK3588_PINCTRL_NODE, vccio6);
	default:
		return 0;
	}
}

static inline size_t rk3588_region_size(enum rk3588_ioc_region_id id)
{
	switch (id) {
	case RK3588_IOC_REGION_BUS:
		return DT_REG_SIZE_BY_NAME(RK3588_PINCTRL_NODE, bus);
	case RK3588_IOC_REGION_VCCIO14:
		return DT_REG_SIZE_BY_NAME(RK3588_PINCTRL_NODE, vccio1_4);
	case RK3588_IOC_REGION_VCCIO35:
		return DT_REG_SIZE_BY_NAME(RK3588_PINCTRL_NODE, vccio3_5);
	case RK3588_IOC_REGION_EMMC:
		return DT_REG_SIZE_BY_NAME(RK3588_PINCTRL_NODE, emmc);
	case RK3588_IOC_REGION_VCCIO6:
		return DT_REG_SIZE_BY_NAME(RK3588_PINCTRL_NODE, vccio6);
	default:
		return 0;
	}
}

static int rk3588_region_enable(enum rk3588_ioc_region_id region_id)
{
	struct rk3588_ioc_region *region;
	k_spinlock_key_t key;
	int ret = 0;
	static bool printed_layout;

	LOG_DBG("rk3588_region_enable(region_id=%d)", region_id);

	if (region_id <= RK3588_IOC_REGION_NONE || region_id >= RK3588_IOC_REGION_MAX) {
		return -EINVAL;
	}

	region = &rk3588_regions[region_id];
	key = k_spin_lock(&region->lock);

	if (region->ready) {
		k_spin_unlock(&region->lock, key);
		return 0;
	}

#if DT_NODE_HAS_PROP(RK3588_PINCTRL_NODE, clocks)
	if (region_id == RK3588_IOC_REGION_BUS) {
		const struct device *clk_dev =
			DEVICE_DT_GET(DT_CLOCKS_CTLR_BY_NAME(RK3588_PINCTRL_NODE,
							     pclk_bus_ioc));

		if (!device_is_ready(clk_dev)) {
			LOG_ERR("CRU device not ready for region %s", region->dbg_name);
			ret = -ENODEV;
			goto out_unlock;
		}

		const struct rockchip_clk_subsys pclk = {
			.id = DT_CLOCKS_CELL_BY_NAME(RK3588_PINCTRL_NODE, pclk_bus_ioc, id),
		};

		ret = clock_control_on(clk_dev, (clock_control_subsys_t)&pclk);
		if (ret < 0 && ret != -ENOTSUP) {
			LOG_ERR("Failed to enable clock for region %s (id=%u): %d",
				region->dbg_name, pclk.id, ret);
			goto out_unlock;
		}
	}
#endif /* clocks */

#if IS_ENABLED(CONFIG_RESET) && DT_NODE_HAS_PROP(RK3588_PINCTRL_NODE, resets)
	if (region_id == RK3588_IOC_REGION_BUS) {
		const struct device *rst_dev =
			DEVICE_DT_GET(DT_RESET_CTLR_BY_NAME(RK3588_PINCTRL_NODE,
							    rst_p_bus_ioc));

		if (device_is_ready(rst_dev)) {
			const uint32_t reset_id =
				DT_RESET_CELL_BY_NAME(RK3588_PINCTRL_NODE, rst_p_bus_ioc, id);

			ret = reset_line_deassert(rst_dev, reset_id);
			if (ret < 0 && ret != -ENOSYS) {
				LOG_ERR("Failed to de-assert reset for region %s (id=%u): %d",
					region->dbg_name, reset_id, ret);
				goto out_unlock;
			}
		} else {
			LOG_WRN("Reset controller not ready, skipping deassert for %s",
				region->dbg_name);
		}
	}
#endif /* reset handling */

	uint8_t *va = NULL;
	uintptr_t phys = rk3588_region_phys_addr(region_id);
	size_t size = rk3588_region_size(region_id);

	if (phys == 0U || size == 0U) {
		ret = -EINVAL;
		goto out_unlock;
	}

	k_mem_map_phys_bare(&va, phys, size, RK_MMIO_FLAGS_DEVICE);
	if (va == NULL) {
		LOG_ERR("Failed to map IOC region %s", region->dbg_name);
		ret = -ENOMEM;
		goto out_unlock;
	}

	region->base = (mm_reg_t)(uintptr_t)va;
	region->ready = true;

	LOG_DBG("Mapped IOC region %s: phys=0x%lx -> virt=0x%lx size=0x%lx",
		region->dbg_name,
		(unsigned long)phys,
		(unsigned long)region->base,
		(unsigned long)size);

	if (region_id == RK3588_IOC_REGION_BUS && !printed_layout) {
		printed_layout = true;
		for (uint8_t b = 0; b <= 4; b++) {
			for (uint8_t p = 0; p < 4; p++) {
				uint32_t off = rk_ioc_iomux_off(b, p, 0);
				LOG_INF("[rockchip-pinctrl] bank %u, iomux %u has iom_offset 0x%x drv_offset 0x0",
					 b, p, off);
			}
		}
	}

out_unlock:
	k_spin_unlock(&region->lock, key);
	return ret;
}

static inline mm_reg_t rk3588_region_base(enum rk3588_ioc_region_id id)
{
	if (id <= RK3588_IOC_REGION_NONE || id >= RK3588_IOC_REGION_MAX) {
		return (mm_reg_t)0;
	}

	return rk3588_regions[id].base;
}

struct rk3588_port_cfg {
	enum rk3588_ioc_region_id region;
	uint16_t ds_l;
	uint16_t ds_h;
	uint16_t pull;
	uint16_t ie;
	uint16_t schmitt;
};

struct rk3588_bank_cfg {
	struct rk3588_port_cfg ports[4];
};

/* Only bank1 is fully described for now (GPIO1_* via VCCIO1_4_IOC) */
static const struct rk3588_bank_cfg rk3588_bank_cfgs[] = {
	[0] = {
		.ports = { },
	},
	[1] = {
		.ports = {
			{
				.region = RK3588_IOC_REGION_VCCIO14,
				.ds_l = 0x0020, .ds_h = 0x0024,
				.pull = 0x0110, .ie = 0x0180, .schmitt = 0x0210,
			},
			{
				.region = RK3588_IOC_REGION_VCCIO14,
				.ds_l = 0x0028, .ds_h = 0x002C,
				.pull = 0x0114, .ie = 0x0184, .schmitt = 0x0214,
			},
			{
				.region = RK3588_IOC_REGION_VCCIO14,
				.ds_l = 0x0030, .ds_h = 0x0034,
				.pull = 0x0118, .ie = 0x0188, .schmitt = 0x0218,
			},
			{
				.region = RK3588_IOC_REGION_VCCIO14,
				.ds_l = 0x0038, .ds_h = 0x003C,
				.pull = 0x011C, .ie = 0x018C, .schmitt = 0x021C,
			},
		},
	},
	[2] = {
		.ports = {
			{
				.region = RK3588_IOC_REGION_EMMC,
				.ds_l = 0x0040, .ds_h = 0x0044,
				.pull = 0x0120, .ie = 0, .schmitt = 0x0220,
			},
			{
				.region = RK3588_IOC_REGION_VCCIO35,
				.ds_l = 0x0048, .ds_h = 0x004C,
				.pull = 0x0124, .ie = 0, .schmitt = 0x0224,
			},
			{
				.region = RK3588_IOC_REGION_VCCIO35,
				.ds_l = 0x0050, .ds_h = 0x0054,
				.pull = 0x0128, .ie = 0, .schmitt = 0x0228,
			},
			{
				.region = RK3588_IOC_REGION_EMMC,
				.ds_l = 0x0058, .ds_h = 0x005C,
				.pull = 0x012C, .ie = 0, .schmitt = 0x022C,
			},
		},
	},
	[3] = {
		.ports = {
			{
				.region = RK3588_IOC_REGION_VCCIO35,
				.ds_l = 0x0060, .ds_h = 0x0064,
				.pull = 0x0130, .ie = 0, .schmitt = 0x0230,
			},
			{
				.region = RK3588_IOC_REGION_VCCIO35,
				.ds_l = 0x0068, .ds_h = 0x006C,
				.pull = 0x0134, .ie = 0, .schmitt = 0x0234,
			},
			{
				.region = RK3588_IOC_REGION_VCCIO35,
				.ds_l = 0x0070, .ds_h = 0x0074,
				.pull = 0x0138, .ie = 0, .schmitt = 0x0238,
			},
			{
				.region = RK3588_IOC_REGION_VCCIO35,
				.ds_l = 0x0078, .ds_h = 0x007C,
				.pull = 0x013C, .ie = 0, .schmitt = 0x023C,
			},
		},
	},
	[4] = {
		.ports = {
			{
				.region = RK3588_IOC_REGION_VCCIO6,
				.ds_l = 0x0080, .ds_h = 0x0084,
				.pull = 0x0140, .ie = 0, .schmitt = 0x0240,
			},
			{
				.region = RK3588_IOC_REGION_VCCIO6,
				.ds_l = 0x0088, .ds_h = 0x008C,
				.pull = 0x0144, .ie = 0, .schmitt = 0x0244,
			},
			{
				.region = RK3588_IOC_REGION_VCCIO6,
				.ds_l = 0x0090, .ds_h = 0x0094,
				.pull = 0x0148, .ie = 0, .schmitt = 0x0248,
			},
			{
				.region = RK3588_IOC_REGION_VCCIO6,
				.ds_l = 0x0098, .ds_h = 0x009C,
				.pull = 0x014C, .ie = 0, .schmitt = 0x024C,
			},
		},
	},
};

static inline bool rk3588_bank_valid(uint8_t bank)
{
	return bank < ARRAY_SIZE(rk3588_bank_cfgs);
}

static const struct rk3588_bank_cfg *rk3588_get_bank_cfg(uint8_t bank)
{
	if (!rk3588_bank_valid(bank)) {
		return NULL;
	}

	return &rk3588_bank_cfgs[bank];
}

static inline const struct rk3588_port_cfg *rk3588_get_port_cfg(uint8_t bank, uint8_t port)
{
	const struct rk3588_bank_cfg *bank_cfg = rk3588_get_bank_cfg(bank);

	if (bank_cfg == NULL || port >= ARRAY_SIZE(bank_cfg->ports)) {
		return NULL;
	}

	return &bank_cfg->ports[port];
}

static inline void rk3588_pick_pull_region_off(uint8_t bank, uint8_t port, uint8_t pin,
					       enum rk3588_ioc_region_id *rid,
					       uint16_t *off)
{
	const struct rk3588_port_cfg *pc = rk3588_get_port_cfg(bank, port);

	if (pc == NULL) {
		*rid = RK3588_IOC_REGION_NONE;
		*off = 0U;
		return;
	}

	*rid = pc->region;
	*off = pc->pull;

	if (bank == 2U && port == RK3588_PORT_A) {
		*rid = (pin >= 6U) ? RK3588_IOC_REGION_VCCIO35 : RK3588_IOC_REGION_EMMC;
	}
}

static inline void rk3588_pick_smt_region_off(uint8_t bank, uint8_t port, uint8_t pin,
					       enum rk3588_ioc_region_id *rid,
					       uint16_t *off)
{
	const struct rk3588_port_cfg *pc = rk3588_get_port_cfg(bank, port);

	if (pc == NULL) {
		*rid = RK3588_IOC_REGION_NONE;
		*off = 0U;
		return;
	}

	*rid = pc->region;
	*off = pc->schmitt;

	if (bank == 2U && port == RK3588_PORT_A) {
		*rid = (pin >= 6U) ? RK3588_IOC_REGION_VCCIO35 : RK3588_IOC_REGION_EMMC;
	}
}

static inline void rk3588_pick_drv_region_off(uint8_t bank, uint8_t port, uint8_t pin,
					      enum rk3588_ioc_region_id *rid,
					      uint16_t *off)
{
	const struct rk3588_port_cfg *pc = rk3588_get_port_cfg(bank, port);

	if (pc == NULL) {
		*rid = RK3588_IOC_REGION_NONE;
		*off = 0U;
		return;
	}

	*rid = pc->region;
	*off = (pin >= 4U) ? pc->ds_h : pc->ds_l;

	if (bank == 2U && port == RK3588_PORT_A) {
		if (pin < 4U) {
			*rid = RK3588_IOC_REGION_EMMC;
			*off = pc->ds_l;
		} else {
			*rid = RK3588_IOC_REGION_VCCIO35;
			*off = pc->ds_h;
		}
	}
}

static int rk3588_configure_pull(uint8_t bank, uint8_t port, uint8_t pin, uint32_t flags,
				    const struct rk3588_port_cfg *port_cfg)
{
	uint32_t sel = (flags & RK3588_PINCFG_PULL_MASK) >> RK3588_PINCFG_PULL_SHIFT;
	uint32_t shift = 2U * (pin & 0x7U);
	enum rk3588_ioc_region_id region_id;
	uint16_t offset;
	int ret;

	LOG_DBG("configure_pull(bank=%u port=%u pin=%u flags=0x%08x)",
		bank, port, pin, flags);

	if (sel == (RK3588_PINCFG_PULL_KEEP >> RK3588_PINCFG_PULL_SHIFT)) {
		return 0;
	}

	if (port_cfg->pull == 0U) {
		return -ENOTSUP;
	}

	rk3588_pick_pull_region_off(bank, port, pin, &region_id, &offset);
	if (region_id == RK3588_IOC_REGION_NONE || offset == 0U) {
		return -ENOTSUP;
	}

	ret = rk3588_region_enable(region_id);
	if (ret < 0) {
		return ret;
	}

	mm_reg_t base = rk3588_region_base(region_id);
	if (base == (mm_reg_t)0) {
		return -EIO;
	}

	LOG_INF("[rockchip-pinctrl] set pull bank%u port%c pin%u -> sel=%u (shift=%u) off=0x%04x region=%s",
		(unsigned)bank, (char)('A' + port), (unsigned)pin, (unsigned)sel,
		(unsigned)shift, (unsigned)offset, rk3588_region_name(region_id));

	sys_write32(HIWORD_UPDATE(sel, 0x3U, shift), base + offset);
	return 0;
}

static int rk3588_configure_drive(uint8_t bank, uint8_t port, uint8_t pin, uint32_t flags,
				    const struct rk3588_port_cfg *port_cfg)
{
	enum rk3588_ioc_region_id region_id;
	uint16_t offset;
	int ret;

	LOG_DBG("configure_drive(bank=%u port=%u pin=%u flags=0x%08x)",
		bank, port, pin, flags);

	if ((flags & RK3588_PINCFG_DRIVE_FLAG) == 0U) {
		return 0;
	}

	if (port_cfg->ds_l == 0U && port_cfg->ds_h == 0U) {
		return -ENOTSUP;
	}

	uint32_t drive = (flags & RK3588_PINCFG_DRIVE_MASK) >> RK3588_PINCFG_DRIVE_SHIFT;
	uint32_t shift = 4U * (pin & 0x3U);

	rk3588_pick_drv_region_off(bank, port, pin, &region_id, &offset);
	if (region_id == RK3588_IOC_REGION_NONE || offset == 0U) {
		return -ENOTSUP;
	}

	ret = rk3588_region_enable(region_id);
	if (ret < 0) {
		return ret;
	}

	mm_reg_t base = rk3588_region_base(region_id);
	if (base == (mm_reg_t)0) {
		return -EIO;
	}

	LOG_INF("[rockchip-pinctrl] set drive bank%u port%c pin%u -> drv=%u (shift=%u) off=0x%04x region=%s",
		(unsigned)bank, (char)('A' + port), (unsigned)pin, (unsigned)drive,
		(unsigned)shift, (unsigned)offset, rk3588_region_name(region_id));

	sys_write32(HIWORD_UPDATE(drive, 0xFU, shift), base + offset);
	return 0;
}

static int rk3588_configure_schmitt(uint8_t bank, uint8_t port, uint8_t pin, uint32_t flags,
				       const struct rk3588_port_cfg *port_cfg)
{
	uint32_t sel = (flags & RK3588_PINCFG_SCHMITT_MASK) >> RK3588_PINCFG_SCHMITT_SHIFT;
	uint32_t value;
	enum rk3588_ioc_region_id region_id;
	uint16_t offset;
	int ret;

	LOG_DBG("configure_schmitt(bank=%u port=%u pin=%u flags=0x%08x)",
		bank, port, pin, flags);

	if (sel == (RK3588_PINCFG_SCHMITT_KEEP >> RK3588_PINCFG_SCHMITT_SHIFT)) {
		return 0;
	}

	if (port_cfg->schmitt == 0U) {
		return -ENOTSUP;
	}

	value = (sel == (RK3588_PINCFG_SCHMITT_ENABLE >> RK3588_PINCFG_SCHMITT_SHIFT)) ? 1U : 0U;

	rk3588_pick_smt_region_off(bank, port, pin, &region_id, &offset);
	if (region_id == RK3588_IOC_REGION_NONE || offset == 0U) {
		return -ENOTSUP;
	}

	ret = rk3588_region_enable(region_id);
	if (ret < 0) {
		return ret;
	}

	mm_reg_t base = rk3588_region_base(region_id);
	if (base == (mm_reg_t)0) {
		return -EIO;
	}

	LOG_INF("[rockchip-pinctrl] set schmitt bank%u port%c pin%u -> %s (off=0x%04x region=%s)",
		(unsigned)bank, (char)('A' + port), (unsigned)pin,
		value ? "enable" : "disable", (unsigned)offset,
		rk3588_region_name(region_id));

	sys_write32(HIWORD_UPDATE(value, 0x1U, pin), base + offset);
	return 0;
}

static int rk3588_configure_input_enable(uint8_t bank, uint8_t port, uint8_t pin,
					    const struct rk3588_port_cfg *port_cfg)
{
	int ret;
	enum rk3588_ioc_region_id region_id = port_cfg->region;

	LOG_DBG("configure_input_enable(bank=%u port=%u pin=%u)",
		bank, port, pin);

	if (port_cfg->ie == 0U) {
		return -ENOTSUP;
	}

	if (region_id == RK3588_IOC_REGION_NONE) {
		return -ENOTSUP;
	}

	ret = rk3588_region_enable(region_id);
	if (ret < 0) {
		return ret;
	}

	mm_reg_t base = rk3588_region_base(region_id);
	if (base == (mm_reg_t)0) {
		return -EIO;
	}

	LOG_INF("[rockchip-pinctrl] input-enable bank%u port%c pin%u (off=0x%04x region=%s)",
		(unsigned)bank, (char)('A' + port), (unsigned)pin,
		(unsigned)port_cfg->ie, rk3588_region_name(region_id));

	sys_write32(HIWORD_UPDATE(1U, 0x1U, pin), base + port_cfg->ie);
	return 0;
}

static int rk3588_configure_electrical(uint8_t bank, uint8_t port, uint8_t pin, uint32_t flags)
{
	const struct rk3588_port_cfg *port_cfg = rk3588_get_port_cfg(bank, port);
	int ret = 0;

	LOG_DBG("configure_electrical(bank=%u port=%u pin=%u flags=0x%08x)",
		bank, port, pin, flags);

	if (port_cfg == NULL) {
		return -EINVAL;
	}

	if (port_cfg->region == RK3588_IOC_REGION_NONE &&
	    port_cfg->ds_l == 0U && port_cfg->ds_h == 0U &&
	    port_cfg->pull == 0U && port_cfg->ie == 0U &&
	    port_cfg->schmitt == 0U) {
		if ((flags & RK3588_PINCFG_FLAGS_MASK) != 0U) {
			LOG_DBG("Electrical config ignored for unsupported bank%u port%u", bank, port);
		}
		return 0;
	}

	/* Pull configuration */
	ret = rk3588_configure_pull(bank, port, pin, flags, port_cfg);
	if (ret == -ENOTSUP) {
		LOG_DBG("Pull configuration unavailable for bank%u port%u", bank, port);
		ret = 0;
	} else if (ret < 0) {
		return ret;
	}

	/* Drive strength */
	ret = rk3588_configure_drive(bank, port, pin, flags, port_cfg);
	if (ret == -ENOTSUP) {
		LOG_DBG("Drive configuration unavailable for bank%u port%u", bank, port);
		ret = 0;
	} else if (ret < 0) {
		return ret;
	}

	/* Schmitt trigger */
	ret = rk3588_configure_schmitt(bank, port, pin, flags, port_cfg);
	if (ret == -ENOTSUP) {
		LOG_DBG("Schmitt configuration unavailable for bank%u port%u", bank, port);
		ret = 0;
	} else if (ret < 0) {
		return ret;
	}

	/* Ensure input buffer enabled */
	ret = rk3588_configure_input_enable(bank, port, pin, port_cfg);
	if (ret == -ENOTSUP) {
		LOG_DBG("Input enable register missing for bank%u port%u", bank, port);
		ret = 0;
	}

	return ret;
}

static int rk3588_configure_mux(uint8_t bank, uint8_t port, uint8_t pin, uint8_t func)
{
	int ret = rk3588_region_enable(RK3588_IOC_REGION_BUS);
	mm_reg_t base;
	uint32_t offset;
	uint32_t shift;
	uint32_t mask = 0xFU;
	uint32_t value;

	LOG_DBG("configure_mux(bank=%u port=%u pin=%u func=%u)", bank, port, pin, func);

	if (ret < 0) {
		LOG_ERR("BUS_IOC not ready (%d)", ret);
		return ret;
	}

	base = rk3588_region_base(RK3588_IOC_REGION_BUS);
	if (base == (mm_reg_t)0) {
		return -EIO;
	}

	offset = rk_ioc_iomux_off(bank, port, pin);
	shift = 4U * (pin & 0x3U);
	value = (uint32_t)func;
	/* Mirror Linux-style log for easier diffing */
	{
		uint32_t gpio_index = (uint32_t)port * 8U + (uint32_t)pin;
		LOG_INF("[rockchip-pinctrl] setting mux of GPIO%u-%u to %u",
			bank, gpio_index, func);
	}

	sys_write32(HIWORD_UPDATE(value, mask, shift), base + offset);

	LOG_DBG("muxed: bank%u %c%u -> func=%u @off 0x%04x",
		bank, 'A' + port, pin, func, offset);

	return 0;
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pins_cnt, uintptr_t reg_base_unused)
{
	ARG_UNUSED(reg_base_unused);

	LOG_DBG("pinctrl_configure_pins(cnt=%u)", pins_cnt);

	for (uint8_t i = 0; i < pins_cnt; i++) {
		uint32_t cfg = pins[i];
		uint8_t bank = RK3588_PINMUX_BANK(cfg);
		uint8_t port = RK3588_PINMUX_PORT(cfg);
		uint8_t pin = RK3588_PINMUX_PIN(cfg);
		uint8_t func = RK3588_PINMUX_FUNC(cfg);
		uint32_t flags = RK3588_PINMUX_FLAGS(cfg);
		int ret;

		if (!rk3588_bank_valid(bank) || port > 3U || pin > 7U) {
			LOG_ERR("Invalid pinmux entry 0x%08x", cfg);
			return -EINVAL;
		}

		ret = rk3588_configure_mux(bank, port, pin, func);
		if (ret < 0) {
			return ret;
		}

		ret = rk3588_configure_electrical(bank, port, pin, flags);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}
