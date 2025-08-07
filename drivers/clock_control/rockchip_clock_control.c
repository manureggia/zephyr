#define DT_DRV_COMPAT rockchip_rk3588_cru

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/rockchip_clock_control.h>
#include <zephyr/sys/util.h>
#include <zephyr/devicetree.h>

#include <zephyr/drivers/reset.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control_rk3588, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

struct rockchip_clk_control_config {
	DEVICE_MMIO_ROM;
};

static int rockchip_clock_control_on(const struct device *dev,
				     clock_control_subsys_t sys)
{
	const struct rockchip_clk_subsys *subsys = sys;
	const struct rockchip_clk_control_config *cfg = dev->config;

	LOG_DBG("Enable clock ID %u", subsys->id);

	/* In un vero driver: manipoleresti i registri del CRU per abilitare */
	/* Per ora, si suppone che i clock siano già attivi */
	ARG_UNUSED(cfg);
	ARG_UNUSED(subsys);

	return 0;
}

static int rockchip_clock_control_off(const struct device *dev,
				      clock_control_subsys_t sys)
{
	const struct rockchip_clk_subsys *subsys = sys;
	LOG_DBG("Disable clock ID %u", subsys->id);

	/* Stub: non disabilitiamo nulla realmente */
	ARG_UNUSED(dev);
	ARG_UNUSED(subsys);
	return 0;
}

static int rockchip_clock_control_get_rate(const struct device *dev,
					   clock_control_subsys_t sys,
					   uint32_t *rate)
{
	/* Potresti leggere la frequenza effettiva dal PLL o da un registro */
	*rate = 24000000;  /* Esempio: 24 MHz */
	return 0;
}

static const struct clock_control_driver_api rockchip_clock_control_api = {
	.on = rockchip_clock_control_on,
	.off = rockchip_clock_control_off,
	.get_rate = rockchip_clock_control_get_rate,
};

static const struct rockchip_clk_control_config rockchip_cru_config = {
	DEVICE_MMIO_ROM_INIT(DT_DRV_INST(0)),
};

DEVICE_DT_INST_DEFINE(0,
		      NULL,
		      NULL,
		      NULL,
		      &rockchip_cru_config,
		      PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		      &rockchip_clock_control_api);