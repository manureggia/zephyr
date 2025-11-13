
#define DT_DRV_COMPAT rockchip_rk3588_cru

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/rockchip_clock_control.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/clock_control/rk3588_cru.h>

LOG_MODULE_REGISTER(clock_control_rk3588, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

struct rockchip_clk_gate {
	uint16_t id;
	uint16_t offset;
	uint8_t bit;
};

struct rockchip_clk_control_config {
	DEVICE_MMIO_ROM;
	const struct rockchip_clk_gate *gates;
	size_t gate_count;
};

struct rockchip_clk_control_data {
	DEVICE_MMIO_RAM;
	struct k_spinlock lock;
};

#define RK_GCLK(_id, _gate_idx, _bit) \
	{ .id = (_id), .offset = RK3588_CLKGATE_CON(_gate_idx), .bit = (_bit) }

/* Gating map derived from Linux clk-rk3588 driver. */
static const struct rockchip_clk_gate rk3588_gate_table[] = {
	/* Top-level bus clock */
	RK_GCLK(PCLK_TOP, 7, 6),

	/* GPIO */
	RK_GCLK(PCLK_GPIO1, 16, 14),
	RK_GCLK(DBCLK_GPIO1, 16, 15),
	RK_GCLK(PCLK_GPIO2, 17, 0),
	RK_GCLK(DBCLK_GPIO2, 17, 1),
	RK_GCLK(PCLK_GPIO3, 17, 2),
	RK_GCLK(DBCLK_GPIO3, 17, 3),
	RK_GCLK(PCLK_GPIO4, 17, 4),
	RK_GCLK(DBCLK_GPIO4, 17, 5),

	/* I2C masters */
	RK_GCLK(PCLK_I2C1, 10, 8),
	RK_GCLK(CLK_I2C1, 11, 0),
	RK_GCLK(PCLK_I2C2, 10, 9),
	RK_GCLK(CLK_I2C2, 11, 1),
	RK_GCLK(PCLK_I2C3, 10, 10),
	RK_GCLK(CLK_I2C3, 11, 2),
	RK_GCLK(PCLK_I2C4, 10, 11),
	RK_GCLK(CLK_I2C4, 11, 3),
	RK_GCLK(PCLK_I2C5, 10, 12),
	RK_GCLK(CLK_I2C5, 11, 4),
	RK_GCLK(PCLK_I2C6, 10, 13),
	RK_GCLK(CLK_I2C6, 11, 5),
	RK_GCLK(PCLK_I2C7, 10, 14),
	RK_GCLK(CLK_I2C7, 11, 6),
	RK_GCLK(PCLK_I2C8, 10, 15),
	RK_GCLK(CLK_I2C8, 11, 7),

	/* SPI controllers */
	RK_GCLK(PCLK_SPI0, 14, 6),
	RK_GCLK(CLK_SPI0, 14, 11),
	RK_GCLK(PCLK_SPI1, 14, 7),
	RK_GCLK(CLK_SPI1, 14, 12),
	RK_GCLK(PCLK_SPI2, 14, 8),
	RK_GCLK(CLK_SPI2, 14, 13),
	RK_GCLK(PCLK_SPI3, 14, 9),
	RK_GCLK(CLK_SPI3, 14, 14),
	RK_GCLK(PCLK_SPI4, 14, 10),
	RK_GCLK(CLK_SPI4, 14, 15),

	/* PWM blocks */
	RK_GCLK(PCLK_PWM1, 15, 3),
	RK_GCLK(CLK_PWM1, 15, 4),
	RK_GCLK(CLK_PWM1_CAPTURE, 15, 5),
	RK_GCLK(PCLK_PWM2, 15, 6),
	RK_GCLK(CLK_PWM2, 15, 7),
	RK_GCLK(CLK_PWM2_CAPTURE, 15, 8),
	RK_GCLK(PCLK_PWM3, 15, 9),
	RK_GCLK(CLK_PWM3, 15, 10),
	RK_GCLK(CLK_PWM3_CAPTURE, 15, 11),

	/* UARTs */
	RK_GCLK(PCLK_UART1, 12, 2),
	RK_GCLK(PCLK_UART2, 12, 3),
	RK_GCLK(PCLK_UART3, 12, 4),
	RK_GCLK(PCLK_UART4, 12, 5),
	RK_GCLK(PCLK_UART5, 12, 6),
	RK_GCLK(PCLK_UART6, 12, 7),
	RK_GCLK(PCLK_UART7, 12, 8),
	RK_GCLK(PCLK_UART8, 12, 9),
	RK_GCLK(PCLK_UART9, 12, 10),
	RK_GCLK(SCLK_UART1, 12, 13),
	RK_GCLK(SCLK_UART2, 13, 0),
	RK_GCLK(SCLK_UART3, 13, 3),
	RK_GCLK(SCLK_UART4, 13, 6),
	RK_GCLK(SCLK_UART5, 13, 9),
	RK_GCLK(SCLK_UART6, 13, 12),
	RK_GCLK(SCLK_UART7, 13, 15),
	RK_GCLK(SCLK_UART8, 14, 2),
	RK_GCLK(SCLK_UART9, 14, 5),
};

static const struct rockchip_clk_gate *rockchip_find_gate(
	const struct rockchip_clk_control_config *cfg, uint32_t id)
{
	for (size_t i = 0; i < cfg->gate_count; i++) {
		if (cfg->gates[i].id == id) {
			return &cfg->gates[i];
		}
	}

	return NULL;
}

static void rockchip_gate_write(mm_reg_t base,
				const struct rockchip_clk_gate *gate,
				bool enable)
{
	const uint32_t mask = BIT(gate->bit);
	const uint32_t value = enable ? 0U : mask;

	sys_write32((mask << 16) | value, base + gate->offset);
}

static bool rockchip_gate_is_enabled(mm_reg_t base,
				     const struct rockchip_clk_gate *gate)
{
	uint32_t reg = sys_read32(base + gate->offset);

	return (reg & BIT(gate->bit)) == 0U;
}

static int rockchip_clock_control_on(const struct device *dev,
				     clock_control_subsys_t sys)
{
	const struct rockchip_clk_control_config *cfg = dev->config;
	struct rockchip_clk_control_data *data = dev->data;
	const struct rockchip_clk_subsys *subsys = sys;
	const struct rockchip_clk_gate *gate =
		rockchip_find_gate(cfg, subsys->id);

	if (gate == NULL) {
		LOG_DBG("Clock ID %u not handled", subsys->id);
		return -ENOTSUP;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);
	mm_reg_t base = DEVICE_MMIO_GET(dev);

	if (!rockchip_gate_is_enabled(base, gate)) {
		rockchip_gate_write(base, gate, true);
	}

	k_spin_unlock(&data->lock, key);

	return 0;
}

static int rockchip_clock_control_off(const struct device *dev,
				      clock_control_subsys_t sys)
{
	const struct rockchip_clk_control_config *cfg = dev->config;
	struct rockchip_clk_control_data *data = dev->data;
	const struct rockchip_clk_subsys *subsys = sys;
	const struct rockchip_clk_gate *gate =
		rockchip_find_gate(cfg, subsys->id);

	if (gate == NULL) {
		LOG_DBG("Clock ID %u not handled", subsys->id);
		return -ENOTSUP;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);
	mm_reg_t base = DEVICE_MMIO_GET(dev);

	if (rockchip_gate_is_enabled(base, gate)) {
		rockchip_gate_write(base, gate, false);
	}

	k_spin_unlock(&data->lock, key);

	return 0;
}

static int rockchip_clock_control_get_rate(const struct device *dev,
					   clock_control_subsys_t sys,
					   uint32_t *rate)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sys);

	return -ENOTSUP;
}

static enum clock_control_status rockchip_clock_control_get_status(
	const struct device *dev, clock_control_subsys_t sys)
{
	const struct rockchip_clk_control_config *cfg = dev->config;
	const struct rockchip_clk_subsys *subsys = sys;
	const struct rockchip_clk_gate *gate =
		rockchip_find_gate(cfg, subsys->id);

	if (gate == NULL) {
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}

	mm_reg_t base = DEVICE_MMIO_GET(dev);

	return rockchip_gate_is_enabled(base, gate) ?
		CLOCK_CONTROL_STATUS_ON : CLOCK_CONTROL_STATUS_OFF;
}

static int rockchip_clock_control_init(const struct device *dev)
{
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	return 0;
}

static const struct clock_control_driver_api rockchip_clock_control_api = {
	.on = rockchip_clock_control_on,
	.off = rockchip_clock_control_off,
	.get_rate = rockchip_clock_control_get_rate,
	.get_status = rockchip_clock_control_get_status,
};

static struct rockchip_clk_control_data rockchip_cru_data;

static const struct rockchip_clk_control_config rockchip_cru_config = {
	DEVICE_MMIO_ROM_INIT(DT_DRV_INST(0)),
	.gates = rk3588_gate_table,
	.gate_count = ARRAY_SIZE(rk3588_gate_table),
};

DEVICE_DT_INST_DEFINE(0,
		      rockchip_clock_control_init,
		      NULL,
		      &rockchip_cru_data,
		      &rockchip_cru_config,
		      PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		      &rockchip_clock_control_api);
