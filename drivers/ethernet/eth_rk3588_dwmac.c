/*
 * Rockchip RK3588 DWMAC glue driver.
 *
 * Brings up the RK3588 Synopsys DesignWare MAC instances and bridges them
 * with Zephyr's generic dwmac core.
 *
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT rockchip_rk3588_dwmac

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eth_rk3588, CONFIG_ETHERNET_LOG_LEVEL);

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/rockchip_clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/mii.h>
#include <zephyr/irq.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/time_units.h>
#include <zephyr/sys/util.h>

#include "eth_dwmac_priv.h"
#include "eth.h"

#define RK3588_RGMII_DELAY_PS_DEFAULT CONFIG_ETH_RK3588_RGMII_DELAY_DEFAULT

#define RK3588_MDIO_BUSY_TIMEOUT_US 1000U

#define RK3588_MAC_ADDR_LEN 6U

#define RK3588_MDIO_GOC_SHIFT 2U
#define RK3588_MDIO_GOC_READ  (3U << RK3588_MDIO_GOC_SHIFT)
#define RK3588_MDIO_GOC_WRITE (1U << RK3588_MDIO_GOC_SHIFT)
#define RK3588_MDIO_C45E      BIT(1)
#define RK3588_MDIO_BUSY      BIT(0)
#define RK3588_MDIO_PA_SHIFT 21U
#define RK3588_MDIO_RDA_SHIFT 16U
#define RK3588_MDIO_CR_SHIFT  8U
#define RK3588_MDIO_CR_MASK   GENMASK(11, 8)

/* Empirically safe divisor for MDC when emac clock feeds DWMAC around 200 MHz */
#define RK3588_MDIO_CSR_250_300 (3U << RK3588_MDIO_CR_SHIFT)

/* Realtek RTL8211F identifiers (used for heuristics) */
#define RTL_OUI_MSB 0x1CU
#define RTL_OUI_LSB_MASK 0x0FFFU
#define RTL_PHY_MODEL_RTL8211F 0x0C63U
#define RTL_PHY_ID(oui_msb, id_lsb) (((uint32_t)(oui_msb) << 16) | (id_lsb))

struct rk3588_dwmac_clk {
	const struct device *dev;
	struct rockchip_clk_subsys subsys;
};

struct rk3588_dwmac_config {
	mem_addr_t mac_base;
	uint32_t irq;
	uint32_t irq_priority;

	struct rk3588_dwmac_clk clk_aclk;
	struct rk3588_dwmac_clk clk_pclk;
	struct rk3588_dwmac_clk clk_tx;
	struct rk3588_dwmac_clk clk_rx;

	struct reset_dt_spec reset;

	const struct pinctrl_dev_config *pincfg;

	bool has_mac_addr;
	uint8_t mac_addr[RK3588_MAC_ADDR_LEN];

	bool has_tx_delay;
	bool has_rx_delay;
	uint32_t tx_delay_ps;
	uint32_t rx_delay_ps;
	const char *phy_mode;
	uint8_t phy_addr;

	uint32_t dma_sysbus_mode;
};

struct rk3588_dwmac_priv {
	struct dwmac_priv dwmac;
	const struct device *dev;
	const struct rk3588_dwmac_config *cfg;

	struct k_mutex mdio_mutex;
	struct k_work_delayable link_check_work;

	bool link_checked_once;
	bool link_up;
	enum phy_link_speed link_speed;

	uint32_t phy_uid;

	struct dwmac_dma_desc *tx_desc_mem;
	struct dwmac_dma_desc *rx_desc_mem;
#ifdef CONFIG_MMU
	uintptr_t tx_desc_phys;
	uintptr_t rx_desc_phys;
#endif
};

static inline struct rk3588_dwmac_priv *to_rk3588_priv(struct dwmac_priv *p)
{
	return CONTAINER_OF(p, struct rk3588_dwmac_priv, dwmac);
}

static inline struct rk3588_dwmac_priv *rk3588_dwmac_get_priv(const struct device *dev)
{
	struct dwmac_priv *core = dev->data;

	return CONTAINER_OF(core, struct rk3588_dwmac_priv, dwmac);
}

static int rk3588_dwmac_clock_enable(const struct device *dev,
				     const struct rk3588_dwmac_clk *clk)
{
	if ((clk == NULL) || (clk->dev == NULL)) {
		return 0;
	}

	if (!device_is_ready(clk->dev)) {
		LOG_ERR("%s: clock controller not ready", dev->name);
		return -ENODEV;
	}

	return clock_control_on(clk->dev, (clock_control_subsys_t)&clk->subsys);
}

static int rk3588_dwmac_reset_deassert(const struct device *dev,
				       const struct reset_dt_spec *reset)
{
	if (!reset->dev) {
		return 0;
	}

	if (!device_is_ready(reset->dev)) {
		LOG_WRN("%s: reset controller not ready", dev->name);
		return -ENODEV;
	}

	return reset_line_deassert(reset);
}

static uint32_t rk3588_dwmac_prepare_mdio_addr(uint8_t phy, uint8_t reg)
{
	return ((uint32_t)phy << RK3588_MDIO_PA_SHIFT) |
	       ((uint32_t)reg << RK3588_MDIO_RDA_SHIFT) |
	       RK3588_MDIO_CSR_250_300;
}

static int rk3588_dwmac_mdio_wait_idle(struct dwmac_priv *p)
{
	k_timepoint_t timeout = sys_timepoint_calc(K_USEC(RK3588_MDIO_BUSY_TIMEOUT_US));

	while (REG_READ(MAC_MDIO_ADDRESS) & RK3588_MDIO_BUSY) {
		if (sys_timepoint_expired(timeout)) {
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int rk3588_dwmac_mdio_read(struct rk3588_dwmac_priv *priv,
				  uint8_t phy, uint8_t reg, uint16_t *value)
{
	struct dwmac_priv *p = &priv->dwmac;
	int ret;

	if (!value) {
		return -EINVAL;
	}

	k_mutex_lock(&priv->mdio_mutex, K_FOREVER);

	ret = rk3588_dwmac_mdio_wait_idle(p);
	if (ret != 0) {
		goto out;
	}

	REG_WRITE(MAC_MDIO_ADDRESS,
		  rk3588_dwmac_prepare_mdio_addr(phy, reg) | RK3588_MDIO_GOC_READ);
	REG_WRITE(MAC_MDIO_ADDRESS,
		  rk3588_dwmac_prepare_mdio_addr(phy, reg) |
		  RK3588_MDIO_GOC_READ | RK3588_MDIO_BUSY);

	ret = rk3588_dwmac_mdio_wait_idle(p);
	if (ret != 0) {
		goto out;
	}

	*value = (uint16_t)(REG_READ(MAC_MDIO_DATA) & 0xFFFF);

out:
	k_mutex_unlock(&priv->mdio_mutex);
	return ret;
}

static int rk3588_dwmac_mdio_write(struct rk3588_dwmac_priv *priv,
				   uint8_t phy, uint8_t reg, uint16_t value)
{
	struct dwmac_priv *p = &priv->dwmac;
	int ret;

	k_mutex_lock(&priv->mdio_mutex, K_FOREVER);

	ret = rk3588_dwmac_mdio_wait_idle(p);
	if (ret != 0) {
		goto out;
	}

	REG_WRITE(MAC_MDIO_DATA, value);
	REG_WRITE(MAC_MDIO_ADDRESS,
		  rk3588_dwmac_prepare_mdio_addr(phy, reg) | RK3588_MDIO_GOC_WRITE);
	REG_WRITE(MAC_MDIO_ADDRESS,
		  rk3588_dwmac_prepare_mdio_addr(phy, reg) |
		  RK3588_MDIO_GOC_WRITE | RK3588_MDIO_BUSY);

	ret = rk3588_dwmac_mdio_wait_idle(p);

out:
	k_mutex_unlock(&priv->mdio_mutex);
	return ret;
}

static void rk3588_dwmac_configure_mac_speed(struct rk3588_dwmac_priv *priv,
					     enum phy_link_speed speed,
					     bool full_duplex)
{
	struct dwmac_priv *p = &priv->dwmac;
	uint32_t mac_conf;

	mac_conf = REG_READ(MAC_CONF);
	mac_conf &= ~(MAC_CONF_PS | MAC_CONF_FES | MAC_CONF_DM);

	switch (speed) {
	case LINK_FULL_10BASE:
	case LINK_HALF_10BASE:
		mac_conf |= MAC_CONF_PS;
		if (full_duplex) {
			mac_conf |= MAC_CONF_DM;
		}
		break;
	case LINK_FULL_100BASE:
	case LINK_HALF_100BASE:
		mac_conf |= MAC_CONF_PS | MAC_CONF_FES;
		if (full_duplex) {
			mac_conf |= MAC_CONF_DM;
		}
		break;
	case LINK_FULL_1000BASE:
	case LINK_HALF_1000BASE:
	default:
		if (full_duplex || speed == LINK_FULL_1000BASE) {
			mac_conf |= MAC_CONF_DM;
		}
		break;
	}

	mac_conf |= MAC_CONF_CST | MAC_CONF_TE | MAC_CONF_RE;
	REG_WRITE(MAC_CONF, mac_conf);
}

static void rk3588_dwmac_update_link_state(struct rk3588_dwmac_priv *priv)
{
	const struct rk3588_dwmac_config *cfg = priv->cfg;
	struct dwmac_priv *p = &priv->dwmac;
	struct phy_link_state new_state = {
		.speed = LINK_FULL_1000BASE,
		.is_up = false,
	};
	bool full_duplex = true;
	int ret;
	uint16_t val;

	/* Read BMSR twice to clear latch */
	ret = rk3588_dwmac_mdio_read(priv, cfg->phy_addr, MII_BMSR, &val);
	if (ret == 0) {
		ret = rk3588_dwmac_mdio_read(priv, cfg->phy_addr, MII_BMSR, &val);
	}
	if (ret != 0) {
		LOG_DBG("%s: MDIO read failed (%d)", priv->dev->name, ret);
		goto commit;
	}

	if (val & MII_BMSR_LINK_STATUS) {
		new_state.is_up = true;
	}

	ret = rk3588_dwmac_mdio_read(priv, cfg->phy_addr, MII_BMCR, &val);
	if (ret == 0) {
		full_duplex = (val & MII_BMCR_DUPLEX_MODE) != 0U;

		switch (val & MII_BMCR_SPEED_MASK) {
		case MII_BMCR_SPEED_10:
			new_state.speed = full_duplex ?
					  LINK_FULL_10BASE : LINK_HALF_10BASE;
			break;
		case MII_BMCR_SPEED_100:
			new_state.speed = full_duplex ?
					  LINK_FULL_100BASE : LINK_HALF_100BASE;
			break;
		case MII_BMCR_SPEED_1000:
			new_state.speed = full_duplex ?
					  LINK_FULL_1000BASE : LINK_HALF_1000BASE;
			break;
		default:
			break;
		}
	}

commit:
	if (new_state.is_up != priv->link_up || new_state.speed != priv->link_speed ||
	    !priv->link_checked_once) {
		rk3588_dwmac_configure_mac_speed(priv, new_state.speed, full_duplex);

		if (new_state.is_up) {
			net_eth_carrier_on(p->iface);
		} else {
			net_eth_carrier_off(p->iface);
		}

		priv->link_speed = new_state.speed;
		priv->link_up = new_state.is_up;
		priv->link_checked_once = true;

		LOG_INF("%s: link %s (%s)",
			priv->dev->name,
			new_state.is_up ? "up" : "down",
			new_state.speed == LINK_FULL_1000BASE ? "1G" :
			new_state.speed == LINK_FULL_100BASE ? "100M" :
			new_state.speed == LINK_FULL_10BASE ? "10M" :
			new_state.speed == LINK_HALF_1000BASE ? "1G-HD" :
			new_state.speed == LINK_HALF_100BASE ? "100M-HD" :
			"10M-HD");
	}
}

static void rk3588_dwmac_link_check_work(struct k_work *work)
{
	struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
	struct rk3588_dwmac_priv *priv =
		CONTAINER_OF(dwork, struct rk3588_dwmac_priv, link_check_work);

	rk3588_dwmac_update_link_state(priv);

	k_work_schedule(&priv->link_check_work, K_SECONDS(1));
}

static void rk3588_dwmac_identify_phy(struct rk3588_dwmac_priv *priv)
{
	const struct rk3588_dwmac_config *cfg = priv->cfg;
	uint16_t id1, id2;
	int ret;

	ret = rk3588_dwmac_mdio_read(priv, cfg->phy_addr, MII_PHYID1R, &id1);
	if (ret != 0) {
		return;
	}

	ret = rk3588_dwmac_mdio_read(priv, cfg->phy_addr, MII_PHYID2R, &id2);
	if (ret != 0) {
		return;
	}

	priv->phy_uid = ((uint32_t)id1 << 16) | id2;
	LOG_INF("%s: external PHY ID 0x%08x", priv->dev->name, priv->phy_uid);
}

static void rk3588_dwmac_apply_delays(struct rk3588_dwmac_priv *priv)
{
	const struct rk3588_dwmac_config *cfg = priv->cfg;

	if (!cfg->has_tx_delay && !cfg->has_rx_delay) {
		return;
	}

	/* TODO: write GRF GMAC delay registers when syscon driver is available. */
	LOG_DBG("%s: requested RGMII delays tx=%u ps rx=%u ps (TODO apply to GRF)",
		priv->dev->name,
		cfg->has_tx_delay ? cfg->tx_delay_ps : RK3588_RGMII_DELAY_PS_DEFAULT,
		cfg->has_rx_delay ? cfg->rx_delay_ps : RK3588_RGMII_DELAY_PS_DEFAULT);
}

int dwmac_bus_init(struct dwmac_priv *p)
{
	struct rk3588_dwmac_priv *priv = to_rk3588_priv(p);
	const struct rk3588_dwmac_config *cfg = priv->cfg;
	int ret;

	p->base_addr = cfg->mac_base;

	ret = rk3588_dwmac_clock_enable(priv->dev, &cfg->clk_aclk);
	if (ret != 0) {
		LOG_ERR("%s: failed to enable aclk (%d)", priv->dev->name, ret);
		return ret;
	}

	ret = rk3588_dwmac_clock_enable(priv->dev, &cfg->clk_pclk);
	if (ret != 0) {
		LOG_ERR("%s: failed to enable pclk (%d)", priv->dev->name, ret);
		return ret;
	}

	ret = rk3588_dwmac_clock_enable(priv->dev, &cfg->clk_tx);
	if (ret != 0) {
		LOG_WRN("%s: tx clock enable failed (%d)", priv->dev->name, ret);
	}

	ret = rk3588_dwmac_clock_enable(priv->dev, &cfg->clk_rx);
	if (ret != 0) {
		LOG_WRN("%s: rx clock enable failed (%d)", priv->dev->name, ret);
	}

	ret = rk3588_dwmac_reset_deassert(priv->dev, &cfg->reset);
	if (ret != 0) {
		LOG_WRN("%s: reset deassert failed (%d)", priv->dev->name, ret);
	}

	if (cfg->pincfg) {
		ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
		if (ret != 0) {
			LOG_ERR("%s: pinctrl apply failed (%d)", priv->dev->name, ret);
			return ret;
		}
	}

	rk3588_dwmac_apply_delays(priv);
	rk3588_dwmac_identify_phy(priv);

	LOG_DBG("%s: phy-mode=%s phy-addr=%u",
		priv->dev->name, cfg->phy_mode,
		cfg->phy_addr);

	return 0;
}

void dwmac_platform_init(struct dwmac_priv *p)
{
	struct rk3588_dwmac_priv *priv = to_rk3588_priv(p);
	const struct rk3588_dwmac_config *cfg = priv->cfg;
	size_t tx_sz = CONFIG_DWMAC_NB_TX_DESCS * sizeof(struct dwmac_dma_desc);
	size_t rx_sz = CONFIG_DWMAC_NB_RX_DESCS * sizeof(struct dwmac_dma_desc);

	sys_cache_data_invd_range(priv->tx_desc_mem, tx_sz);
	sys_cache_data_invd_range(priv->rx_desc_mem, rx_sz);

	p->tx_descs = priv->tx_desc_mem;
	p->rx_descs = priv->rx_desc_mem;

#ifdef CONFIG_MMU
	priv->tx_desc_phys = k_mem_phys_addr(priv->tx_desc_mem);
	priv->rx_desc_phys = k_mem_phys_addr(priv->rx_desc_mem);
	p->tx_descs_phys = priv->tx_desc_phys;
	p->rx_descs_phys = priv->rx_desc_phys;
#endif

	if (cfg->has_mac_addr) {
		memcpy(p->mac_addr, cfg->mac_addr, RK3588_MAC_ADDR_LEN);
	} else {
		gen_random_mac(p->mac_addr, 0x6C, 0x14, 0x08);
	}

	REG_WRITE(MAC_CONF, MAC_CONF_PS | MAC_CONF_FES | MAC_CONF_DM);
	REG_WRITE(DMA_SYSBUS_MODE, cfg->dma_sysbus_mode);

	irq_connect_dynamic(cfg->irq, cfg->irq_priority, dwmac_isr, priv->dev, 0);
	irq_enable(cfg->irq);
}

static void rk3588_dwmac_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct rk3588_dwmac_priv *priv = rk3588_dwmac_get_priv(dev);

	if (dwmac_api.iface_api.init) {
		dwmac_api.iface_api.init(iface);
	}

	k_work_schedule(&priv->link_check_work, K_SECONDS(1));
}

static enum ethernet_hw_caps rk3588_dwmac_get_capabilities(const struct device *dev)
{
	return dwmac_api.get_capabilities(dev);
}

static int rk3588_dwmac_send(const struct device *dev, struct net_pkt *pkt)
{
	return dwmac_api.send(dev, pkt);
}

static int rk3588_dwmac_set_config(const struct device *dev,
				   enum ethernet_config_type type,
				   const struct ethernet_config *config)
{
	return dwmac_api.set_config(dev, type, config);
}

static int rk3588_dwmac_start(const struct device *dev)
{
	struct rk3588_dwmac_priv *priv = rk3588_dwmac_get_priv(dev);
	struct dwmac_priv *p = &priv->dwmac;
	uint32_t val;

	val = REG_READ(DMA_CHn_TX_CTRL(0));
	REG_WRITE(DMA_CHn_TX_CTRL(0), val | DMA_CHn_TX_CTRL_St);

	val = REG_READ(DMA_CHn_RX_CTRL(0));
	REG_WRITE(DMA_CHn_RX_CTRL(0), val | DMA_CHn_RX_CTRL_SR);

	val = REG_READ(MAC_CONF);
	val |= MAC_CONF_CST | MAC_CONF_TE | MAC_CONF_RE;
	REG_WRITE(MAC_CONF, val);

	k_work_schedule(&priv->link_check_work, K_SECONDS(1));

	return 0;
}

static int rk3588_dwmac_stop(const struct device *dev)
{
	struct rk3588_dwmac_priv *priv = rk3588_dwmac_get_priv(dev);
	struct dwmac_priv *p = &priv->dwmac;
	uint32_t val;

	k_work_cancel_delayable(&priv->link_check_work);

	val = REG_READ(DMA_CHn_TX_CTRL(0));
	REG_WRITE(DMA_CHn_TX_CTRL(0), val & ~DMA_CHn_TX_CTRL_St);

	val = REG_READ(DMA_CHn_RX_CTRL(0));
	REG_WRITE(DMA_CHn_RX_CTRL(0), val & ~DMA_CHn_RX_CTRL_SR);

	val = REG_READ(MAC_CONF);
	val &= ~(MAC_CONF_TE | MAC_CONF_RE);
	REG_WRITE(MAC_CONF, val);

	return 0;
}

static const struct device *rk3588_dwmac_get_phy(const struct device *dev)
{
	ARG_UNUSED(dev);
	return NULL;
}

static const struct ethernet_api rk3588_dwmac_api = {
	.iface_api.init = rk3588_dwmac_iface_init,
	.start = rk3588_dwmac_start,
	.stop = rk3588_dwmac_stop,
	.get_capabilities = rk3588_dwmac_get_capabilities,
	.set_config = rk3588_dwmac_set_config,
	.get_phy = rk3588_dwmac_get_phy,
	.send = rk3588_dwmac_send,
};

static int rk3588_dwmac_init(const struct device *dev)
{
	struct rk3588_dwmac_priv *priv = rk3588_dwmac_get_priv(dev);
	const struct rk3588_dwmac_config *cfg = dev->config;

	priv->dev = dev;
	priv->cfg = cfg;
	priv->link_up = false;
	priv->link_checked_once = false;
	priv->link_speed = LINK_FULL_1000BASE;
	priv->phy_uid = 0U;

	k_mutex_init(&priv->mdio_mutex);
	k_work_init_delayable(&priv->link_check_work, rk3588_dwmac_link_check_work);

	return dwmac_probe(dev);
}

#define RK3588_DWMAC_CLOCK_INIT(node_id, name)\
	{\
		.dev = COND_CODE_1(DT_CLOCKS_HAS_NAME(node_id, name),\
			(DEVICE_DT_GET(DT_CLOCKS_CTLR_BY_NAME(node_id, name))), \
			(NULL)),\
		.subsys = {\
			.id = COND_CODE_1(DT_CLOCKS_HAS_NAME(node_id, name),\
				(DT_CLOCKS_CELL_BY_NAME(node_id, name, id)),\
				(0U)),\
		},\
	}

#define RK3588_DWMAC_HAS_CLOCK_BY_NAME(node_id, name) \
	DT_NODE_HAS_PROP(node_id, clocks) && DT_CLOCKS_HAS_NAME(node_id, name)

#define RK3588_DWMAC_DMA_SYSBUS_MODE					\
	(DMA_SYSBUS_MODE_AAL | DMA_SYSBUS_MODE_FB |			\
	 DMA_SYSBUS_MODE_BLEN16 | DMA_SYSBUS_MODE_BLEN32 |		\
	 DMA_SYSBUS_MODE_BLEN64 | DMA_SYSBUS_MODE_BLEN128 |		\
	 DMA_SYSBUS_MODE_BLEN256)

#define RK3588_DWMAC_MAC_ADDR_INIT(inst)					\
	.has_mac_addr = DT_INST_NODE_HAS_PROP(inst, local_mac_address),	\
	.mac_addr = { DT_INST_PROP_OR(inst, local_mac_address, {0, 0, 0, 0, 0, 0}) }

#define RK3588_DWMAC_PHY_ADDR(inst) DT_INST_PROP_OR(inst, phy_addr, 1U)

#define RK3588_DWMAC_PINCTRL(inst)							\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, pinctrl_0),				\
		(PINCTRL_DT_INST_DEV_CONFIG_GET(inst)),				\
		(NULL))

#define RK3588_DWMAC_DEFINE(inst)							\
	PINCTRL_DT_INST_DEFINE(inst);						\
	static struct dwmac_dma_desc						\
		rk3588_dwmac_tx_desc_##inst[CONFIG_DWMAC_NB_TX_DESCS]		\
		__aligned(64);							\
	static struct dwmac_dma_desc						\
		rk3588_dwmac_rx_desc_##inst[CONFIG_DWMAC_NB_RX_DESCS]		\
		__aligned(64);							\
	static const struct rk3588_dwmac_config rk3588_dwmac_config_##inst = {	\
		.mac_base = DT_INST_REG_ADDR(inst),				\
		.irq = DT_INST_IRQN(inst),					\
		.irq_priority = DT_INST_IRQ(inst, priority),			\
		.clk_aclk = RK3588_DWMAC_CLOCK_INIT(DT_DRV_INST(inst), "aclk"),\
		.clk_pclk = RK3588_DWMAC_CLOCK_INIT(DT_DRV_INST(inst), "pclk"),\
		.clk_tx = RK3588_DWMAC_CLOCK_INIT(DT_DRV_INST(inst), "tx"),	\
		.clk_rx = RK3588_DWMAC_CLOCK_INIT(DT_DRV_INST(inst), "rx"),	\
		.reset = RESET_DT_SPEC_INST_GET_OR(inst, {0}),			\
		.pincfg = RK3588_DWMAC_PINCTRL(inst),				\
		.has_tx_delay = DT_INST_NODE_HAS_PROP(inst, rockchip_tx_delay_ps), \
		.has_rx_delay = DT_INST_NODE_HAS_PROP(inst, rockchip_rx_delay_ps), \
		.tx_delay_ps = DT_INST_PROP_OR(inst, rockchip_tx_delay_ps,	\
					       RK3588_RGMII_DELAY_PS_DEFAULT),	\
		.rx_delay_ps = DT_INST_PROP_OR(inst, rockchip_rx_delay_ps,	\
					       RK3588_RGMII_DELAY_PS_DEFAULT),	\
		.phy_mode = DT_INST_PROP_OR(inst, phy_connection_type, "rgmii"), \
		.phy_addr = RK3588_DWMAC_PHY_ADDR(inst),\
		.dma_sysbus_mode = RK3588_DWMAC_DMA_SYSBUS_MODE,		\
		RK3588_DWMAC_MAC_ADDR_INIT(inst),				\
	};									\
	static struct rk3588_dwmac_priv rk3588_dwmac_priv_##inst = {		\
		.tx_desc_mem = rk3588_dwmac_tx_desc_##inst,			\
		.rx_desc_mem = rk3588_dwmac_rx_desc_##inst,			\
	};									\
	ETH_NET_DEVICE_DT_INST_DEFINE(inst,					\
				      rk3588_dwmac_init,			\
				      NULL,					\
				      &rk3588_dwmac_priv_##inst.dwmac,		\
				      &rk3588_dwmac_config_##inst,		\
				      CONFIG_ETH_INIT_PRIORITY,			\
				      &rk3588_dwmac_api,			\
				      NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(RK3588_DWMAC_DEFINE)
