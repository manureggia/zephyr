/*
 * Copyright (c) 2023, The Zephyr Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_DRIVERS_CLOCK_CONTROL_RK3588_CRU_PUBLIC_H_
#define ZEPHYR_DRIVERS_CLOCK_CONTROL_RK3588_CRU_PUBLIC_H_

#include <stdint.h>

#include <zephyr/devicetree.h>
#include <zephyr/dt-bindings/clock/rk3588-cru.h>
#include <zephyr/sys/sys_io.h>

extern mm_reg_t rk3588_cru_base;

#define CRU_REG_WRITE(offset, we_mask, data) \
	sys_write32((((uint32_t)(we_mask) << 16) | (uint32_t)(data)), \
		    rk3588_cru_base + (offset))

#define CRU_GLB_SRST_FST_VALUE	0x0C08U
#define CRU_GLB_SRST_SND_VALUE	0x0C0CU

#define CRU_GPLL_CON1		0x01C4U
#define CRU_CPLL_CON1		0x01A4U
#define CRU_AUPLL_CON1		0x0184U
#define CRU_NPLL_CON1		0x01E4U

#define CRU_GATE_CON_START	0x0800U
#define CRU_GATE_CON_END	0x0934U
#define CRU_GATE_CON_STRIDE	0x4U

#define CRU_SOFTRST_CON_START	0x0A00U
#define CRU_SOFTRST_CON_END	0x0B34U
#define CRU_SOFTRST_CON_STRIDE	0x4U

#define CRU_SOFTRST_CON05	0x0A14U

void rk3588_cru_init(void);
void rk3588_cru_dump_state(void);



#define HIWORD_UPDATE(val, mask, shift)	(((uint32_t)(val) & (mask)) << (shift) | ((mask) << ((shift) + 16)))
#define HIWORD_MASK(mask, shift)	((mask) << ((shift) + 16))

/* Sezione PLL */
#define RK3588_PLL_CON(pll_id, i)	(0x0000 + (pll_id) * 0x10 + (i) * 4)

/* Sezione clock gate */
#define RK3588_CLKGATE_CON(i)		(0x0300 + (i) * 4)

/* Selettori e divisori del clock */
#define RK3588_BIGCORE0_CLKSEL_CON(n)	(0x0500 + (n) * 4)
#define RK3588_BIGCORE1_CLKSEL_CON(n)	(0x0580 + (n) * 4)
#define RK3588_DSU_CLKSEL_CON(n)	(0x0600 + (n) * 4)

/* Sezione soft-reset (SOFTRST_CONxx) */
#define RK3588_SOFTRST_CON(i)		(0x0A00 + (i) * 4)

static inline uint32_t rk3588_reset_reg_idx(uint32_t reset_id)
{
	return reset_id / 16U;
}

static inline uint32_t rk3588_reset_bit(uint32_t reset_id)
{
	return reset_id % 16U;
}

static inline uint32_t rk3588_reset_reg_off(uint32_t reset_id)
{
	return RK3588_SOFTRST_CON(rk3588_reset_reg_idx(reset_id));
}

/* --- Campi ricavati dal driver Linux --- */

/* CLK_CORE_B0_SEL_CLEAN */
#define CLK_CORE_B0_SEL_CLEAN_SHIFT	13
#define CLK_CORE_B0_SEL_CLEAN_MASK	0x3

/* CLK_CORE_B1_SEL_CLEAN */
#define CLK_CORE_B1_SEL_CLEAN_SHIFT	5
#define CLK_CORE_B1_SEL_CLEAN_MASK	0x3

/* CLK_CORE_B0_GPLL_DIV */
#define CLK_CORE_B0_GPLL_DIV_SHIFT	1
#define CLK_CORE_B0_GPLL_DIV_MASK	0x1F

/* CLK_DSU_DF_SRC */
#define CLK_DSU_DF_SRC_SHIFT		12
#define CLK_DSU_DF_SRC_MASK		0x3

/* CLK_DSU_DF_DIV */
#define CLK_DSU_DF_DIV_SHIFT		7
#define CLK_DSU_DF_DIV_MASK		0x1F

#endif /* ZEPHYR_DRIVERS_CLOCK_CONTROL_RK3588_CRU_PUBLIC_H_ */
