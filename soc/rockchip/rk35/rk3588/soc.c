#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/sys/util.h>

#include <zephyr/drivers/clock_control/rk3588_cru.h>

void rk3588_cru_init(void)
{
	/* Fase A: Sequenza di reset globale */
	CRU_REG_WRITE(CRU_GLB_SRST_FST_VALUE, 0xFFFFU, 0xFDB9U);
	CRU_REG_WRITE(CRU_GLB_SRST_SND_VALUE, 0xFFFFU, 0xECA8U);
	CRU_REG_WRITE(CRU_SOFTRST_CON05, BIT(0), BIT(0));

	/* Fase B: porta i PLL principali fuori da reset */
	static const uint32_t pll_con1_offsets[] = {
		CRU_GPLL_CON1,
		CRU_CPLL_CON1,
		CRU_AUPLL_CON1,
		CRU_NPLL_CON1,
	};

	for (size_t i = 0U; i < ARRAY_SIZE(pll_con1_offsets); i++) {
		CRU_REG_WRITE(pll_con1_offsets[i], BIT(13), 0x0000U);
	}

	/* Fase C: abilita tutti i clock gating principali */
	for (uint32_t offset = CRU_GATE_CON_START; offset <= CRU_GATE_CON_END;
	     offset += CRU_GATE_CON_STRIDE) {
		CRU_REG_WRITE(offset, 0xFFFFU, 0x0000U);
	}

	/* Fase D: rilascia tutti i soft reset rilevanti */
	for (uint32_t offset = CRU_SOFTRST_CON_START;
	     offset <= CRU_SOFTRST_CON_END;
	     offset += CRU_SOFTRST_CON_STRIDE) {
		CRU_REG_WRITE(offset, 0xFFFFU, 0xFFFFU);
	}
}

static int rockchip_rk3588_init(void)
{
	rk3588_cru_init();

	return 0;
}

SYS_INIT(rockchip_rk3588_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
//TODO: veriica il PRE_KERNEL secondo me crasha
