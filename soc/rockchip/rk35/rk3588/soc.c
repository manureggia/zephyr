#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/mem_manage.h>
#include <zephyr/kernel/internal/mm.h>
#include <zephyr/sys/printk.h>

#include <zephyr/drivers/clock_control/rk3588_cru.h>

#ifndef RK_MMIO_FLAGS_DEVICE
#define RK_MMIO_FLAGS_DEVICE 0xA
#endif

mm_reg_t rk3588_cru_base;

static const uint32_t rk3588_pll_con1_offsets[] = {
	CRU_GPLL_CON1,
	CRU_CPLL_CON1,
	CRU_AUPLL_CON1,
	CRU_NPLL_CON1,
};

static int rk3588_cru_map(void)
{
	static bool mapped;
	uint8_t *va = NULL;

	if (mapped) {
		return 0;
	}

	k_mem_map_phys_bare(&va,
			      DT_REG_ADDR(DT_NODELABEL(cru)),
			      DT_REG_SIZE(DT_NODELABEL(cru)),
			      RK_MMIO_FLAGS_DEVICE);
	if (va == NULL) {
		return -ENOMEM;
	}

	rk3588_cru_base = (mm_reg_t)(uintptr_t)va;
	mapped = true;

	return 0;
}

#define CRU_GATE_CON_COUNT \
	(((CRU_GATE_CON_END - CRU_GATE_CON_START) / CRU_GATE_CON_STRIDE) + 1U)
#define CRU_SOFTRST_CON_COUNT \
	(((CRU_SOFTRST_CON_END - CRU_SOFTRST_CON_START) / CRU_SOFTRST_CON_STRIDE) + 1U)

static inline uint32_t cru_reg_read(uint32_t offset)
{
	return sys_read32(rk3588_cru_base + offset);
}

static void rk3588_capture_offsets(const uint32_t *offsets,
				      size_t count,
				      uint32_t *out)
{
	for (size_t i = 0; i < count; i++) {
		out[i] = cru_reg_read(offsets[i]);
	}
}

static void rk3588_capture_range(uint32_t start,
			       uint32_t stride,
			       size_t count,
			       uint32_t *out)
{
	uint32_t offset = start;
	for (size_t i = 0; i < count; i++, offset += stride) {
		out[i] = cru_reg_read(offset);
	}
}

static uint32_t pll_before[ARRAY_SIZE(rk3588_pll_con1_offsets)];
static uint32_t pll_after[ARRAY_SIZE(rk3588_pll_con1_offsets)];
static uint32_t gate_before[CRU_GATE_CON_COUNT];
static uint32_t gate_after[CRU_GATE_CON_COUNT];
static uint32_t reset_before[CRU_SOFTRST_CON_COUNT];
static uint32_t reset_after[CRU_SOFTRST_CON_COUNT];
static bool cru_dump_ready;

void rk3588_cru_init(void)
{
	static bool once;

	if (once) {
		return;
	}

	/*
	 * Do not touch CRU_GLB_SRST_* registers here: writing the boot ROM
	 * sequences causes a full chip reset on RK3588 and the system loops.
	 * We assume the boot firmware already performed the necessary POR flow.
	 */

	/* Fase B: porta i PLL principali fuori da reset */
	rk3588_capture_offsets(rk3588_pll_con1_offsets,
				 ARRAY_SIZE(rk3588_pll_con1_offsets),
				 pll_before);
	for (size_t i = 0U; i < ARRAY_SIZE(rk3588_pll_con1_offsets); i++) {
		CRU_REG_WRITE(rk3588_pll_con1_offsets[i], BIT(13), 0x0000U);
	}
	rk3588_capture_offsets(rk3588_pll_con1_offsets,
				 ARRAY_SIZE(rk3588_pll_con1_offsets),
				 pll_after);

	/* Fase C: abilita tutti i clock gating principali */
	rk3588_capture_range(CRU_GATE_CON_START,
			       CRU_GATE_CON_STRIDE,
			       CRU_GATE_CON_COUNT,
			       gate_before);
	for (uint32_t offset = CRU_GATE_CON_START; offset <= CRU_GATE_CON_END;
	     offset += CRU_GATE_CON_STRIDE) {
		CRU_REG_WRITE(offset, 0xFFFFU, 0x0000U);
	}
	rk3588_capture_range(CRU_GATE_CON_START,
			       CRU_GATE_CON_STRIDE,
			       CRU_GATE_CON_COUNT,
			       gate_after);

	/* Fase D: rilascia tutti i soft reset rilevanti */
	rk3588_capture_range(CRU_SOFTRST_CON_START,
			       CRU_SOFTRST_CON_STRIDE,
			       CRU_SOFTRST_CON_COUNT,
			       reset_before);
	for (uint32_t offset = CRU_SOFTRST_CON_START;
	     offset <= CRU_SOFTRST_CON_END;
	     offset += CRU_SOFTRST_CON_STRIDE) {
		/* write 0 (deassert) to every reset bit using hiword update */
		CRU_REG_WRITE(offset, 0xFFFFU, 0x0000U);
	}
	rk3588_capture_range(CRU_SOFTRST_CON_START,
			       CRU_SOFTRST_CON_STRIDE,
			       CRU_SOFTRST_CON_COUNT,
			       reset_after);

	once = true;
	cru_dump_ready = true;
}

static int rockchip_rk3588_init(void)
{
	int ret = rk3588_cru_map();
	if (ret < 0) {
		return ret;
	}

	rk3588_cru_init();

	return 0;
}
void rk3588_cru_dump_state(void)
{
	if (!cru_dump_ready) {
		printk("[CRU] dump requested before init has completed\n");
		return;
	}

	printk("[CRU] === PLL state ===\n");
	for (size_t i = 0; i < ARRAY_SIZE(rk3588_pll_con1_offsets); i++) {
		printk("[CRU] pll off=0x%04x before=0x%08x after=0x%08x\n",
		       rk3588_pll_con1_offsets[i],
		       pll_before[i],
		       pll_after[i]);
	}

	printk("[CRU] === Gate state ===\n");
	uint32_t offset = CRU_GATE_CON_START;
	for (size_t i = 0; i < CRU_GATE_CON_COUNT; i++, offset += CRU_GATE_CON_STRIDE) {
		printk("[CRU] gate off=0x%04x before=0x%08x after=0x%08x\n",
		       offset, gate_before[i], gate_after[i]);
	}

	printk("[CRU] === Reset state ===\n");
	offset = CRU_SOFTRST_CON_START;
	for (size_t i = 0; i < CRU_SOFTRST_CON_COUNT; i++, offset += CRU_SOFTRST_CON_STRIDE) {
		printk("[CRU] reset off=0x%04x before=0x%08x after=0x%08x\n",
		       offset, reset_before[i], reset_after[i]);
	}
}
SYS_INIT(rockchip_rk3588_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
