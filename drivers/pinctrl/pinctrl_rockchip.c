/*
 * File: zephyr/drivers/pinctrl/pinctrl_rockchip.c
 * AGGIUNTA LA FUNZIONE WRAPPER MANCANTE
 */
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>

LOG_MODULE_REGISTER(pinctrl_rockchip, CONFIG_PINCTRL_LOG_LEVEL);

#define RK_PIN_BANK_GET(cfg)     (((cfg) >> 20) & 0x7)
#define RK_PIN_BANK_PIN_GET(cfg) (((cfg) >> 16) & 0x1F)
#define RK_PIN_FUNC_GET(cfg)     (((cfg) >> 0) & 0x7)
#define RK_PIN_PULL_GET(cfg)     (((cfg) >> 4) & 0x7)
#define RK_PIN_DRIVE_GET(cfg)    (((cfg) >> 8) & 0xF)

#define NODE_ID DT_NODELABEL(pinctrl)
static const uintptr_t grf_base[] = {
	DT_REG_ADDR_BY_NAME(NODE_ID, pmu0_grf),
	DT_REG_ADDR_BY_NAME(NODE_ID, pmu1_grf),
	DT_REG_ADDR_BY_NAME(NODE_ID, pmu2_grf),
	DT_REG_ADDR_BY_NAME(NODE_ID, sys_grf),
};

static uintptr_t get_grf_base_for_bank(uint32_t bank)
{
	switch (bank) {
	case 0: case 1: return grf_base[1]; /* pmu1_grf */
	case 2: return grf_base[3]; /* sys_grf */
	case 3: return grf_base[0]; /* pmu0_grf */
	case 4: return grf_base[2]; /* pmu2_grf */
	default: return 0;
	}
}

/* Funzione specifica del SoC (implementazione vera e propria) */
int pinctrl_soc_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);
	uintptr_t grf;

	for (uint8_t i = 0; i < pin_cnt; i++) {
		pinctrl_soc_pin_t pincfg = pins[i];
		uint32_t bank = RK_PIN_BANK_GET(pincfg);
		uint32_t pin = RK_PIN_BANK_PIN_GET(pincfg);
		uint32_t func = RK_PIN_FUNC_GET(pincfg);
		uint32_t pull = RK_PIN_PULL_GET(pincfg);

		grf = get_grf_base_for_bank(bank);
		if (!grf) {
			LOG_ERR("Banco %u non valido", bank);
			return -EINVAL;
		}

		uint32_t mux_offset = (bank * 0x10) + ((pin / 4) * 4);
		uint32_t mux_shift = (pin % 4) * 4;
		sys_write32((0xFU << (mux_shift + 16)) | (func << mux_shift),
			    grf + mux_offset);

		uint32_t pull_offset = (bank * 0x10) + ((pin / 8) * 4);
		uint32_t pull_shift = (pin % 8) * 2;
		sys_write32((0x3U << (pull_shift + 16)) | (pull << pull_shift),
			    grf + 0x40 + pull_offset);
	}

	return 0;
}

/*
 * NUOVA FUNZIONE WRAPPER:
 * Forniamo l'implementazione di pinctrl_configure_pins che il linker sta cercando.
 * Questa funzione chiama semplicemente la nostra implementazione specifica.
 */
int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	return pinctrl_soc_configure_pins(pins, pin_cnt, reg);
}