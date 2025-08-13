/* soc/rockchip/rk35/rk3588/include/pinctrl_soc.h */
#pragma once
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>

/* Tipo dei “pin descrittori” codificati dal DTS (RK_PINCFG(...)) */
typedef uint32_t pinctrl_soc_pin_t;

/* --- Se usi la codifica RK_PINCFG in DTS, definisci qui i decoder C --- */
/* Esempio di layout (ADATTA ai bit reali che usi nel tuo rk3588-pinctrl.h):
 * [31:28]=bank, [27:20]=pin, [19:16]=func, [15:12]=pull, [11:8]=drive, [0]=smt
 */
#define RK_PIN_BANK(v)   (((v) >> 28) & 0xF)
#define RK_PIN_NUM(v)    (((v) >> 20) & 0xFF)
#define RK_PIN_FUNC(v)   (((v) >> 16) & 0xF)
#define RK_PIN_PULL(v)   (((v) >> 12) & 0xF)
#define RK_PIN_DRIVE(v)  (((v) >> 8)  & 0xF)
#define RK_PIN_SMT(v)    ((v) & 0x1)

/* Descrizione delle basi IOC/GRF (se ti serve per l’implementazione reale) */
struct rk3588_ioc_desc {
	uintptr_t pmu0_grf, pmu1_grf, pmu2_grf, sys_grf;
	uintptr_t pmu0_ioc, pmu1_ioc, pmu2_ioc, bus_ioc;
};

/* Se vuoi: esponi una get delle basi (in futuro) */
// const struct rk3588_ioc_desc *rk3588_ioc(void);