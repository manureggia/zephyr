/*
 * File: zephyr/soc/rockchip/include/pinctrl_soc.h
 * Versione corretta per la nuova architettura pinctrl
 */
#ifndef ZEPHYR_SOC_ROCKCHIP_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ROCKCHIP_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Il tipo base per la configurazione di un pin è un intero a 32-bit */
typedef uint32_t pinctrl_soc_pin_t;

/*
 * Macro per inizializzare un singolo pin dall'array 'rockchip,pins' nel DTS.
 * Semplicemente prende il valore intero dall'array alla posizione 'idx'.
 */
#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx) \
	DT_PROP_BY_IDX(node_id, prop, idx),

/*
 * Macro per creare un array di inizializzatori C
 * iterando su tutti gli elementi della proprietà 'rockchip,pins'.
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)			\
	{DT_FOREACH_PROP_ELEM_SEP(node_id, prop, Z_PINCTRL_STATE_PIN_INIT, (,))}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_ROCKCHIP_PINCTRL_SOC_H_ */