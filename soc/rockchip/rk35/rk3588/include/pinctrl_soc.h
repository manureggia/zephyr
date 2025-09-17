/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <stdint.h>
#include <zephyr/devicetree.h>

/* Rappresentiamo un pin muxato come un u32 (valore DT "pinmux = <...>") */
typedef uint32_t pinctrl_soc_pin_t;

/* Per ogni phandle in pinctrl-<state>, espandi gli elementi del 'pinmux' del relativo gruppo */
#define Z_PINCTRL_STATE_PIN_INIT(group_id, pin_prop, idx) \
    DT_PROP_BY_IDX(group_id, pin_prop, idx),

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop) \
    { DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop), \
                             DT_FOREACH_PROP_ELEM, pinmux, \
                             Z_PINCTRL_STATE_PIN_INIT) }
