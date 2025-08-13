/* SPDX-License-Identifier: Apache-2.0 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>

#include <pinctrl_soc.h>

/*
 * Implementazione richiesta dall’infrastruttura pinctrl “nuova” di Zephyr.
 * La versione generica di Zephyr fornisce pinctrl_configure_pins(...)
 * che chiama questa per ogni pin.
 */
int pinctrl_configure_pin(pinctrl_soc_pin_t v, uintptr_t port)
{
	/* TODO: estrai i campi dal valore 'v' con i macro del tuo pinctrl_soc.h
	 * (es. banca, numero pin, funzione, pull, drive, ecc.)
	 *
	 * Esempio (inventato): 
	 *  unsigned bank = RK_PIN_BANK(v);
	 *  unsigned pin  = RK_PIN_NUM(v);
	 *  unsigned func = RK_PIN_FUNC(v);
	 *  unsigned pull = RK_PIN_PULL(v);
	 *  void *base = (void *)port; // cookie passato da Zephyr, tipicamente base regs
	 *  programma i registri partendo da base + offset(bank, pin) ...
	 */

	ARG_UNUSED(v);
	ARG_UNUSED(port);

	/* Ritorna 0 se ok, oppure -errno */
	return 0;
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			   uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		pinctrl_configure_pin(pins, 0/*TODO: MODIFICARE*/);
	}

	return 0;
}