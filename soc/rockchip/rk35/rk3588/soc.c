#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>

static int rockchip_rk3588_init(void)
{
	/* In un porting completo, qui si configurano clock e power domain base.
	 * Per ora, la sua esistenza è sufficiente a soddisfare l'architettura.
	 */
	return 0;
}

SYS_INIT(rockchip_rk3588_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
