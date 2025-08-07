#ifndef ZEPHYR_DRIVERS_CLOCK_CONTROL_ROCKCHIP_CLOCK_CONTROL_H_
#define ZEPHYR_DRIVERS_CLOCK_CONTROL_ROCKCHIP_CLOCK_CONTROL_H_

#include <zephyr/drivers/clock_control.h>

struct rockchip_clk_subsys {
	uint32_t id; /* ID del clock, tipicamente un intero o enum */
};

#endif /* ZEPHYR_DRIVERS_CLOCK_CONTROL_ROCKCHIP_CLOCK_CONTROL_H_ */