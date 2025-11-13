// SPDX-License-Identifier: Apache-2.0

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/fatal.h>

LOG_MODULE_REGISTER(pi5_plus_fatal, CONFIG_LOG_DEFAULT_LEVEL);

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	LOG_ERR("Fatal error: reason=%u esf=%p", reason, esf);
	k_fatal_halt(reason);
}
