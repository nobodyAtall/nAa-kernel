#ifndef _SEMC_POWER_PLATFORM_H_
#define _SEMC_POWER_PLATFORM_H_

#include <linux/semc/power/semc_power.h>

#define SEMC_POWER_PLATFORM_NAME "semc_pwr_pltf"

int setup_platform(struct power_ops *platform_ops, event_callback_t fn);
int teardown_platform(void);

#endif /* _SEMC_POWER_PLATFORM_H_ */
