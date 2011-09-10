#ifndef _SEMC_GPIO_EVENT_H_
#define _SEMC_GPIO_EVENT_H_

#include <linux/gpio_event.h>
#include <linux/input.h>

int semc_gpio_event_matrix_func(struct gpio_event_input_devs *input_dev,
	struct gpio_event_info *info, void **data, int func);

int semc_gpio_event_input_func(struct gpio_event_input_devs *input_dev,
	struct gpio_event_info *info, void **data, int func);

#endif /*_SEMC_GPIO_EVENT_H_*/
