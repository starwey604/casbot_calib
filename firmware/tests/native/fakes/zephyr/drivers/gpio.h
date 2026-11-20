#ifndef FAKE_GPIO_H
#define FAKE_GPIO_H
#include "fake_zephyr.h"

struct gpio_dt_spec { int pin; };
#define GPIO_DT_SPEC_GET(node, prop) { .pin = 15 }
#define GPIO_OUTPUT_INACTIVE 0
static inline bool gpio_is_ready_dt(const struct gpio_dt_spec *led) { return led->pin == 15; }
static inline int gpio_pin_configure_dt(const struct gpio_dt_spec *led, int flags)
{ assert(led->pin == 15 && flags == GPIO_OUTPUT_INACTIVE); return 0; }
int gpio_pin_set_dt(const struct gpio_dt_spec *led, int value);
#endif
