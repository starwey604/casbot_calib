#include "fake_zephyr.h"
#ifndef FAKE_DEVICE_H
#define FAKE_DEVICE_H
struct device { int ready; };
extern struct device fake_uart;
#define DT_NODELABEL(x) 0
#define DEVICE_DT_GET(x) (&fake_uart)
static inline bool device_is_ready(const struct device *dev){return dev->ready!=0;}
#endif
