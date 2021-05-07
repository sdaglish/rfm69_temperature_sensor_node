#ifndef TIMER_H
#define TIMER_H

#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/cm3/systick.h>
#include <stdint.h>

uint32_t timer_get_sys_tick(void);
void timer_setup(void);

#endif
