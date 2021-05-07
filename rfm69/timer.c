#include "timer.h"
#include "libopencm3/stm32/l0/nvic.h"
#include <libopencm3/cm3/nvic.h>

static volatile uint32_t sys_tick_timer = 0;

uint32_t timer_get_sys_tick(void) {
  return sys_tick_timer;
}

void timer_setup(void) {
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB); // 2.097MHz (?)
  systick_set_reload((2097*1)-1); // 2.097MHz / 2097 = 1000 overflows per second - every 1ms one interrupt.
  systick_clear();
  systick_interrupt_enable();
  systick_counter_enable();
}

// ISR handler for systick overflow
void sys_tick_handler(void) {
  sys_tick_timer++;
}
