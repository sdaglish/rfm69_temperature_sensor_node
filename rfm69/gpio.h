#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include <stdbool.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/l0/nvic.h>
#include <libopencm3/cm3/nvic.h>

#include "RFM69.h"

void gpio_setup(void);
void RFM69_setCSPin(bool state);

#endif
