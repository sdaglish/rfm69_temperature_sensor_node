#include "gpio.h"

void gpio_setup(void) {
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO3);
  
  nvic_enable_irq(NVIC_EXTI2_3_IRQ);
  exti_select_source(EXTI3, GPIOA);
  exti_set_trigger(EXTI3, EXTI_TRIGGER_RISING);
  exti_enable_request(EXTI3);
  nvic_clear_pending_irq(NVIC_EXTI2_3_IRQ);
  nvic_set_priority(NVIC_EXTI2_3_IRQ, 2);
}

void exti2_3_isr() {
  exti_reset_request(EXTI3);
  isr0();
}

void RFM69_setCSPin(bool state) {
  if (state == true) {
    gpio_set(GPIOA, GPIO4);
  }
  else {
    gpio_clear(GPIOA, GPIO4);
  }
}

void noInterrupts(void) {
   nvic_disable_irq(NVIC_EXTI2_3_IRQ);
}

void interrupts(void) {
  nvic_enable_irq(NVIC_EXTI2_3_IRQ);
}
