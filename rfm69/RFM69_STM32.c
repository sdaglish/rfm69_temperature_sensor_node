#include "RFM69_STM32.h"
#include "stm32l010f4-gpio.h"
#include "stm32l010f4-spi.h"

void noInterrupts() {
  // HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
}
void interrupts() {
  //  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}
void RFM69_setCSPin(bool state) {
  //  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, (state == true) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  if (state == true) {
    gpio_set_pin_high(1, 1);
  }
  else {
    gpio_set_pin_low(1, 1);
  }
}
uint8_t SPI_transfer8Bit(uint8_t tx) {
  //   uint8_t inData;
  //  HAL_SPI_TransmitReceive(&hspi1, &tx, &inData, 1, 1000);
  //   return inData;
  //return 0;
  return spi_transfer(tx);
}
