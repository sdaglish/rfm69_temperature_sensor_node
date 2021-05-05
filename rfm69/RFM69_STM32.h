#ifndef RFM69_STM32_H
#define RFM69_STM32_H

//#include "main.h"
#include<stdint.h>
#include<stdbool.h>

#include "stm32l010f4-gpio.h"
#include "stm32l010f4-spi.h"

void noInterrupts();
void interrupts();
void RFM69_setCSPin(bool state);
uint8_t SPI_transfer8Bit(uint8_t tx); 



#endif
