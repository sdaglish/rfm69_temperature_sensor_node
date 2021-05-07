#ifndef SPI_H
#define SPI_H

#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>

void spi_setup(void);
uint8_t SPI_transfer8Bit(uint8_t tx);
uint8_t spi_transfer(uint8_t tx);

#endif
