#include "spi.h"

void spi_setup(void) {
  // mosi, miso, sck, css = a7, a6, a5, a4 (respectively)
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7 | GPIO5 | GPIO6);
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO4);

  gpio_set_af(GPIOA, GPIO_AF0, GPIO5 | GPIO6 | GPIO7);

  spi_reset(SPI1);
  spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_16, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

  spi_enable_ss_output(SPI1);
  gpio_set(GPIOA, GPIO4);

  spi_enable(SPI1);
}

uint8_t SPI_transfer8Bit(uint8_t tx) {
  return spi_xfer(SPI1, tx);
}

uint8_t spi_transfer(uint8_t tx) { return spi_xfer(SPI1, tx); }


