#include "i2c.h"

void i2c_setup(void) {
  i2c_reset(I2C1);
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
  gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_HIGH, GPIO9 | GPIO10);
  gpio_set_af(GPIOA, GPIO_AF1, GPIO9 | GPIO10);

  i2c_reset(I2C1);
  i2c_peripheral_disable(I2C1);
  
  i2c_set_speed(I2C1, i2c_speed_sm_100k, 2);  // Actually 2.097MHz, that that doesn't seem to be an option
  i2c_enable_stretching(I2C1);
  i2c_set_7bit_addr_mode(I2C1);
  i2c_peripheral_enable(I2C1);
}

