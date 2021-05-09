#include "i2c.h"

void i2c_setup(void) {
  // i2c_reset(I2C1);
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
  gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_HIGH, GPIO9 | GPIO10);
  gpio_set_af(GPIOA, GPIO_AF1, GPIO9 | GPIO10);

  //  i2c_reset(I2C1);
  //i2c_peripheral_disable(I2C1);
  
  // The high and low values below had to be hand adjusted to make them work
  // TODO: I'd like to be able to calculate these values better!
  //  i2c_set_speed(I2C1, i2c_speed_sm_100k, 2);  // Actually 2.097MHz, that that doesn't seem to be an option
  i2c_set_prescaler(I2C1, 0);    // 2.097MHz = 476.87ns
  i2c_set_scl_low_period(I2C1, 7); // 5.0us / 476.87ns = 10.49 ==> 10*476.87ns = 4.77us
  i2c_set_scl_high_period(I2C1, 6); // 4.0us / 476.87ns = 8.39 ==> 8*476.87ns = 3.81us
  //i2c_set_scl_low_period(I2C1, 9); // 5.0us / 476.87ns = 10.49 ==> 10*476.87ns = 4.77us
  //i2c_set_scl_high_period(I2C1, 7); // 4.0us / 476.87ns = 8.39 ==> 8*476.87ns = 3.81us
  // tSCL ~= 10us ==> 4.77us + 3.81us = 8.58us ==> 8.58us - 10us == 1.42us
  //i2c_set_data_hold_time(I2C1, 1); //  500ns / 476.87ns = 1.05 ==> 1*476.87ns = 476.87ns
  //i2c_set_data_setup_time(I2C1, 2); //  500ns / 476.87ns = 1.05 ==> 1*476.87ns = 476.87ns
  i2c_enable_stretching(I2C1);  // 1250ns / 476.87ns = 2.62
  i2c_set_7bit_addr_mode(I2C1);
  i2c_peripheral_enable(I2C1);
}

