#include "libopencm3/stm32/l0/rcc.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/i2c.h>
#include "libopencm3/stm32/l0/syscfg.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "spi.h"
#include "gpio.h"
#include "RFM69.h"
#include "mcp9808.h"
#include "timer.h"

#define NODEID    10
#define NETWORKID 10
#define GATEWAYID 11
#define FREQUENCY RF69_433MHZ



static void clock_setup(void) {
  rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
  rcc_set_ppre1(RCC_CFGR_PPRE1_NODIV);
  rcc_set_ppre2(RCC_CFGR_PPRE2_NODIV);
  
  // GPIO 
  // rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_SYSCFG);

  // SPI
  rcc_periph_clock_enable(RCC_SPI1);

  // I2C
  rcc_periph_clock_enable(RCC_I2C1);
}




static void i2c_setup(void) {
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

static char *itoa(int value, char *result, int base) {
  // check that the base if valid
  if (base < 2 || base > 36) {
    *result = '\0';
    return result;
  }

  char *ptr = result, *ptr1 = result, tmp_char;
  int tmp_value;

  do {
    tmp_value = value;
    value /= base;
    *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrst"
	     "uvwxyz"[35 + (tmp_value - value * base)];
  } while (value);

  // Apply negative sign
  if (tmp_value < 0)
    *ptr++ = '-';
  *ptr-- = '\0';
  while (ptr1 < ptr) {
    tmp_char = *ptr;
    *ptr-- = *ptr1;
    *ptr1++ = tmp_char;
  }
  return result;
}

static void ftoa(float Value, char *Buffer) {
  union {
    float f;

    struct {
      unsigned int mantissa_lo : 16;
      unsigned int mantissa_hi : 7;
      unsigned int exponent : 8;
      unsigned int sign : 1;
    };
  } helper;

  unsigned long mantissa;
  signed char exponent;
  unsigned int int_part;
  char frac_part[3];
  int i, count = 0;

  helper.f = Value;
  // mantissa is LS 23 bits
  mantissa = helper.mantissa_lo;
  mantissa += ((unsigned long)helper.mantissa_hi << 16);
  // add the 24th bit to get 1.mmmm^eeee format
  mantissa += 0x00800000;
  // exponent is biased by 127
  exponent = (signed char)helper.exponent - 127;

  // too big to shove into 8 chars
  if (exponent > 18) {
    Buffer[0] = 'I';
    Buffer[1] = 'n';
    Buffer[2] = 'f';
    Buffer[3] = '\0';
    return;
  }

  // too small to resolve (resolution of 1/8)
  if (exponent < -3) {
    Buffer[0] = '0';
    Buffer[1] = '\0';
    return;
  }

  count = 0;

  // add negative sign (if applicable)
  if (helper.sign) {
    Buffer[0] = '-';
    count++;
  }

  // get the integer part
  int_part = mantissa >> (23 - exponent);
  // convert to string
  itoa(int_part, &Buffer[count], 10);

  // find the end of the integer
  for (i = 0; i < 8; i++)
    if (Buffer[i] == '\0') {
      count = i;
      break;
    }

  // not enough room in the buffer for the frac part
  if (count > 5)
    return;

  // add the decimal point
  Buffer[count++] = '.';

  // use switch to resolve the fractional part
  switch (0x7 & (mantissa >> (20 - exponent))) {
  case 0:
    frac_part[0] = '0';
    frac_part[1] = '0';
    frac_part[2] = '0';
    break;
  case 1:
    frac_part[0] = '1';
    frac_part[1] = '2';
    frac_part[2] = '5';
    break;
  case 2:
    frac_part[0] = '2';
    frac_part[1] = '5';
    frac_part[2] = '0';
    break;
  case 3:
    frac_part[0] = '3';
    frac_part[1] = '7';
    frac_part[2] = '5';
    break;
  case 4:
    frac_part[0] = '5';
    frac_part[1] = '0';
    frac_part[2] = '0';
    break;
  case 5:
    frac_part[0] = '6';
    frac_part[1] = '2';
    frac_part[2] = '5';
    break;
  case 6:
    frac_part[0] = '7';
    frac_part[1] = '5';
    frac_part[2] = '0';
    break;
  case 7:
    frac_part[0] = '8';
    frac_part[1] = '7';
    frac_part[2] = '5';
    break;
  }

  // add the fractional part to the output string
  for (i = 0; i < 3; i++)
    if (count < 7)
      Buffer[count++] = frac_part[i];

  // make sure the output is terminated
  Buffer[count] = '\0';
}

int main(void) {
  clock_setup();
  timer_setup();
  spi_setup();
  gpio_setup();
  i2c_setup();
  
  rfm69_init(RF69_433MHZ, NODEID, NETWORKID);
  RFM69_enableAutoPower(-80);

  mcp9808_init();

  char payload[16];
  uint8_t sendSize = 0;//strlen(payload);

  while (1) {
    uint16_t tempValue = mcp9808_readTempReg();
    float tempFloat = tempValue & 0x0FFF;
    tempFloat /= 16.0;

    uint8_t powerLevel = RFM69_getPowerLevel();
    char powerLevelBuff[16];
    itoa(powerLevel, powerLevelBuff, 10);
    uint8_t powerLevelBuffSize = strlen(powerLevelBuff);

    ftoa(tempFloat, payload);
    sendSize = strlen(payload);

    payload[sendSize - 1] = ' ';
    for (int i = 0; i < powerLevelBuffSize; i++) {
      payload[(sendSize) + i] = powerLevelBuff[i];
    }

    sendSize += (powerLevelBuffSize + 1);
    payload[sendSize - 1] = '\0';
    rfm69_sendWithRetry(GATEWAYID, payload, sendSize, 20, 200);

    uint32_t now = timer_get_sys_tick();
    while ((timer_get_sys_tick() - now) < 1000) {
    }
  }
}
