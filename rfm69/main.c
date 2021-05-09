#include "libopencm3/stm32/l0/memorymap.h"
#include "libopencm3/stm32/l0/rcc.h"
#include "libopencm3/stm32/l0/syscfg.h"
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/rcc.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "RFM69.h"
#include "gpio.h"
#include "i2c.h"
#include "mcp9808.h"
#include "spi.h"
#include "timer.h"

#define NODEID 10
#define NETWORKID 10
#define GATEWAYID 11
#define FREQUENCY RF69_433MHZ

static void clock_setup(void) {
  /* The clock setup below is a modification of the built-in
     rcc_clock_setup_pll as it doesn't allow use of the MSI
  */

  struct rcc_clock_scale clock = {
    .pll_mul = RCC_CFGR_PLLMUL_MUL3,
    .pll_div = RCC_CFGR_PLLDIV_DIV4,
    .pll_source = RCC_MSI,
    .voltage_scale = PWR_SCALE1,
    .hpre = RCC_CFGR_HPRE_NODIV,
    .ppre1 = RCC_CFGR_PPRE1_NODIV,
    .ppre2 = RCC_CFGR_PPRE2_NODIV,
    .msi_range = 5
  };

  rcc_set_msi_range(clock.msi_range);
  rcc_osc_on(RCC_MSI);
  rcc_wait_for_osc_ready(RCC_MSI);
  rcc_set_hpre(clock.hpre);
  rcc_set_ppre1(clock.ppre1);
  rcc_set_ppre2(clock.ppre2);
  rcc_periph_clock_enable(RCC_PWR);
  pwr_set_vos_scale(clock.voltage_scale);

  rcc_osc_off(RCC_PLL);
  while (rcc_is_osc_ready(RCC_PLL)) {}

  flash_prefetch_enable();
  flash_set_ws(clock.flash_waitstates);

  /* Set up the PLL */
  rcc_set_pll_multiplier(clock.pll_mul);
  rcc_set_pll_divider(clock.pll_div);
  rcc_set_pll_source(clock.pll_source);

  rcc_osc_on(RCC_MSI);
  rcc_wait_for_osc_ready(RCC_MSI);
  rcc_set_sysclk_source(RCC_MSI);

  /* Set the peripheral clock frequencies used. */
  // These currently aren't actually being used and could probably be removed without any issues.
  rcc_ahb_frequency = clock.ahb_frequency;
  rcc_apb1_frequency = clock.apb1_frequency;
  rcc_apb2_frequency = clock.apb2_frequency;

  // GPIO
  // rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_SYSCFG);

  // SPI
  rcc_periph_clock_enable(RCC_SPI1);

  // I2C
  rcc_set_peripheral_clk_sel(I2C1_BASE, 0b01);
  //RCC_CCIPR |= (1 << 12);
  rcc_periph_clock_enable(RCC_I2C1);
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
  uint8_t sendSize = 0; // strlen(payload);

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
