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
#include "utils.h"

// TODO: Place these in a seperate header file that will contain all the
// information about the network.
#define NODEID 10
#define NETWORKID 10
#define GATEWAYID 11
#define FREQUENCY RF69_433MHZ

static void clock_setup(void) {
  /* The clock setup below is a modification of the built-in
     rcc_clock_setup_pll as it doesn't allow use of the MSI
  */

  struct rcc_clock_scale clock = {.pll_mul = RCC_CFGR_PLLMUL_MUL3,
				  .pll_div = RCC_CFGR_PLLDIV_DIV4,
				  .pll_source = RCC_MSI,
				  .voltage_scale = PWR_SCALE1,
				  .hpre = RCC_CFGR_HPRE_NODIV,
				  .ppre1 = RCC_CFGR_PPRE1_NODIV,
				  .ppre2 = RCC_CFGR_PPRE2_NODIV,
				  .msi_range = 5};

  rcc_set_msi_range(clock.msi_range);
  rcc_osc_on(RCC_MSI);
  rcc_wait_for_osc_ready(RCC_MSI);
  rcc_set_hpre(clock.hpre);
  rcc_set_ppre1(clock.ppre1);
  rcc_set_ppre2(clock.ppre2);
  rcc_periph_clock_enable(RCC_PWR);
  pwr_set_vos_scale(clock.voltage_scale);

  rcc_osc_off(RCC_PLL);
  while (rcc_is_osc_ready(RCC_PLL)) {
  }

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
  // These currently aren't actually being used and could probably be removed
  // without any issues.
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
  // RCC_CCIPR |= (1 << 12);
  rcc_periph_clock_enable(RCC_I2C1);
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
