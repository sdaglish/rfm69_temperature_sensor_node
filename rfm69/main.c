#include "libopencm3/stm32/l0/nvic.h"
#include "libopencm3/stm32/l0/rcc.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/syscfg.h>

#include <string.h>
#include <stdlib.h>

#include "libopencm3/stm32/l0/syscfg.h"
#include "spi.h"
#include "gpio.h"
#include "RFM69.h"
#include "mcp9808.h"

#define NODEID    10
#define NETWORKID 10
#define GATEWAYID 11
#define FREQUENCY RF69_433MHZ

static volatile uint32_t sys_tick_timer = 0;

extern uint8_t SPI_transfer8Bit(uint8_t tx) {
  return spi_xfer(SPI1, tx);
}

extern uint8_t spi_transfer(uint8_t tx) { return spi_xfer(SPI1, tx); }

extern uint32_t HAL_GetTick(void) {
  return sys_tick_timer;
}

extern void noInterrupts(void) {
   nvic_disable_irq(NVIC_EXTI2_3_IRQ);
  // exti_disable_request(EXTI3);
}

extern void interrupts(void) {
  //gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO3);
  //SYSCFG->EXTICR[0] &= (uint16_t)~SYSCFG_EXTICR1_EXTI0_PA; /* (3) */
  //SYSCFG_EXTICR(0) &= 0b1111; 

  // exti_select_source(EXTI3, GPIOA);
  //exti_set_trigger(EXTI3, EXTI_TRIGGER_RISING);
  //exti_enable_request(EXTI3);

  //nvic_clear_pending_irq(NVIC_EXTI2_3_IRQ);

  nvic_enable_irq(NVIC_EXTI2_3_IRQ);
  //nvic_set_priority(NVIC_EXTI2_3_IRQ, 2);
}

extern void RFM69_setCSPin(bool state) {
  if (state == true) {
    gpio_set(GPIOA, GPIO4);
  }
  else {
    gpio_clear(GPIOA, GPIO4);
  }
}


void sys_tick_handler() {
  sys_tick_timer++;
}

static void clock_setup(void) {
  rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
  rcc_set_ppre1(RCC_CFGR_PPRE1_NODIV);
  rcc_set_ppre2(RCC_CFGR_PPRE2_NODIV);
  
  // GPIO 
  // rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_SYSCFG);
  

  // Systick
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB); // 2.097MHz (?)
  systick_set_reload((2097*1)-1); // 2.097MHz / 2097 = 1000 overflows per second - every 1ms one interrupt.
  systick_clear();
  systick_interrupt_enable();
  systick_counter_enable();

  // SPI
  rcc_periph_clock_enable(RCC_SPI1);

  // I2C
  rcc_periph_clock_enable(RCC_I2C1);

  
}

static void spi_setup(void) {
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

static void gpio_setup(void) {
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO3);
  
  nvic_enable_irq(NVIC_EXTI2_3_IRQ);
  exti_select_source(EXTI3, GPIOA);
  exti_set_trigger(EXTI3, EXTI_TRIGGER_RISING);
   exti_enable_request(EXTI3);
  //exti_disable_request(EXTI3);
  nvic_clear_pending_irq(NVIC_EXTI2_3_IRQ);
  nvic_set_priority(NVIC_EXTI2_3_IRQ, 2);
}

void exti2_3_isr() {
  exti_reset_request(EXTI3);
  isr0();
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

    uint32_t now = sys_tick_timer;
    while ((sys_tick_timer - now) < 5000) {
    }
  }
}
