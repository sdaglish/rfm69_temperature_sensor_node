#include "mcp9808.h"
#include <stdint.h>

static void write16(uint8_t reg, uint16_t value) {
  uint8_t buff[3];
  buff[0] = reg;
  buff[1] = value >> 8;
  buff[2] = value & 0xFF;
  //HAL_I2C_Master_Transmit(&hi2c1, MCP9808_I2CADDR_DEFAULT << 1, buff, 3, HAL_MAX_DELAY);
  i2c_transfer7(I2C1, MCP9808_I2CADDR_DEFAULT << 0, buff, 3, buff, 0);
}

static uint16_t read16(uint8_t reg) {
  uint8_t buff[2];
  i2c_transfer7(I2C1, MCP9808_I2CADDR_DEFAULT << 0, &reg, 1, buff, 0);
  //HAL_I2C_Master_Transmit(&hi2c1, MCP9808_I2CADDR_DEFAULT << 1, &reg, 1, HAL_MAX_DELAY);
  //HAL_I2C_Master_Receive(&hi2c1, MCP9808_I2CADDR_DEFAULT << 1, buff, 2, HAL_MAX_DELAY);
  i2c_transfer7(I2C1, MCP9808_I2CADDR_DEFAULT << 0, &reg, 0, buff, 2);
  uint16_t val = 0;
  val = buff[0];
  val <<= 8;
  val |= buff[1];
  return val;
}

bool mcp9808_init(void) {
  if (read16(MCP9808_REG_MANUF_ID) != 0x0054)
    return false;
  if (read16(MCP9808_REG_DEVICE_ID) != 0x0400)
    return false;

  write16(MCP9808_REG_CONFIG, 0x0);
  return true; 
}

uint16_t mcp9808_readTempReg(void) {
  return read16(MCP9808_REG_AMBIENT_TEMP);
}
