 
#ifndef RFM69_h
#define RFM69_h

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "RFM69registers.h"
#include "timer.h"
#include "gpio.h"
#include "spi.h"


// available frequency bands
#define RF69_315MHZ            31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ            43
#define RF69_868MHZ            86
#define RF69_915MHZ            91

#define RF69_MAX_DATA_LEN       61 // to take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 3 bytes overhead - 2 bytes crc)
#define CSMA_LIMIT              -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP         0 // XTAL OFF
#define RF69_MODE_STANDBY       1 // XTAL ON
#define RF69_MODE_SYNTH         2 // PLL ON
#define RF69_MODE_RX            3 // RX MODE
#define RF69_MODE_TX            4 // TX MODE

#define null                  0
#define COURSE_TEMP_COEF    -90 // puts the temperature reading in the ballpark, user can fine tune the returned value
#define RF69_BROADCAST_ADDR   0
#define RF69_CSMA_LIMIT_MS 1000
#define RF69_TX_LIMIT_MS   1000
#define RF69_FSTEP  61.03515625 // == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)

// TWS: define CTLbyte bits
#define RFM69_CTL_SENDACK   0x80
#define RFM69_CTL_REQACK    0x40

#define RFM69_ACK_TIMEOUT   30  // 30ms roundtrip req for 61byte packets

bool rfm69_init(uint8_t freqBand, uint8_t nodeID, uint8_t networkID);
bool rfm69_sendWithRetry(uint16_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime);
void rfm69_listenModeInterruptHandler(void);
void interruptHandler(void);
void isr0(void);
void rfm69_setPowerLevel(uint8_t powerLevel_param);
uint8_t RFM69_getPowerLevel(void);
void RFM69_enableAutoPower(int16_t targetRSSI_param);
void RFM69_setPowerLevel(uint8_t powerLevel_param);
// extern functions
// This is done so that this module can be as target agnostic as possbile (though time will tell how well this works)
extern void noInterrupts(void);
extern void interrupts(void);

#endif
