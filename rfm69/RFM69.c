
#include "RFM69.h"
#include "timer.h"

static uint8_t mode;
static bool isRFM69HW = false;
static uint8_t powerLevel;
//static const uint8_t maxPowerLevel = 31;
static uint16_t address;
static uint8_t PAYLOADLEN;
static volatile bool haveData;
static uint8_t DATA[RF69_MAX_DATA_LEN + 1];
static uint8_t DATALEN;
static uint16_t SENDERID;
static uint16_t TARGETID;
static uint8_t ACK_REQUESTED;
static uint8_t ACK_RECEIVED;
static uint16_t RSSI;
static volatile uint16_t RF69_LISTEN_BURST_REMAINING_MS;
static bool spyMode = false;
static uint8_t freqBand;
static uint8_t networkID;
static const uint8_t RFM69_CTL_RESERVE1 = 0x20;

// Variables used the ACT side
static int16_t targetRSSI;
static int16_t ackRSSI;
static volatile uint8_t ACK_RSSI_REQUESTED;
static const uint8_t startingPowerLevel = 1;
static uint16_t transmitLevel;
static uint16_t transmitLevelStep;

#define abs(x) (((x) > 0) ? (x) : -(x))

static void select(void) {
  noInterrupts();
  //  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
  RFM69_setCSPin(false);
}

static void unselect(void) {
  //  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
  RFM69_setCSPin(true);
  interrupts();
}

static uint8_t readReg(uint8_t addr) {
  select();
  // Perform the SPI read operation
  uint8_t regVal = 0;
  // regVal = ...


  //uint8_t buff = addr;
  uint8_t buff = addr & 0x7F;
  SPI_transfer8Bit(buff);
  regVal = SPI_transfer8Bit(0);
  //  HAL_SPI_Transmit(&hspi1, &buff, 1, 1000);
  //  HAL_SPI_Receive(&hspi1, &regVal, 1, 1000);
  unselect();
  return regVal;
}

static void writeReg(uint8_t addr, uint8_t value) {
  select();
  // Perform the SPI write operation
  //  uint8_t buff[2];
  //  buff[0] = addr | 0x80;
  //  buff[1] = value;
  SPI_transfer8Bit(addr | 0x80);
  SPI_transfer8Bit(value);
  //  HAL_SPI_Transmit(&hspi1, buff, 2, 1000);
  unselect();
}
// HAL_SPI_TransmitReceive (SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
static uint8_t writeAndReceive(uint8_t outData) {
  return SPI_transfer8Bit(outData);
  //  uint8_t inData;
  //  HAL_SPI_TransmitReceive(&hspi1, &outData, &inData, 1, 1000);
  //  return inData;
}

//void spi_transfer(uint8_t data) {
//  SPI_transfer8Bit(data);
//  //  HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
//}

static void setHighPowerRegs(bool onOff) {
  writeReg(REG_TESTPA1, onOff ? 0x5D : 0x55);
  writeReg(REG_TESTPA2, onOff ? 0x7C : 0x70);
}

void isr0(void) {
  haveData = true;  
}

static void setMode(uint8_t newMode) {
  if (newMode == mode) {
    return;
  }

  switch (newMode) {
  case RF69_MODE_TX:
    writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
    if (isRFM69HW)
      setHighPowerRegs(true);
    if (targetRSSI) {
      RFM69_setPowerLevel(transmitLevel);
    }
    break;
  case RF69_MODE_RX:
    writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
    if (isRFM69HW)
      setHighPowerRegs(false);
    break;
  case RF69_MODE_SYNTH:
    writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
    break;
  case RF69_MODE_STANDBY:
    writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
    break;
  case RF69_MODE_SLEEP:
    writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
    break;
  default:
    return;
  }

   // we are using packet mode, so this check is not really needed
  // but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
  while (mode == RF69_MODE_SLEEP && (readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady

  mode = newMode;
}

// get the received signal strength indicator (RSSI)
static int16_t readRSSI(bool forceTrigger) {
  int16_t rssi = 0;
  if (forceTrigger)
  {
    // RSSI trigger not needed if DAGC is in continuous mode
    writeReg(REG_RSSICONFIG, RF_RSSI_START);
    while ((readReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00); // wait for RSSI_Ready
  }
  rssi = -readReg(REG_RSSIVALUE);
   rssi >>= 1;
  return rssi;
}



static void encrypt(const char *key) {
  // Something about is listenmode_enable
  setMode(RF69_MODE_STANDBY);
  //uint8_t validKey = key != 0 && strlen(key) != 0;
  uint8_t validKey = key != 0 && strlen(key) != 0;
  if (validKey) {
    // Listemmode_enable
    select();
    spi_transfer(REG_AESKEY1 | 0x80);
    for (int i = 0; i < 16; i++) {
      spi_transfer(key[i]);
    }
    unselect();
  }
  writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFE) | (validKey ? 1 : 0));
}

// for RFM69HW only: you must call setHighPower(true) after initialize() or else transmission won't work
static void setHighPower(bool onOff) {
  isRFM69HW = onOff;
  writeReg(REG_OCP, isRFM69HW ? RF_OCP_OFF : RF_OCP_ON);
  if (isRFM69HW) // turning ON
    writeReg(REG_PALEVEL, (readReg(REG_PALEVEL) & 0x1F) | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON); // enable P1 & P2 amplifier stages
  else
    writeReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | powerLevel); // enable P0 only
}

bool rfm69_init(uint8_t freqBand_param, uint8_t nodeID, uint8_t networkID_param) {
  const uint8_t CONFIG[][2] =
  {
    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
    /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_55555}, // default: 4.8 KBPS
    /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_55555},
    /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_50000}, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
    /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_50000},

    /* 0x07 */ { REG_FRFMSB, (uint8_t) (freqBand_param==RF69_315MHZ ? RF_FRFMSB_315 : (freqBand_param==RF69_433MHZ ? RF_FRFMSB_433 : (freqBand_param==RF69_868MHZ ? RF_FRFMSB_868 : RF_FRFMSB_915))) },
    /* 0x08 */ { REG_FRFMID, (uint8_t) (freqBand_param==RF69_315MHZ ? RF_FRFMID_315 : (freqBand_param==RF69_433MHZ ? RF_FRFMID_433 : (freqBand_param==RF69_868MHZ ? RF_FRFMID_868 : RF_FRFMID_915))) },
    /* 0x09 */ { REG_FRFLSB, (uint8_t) (freqBand_param==RF69_315MHZ ? RF_FRFLSB_315 : (freqBand_param==RF69_433MHZ ? RF_FRFLSB_433 : (freqBand_param==RF69_868MHZ ? RF_FRFLSB_868 : RF_FRFLSB_915))) },

    // looks like PA1 and PA2 are not implemented on RFM69W/CW, hence the max output power is 13dBm
    // +17dBm and +20dBm are possible on RFM69HW
    // +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
    // +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
    // +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
    ///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
    ///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

    // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
    //for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
    /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
    /* 0x26 */ { REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
    /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
    /* 0x29 */ { REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
    ///* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
    /* 0x2E */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
    /* 0x2F */ { REG_SYNCVALUE1, 0x2D },      // attempt to make this compatible with sync1 byte of RFM12B lib
    /* 0x30 */ { REG_SYNCVALUE2, networkID_param }, // NETWORK ID
    //* 0x31 */ { REG_SYNCVALUE3, 0xAA },
    //* 0x31 */ { REG_SYNCVALUE4, 0xBB },
    /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF },
    /* 0x38 */ { REG_PAYLOADLENGTH, 66 }, // in variable length mode: the max frame size, not used in TX
    ///* 0x39 */ { REG_NODEADRS, nodeID }, // turned off because we're not using address filtering
    /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
    /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    //for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
    {255, 0}
  };

  // Setup gpio CE pin (this has already been done via the CubeMX)
  // I think that it might be better if this was at the beginning of the function
  // Though it probably doesn't actually matter
  //  MX_GPIO_Init();
  //MX_SPI1_Init();
  unselect();
		

  // TODO: Add the timeout and return here as I currently don't implement any kind of timeout in the library.
  do {
    writeReg(REG_SYNCVALUE1, 0xAA);
  } while (readReg(REG_SYNCVALUE1) != 0xAA);

  do {
    writeReg(REG_SYNCVALUE1, 0x55);
  } while (readReg(REG_SYNCVALUE1) != 0x55);

  for (uint8_t i = 0; CONFIG[i][0] != 255; i++)
    writeReg(CONFIG[i][0], CONFIG[i][1]);

  // Encryption is persistent between resets and can trip you up during debugging.
  // Disable it during initialization so we always start from a known state.
  encrypt(0);

  setHighPower(false);
  // setHighPower(isRFM69HW);
  setMode(RF69_MODE_STANDBY);
  // TODO: Add timeout check here
  while((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) {
    // Add timeout
  }

  // TODO: Add interupt code
  address = nodeID;
  freqBand = freqBand_param;
  networkID = networkID_param;
  // TODO: Finish off here from here!!!
  noInterrupts();
  rfm69_setPowerLevel(startingPowerLevel);

  // Setting up ATC
  // TODO: Set up method to only run ATC if requested, and not by default
  // I might want to move these to the top of init, as done in the RFM69 Arduino library
  targetRSSI = 0;
  ackRSSI = 0;
  ACK_RSSI_REQUESTED = 0;
  transmitLevel = 20;
  transmitLevelStep = 1;
  
  return true;
}


static bool canSend(void)
{
  if (mode == RF69_MODE_RX && PAYLOADLEN == 0 && readRSSI(false) < CSMA_LIMIT) // if signal stronger than -100dBm is detected assume channel activity
  {
    setMode(RF69_MODE_STANDBY);
    return true;
  }
  return false;
}

// internal function
static void receiveBegin(void) {
  ACK_RSSI_REQUESTED = 0;
  DATALEN = 0;
  SENDERID = 0;
  TARGETID = 0;
  PAYLOADLEN = 0;
  ACK_REQUESTED = 0;
  ACK_RECEIVED = 0;
#if defined(RF69_LISTENMODE_ENABLE)
  RF69_LISTEN_BURST_REMAINING_MS = 0;
#endif
  RSSI = 0;
  if (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)
    writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
  setMode(RF69_MODE_RX);
}

static void interruptHook(uint8_t CTLbyte) {
  ACK_RSSI_REQUESTED = CTLbyte & RFM69_CTL_RESERVE1; // TomWS1: extract the ACK RSSI request bit (could potentially merge with ACK_REQUESTED)
  // TomWS1: now see if this was an ACK with an ACK_RSSI response
  if (ACK_RECEIVED){// && ACK_RSSI_REQUESTED) {
    // the next two bytes contain the ACK_RSSI (assuming the datalength is valid)
    if (DATALEN >= 1) {
      ackRSSI = -1 * writeAndReceive(0); //rssi was sent as single byte positive value, get the real value by * -1
      DATALEN -= 1;   // and compensate data length accordingly
      // TomWS1: Now dither transmitLevel value (register update occurs later when transmitting);
      if (targetRSSI != 0) {
        // if (_isRFM69HW) {
          // if (_ackRSSI < _targetRSSI && _transmitLevel < 51) _transmitLevel++;
          // else if (_ackRSSI > _targetRSSI && _transmitLevel > 32) _transmitLevel--;
        // } else {
        if (ackRSSI < targetRSSI && transmitLevel < 31)
        {
          transmitLevel += transmitLevelStep;
          if (transmitLevel > 31) transmitLevel = 31;
        }
        else if (ackRSSI > targetRSSI && transmitLevel > 0)
          transmitLevel--;
        //}
      }
    }
  }
}

// internal function - interrupt gets called when a packet is received
void interruptHandler() {
  if (mode == RF69_MODE_RX && (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY))
  {
    setMode(RF69_MODE_STANDBY);
    select();
    spi_transfer(REG_FIFO & 0x7F);
    PAYLOADLEN = writeAndReceive(0);
    PAYLOADLEN = PAYLOADLEN > 66 ? 66 : PAYLOADLEN; // precaution
    TARGETID = writeAndReceive(0);//_spi->transfer(0);
    SENDERID = writeAndReceive(0);
    uint8_t CTLbyte = writeAndReceive(0);
    TARGETID |= ((uint16_t)(CTLbyte) & 0x0C) << 6; //10 bit address (most significant 2 bits stored in bits(2,3) of CTL byte
		 SENDERID |= ((uint16_t)(CTLbyte) & 0x03) << 8; //10 bit address (most sifnigicant 2 bits stored in bits(0,1) of CTL byte

    if(!(spyMode || TARGETID == address || TARGETID == RF69_BROADCAST_ADDR) // match this node's address, or broadcast address or anything in spy mode
       || PAYLOADLEN < 3) // address situation could receive packets that are malformed and don't fit this libraries extra fields
    {
      PAYLOADLEN = 0;
      unselect();
      receiveBegin();
      return;
    }

    DATALEN = PAYLOADLEN - 3;
    ACK_RECEIVED = CTLbyte & RFM69_CTL_SENDACK; // extract ACK-received flag
    ACK_REQUESTED = CTLbyte & RFM69_CTL_REQACK; // extract ACK-requested flag
    interruptHook(CTLbyte);     // TWS: hook to derived class interrupt function

    for (uint8_t i = 0; i < DATALEN; i++) DATA[i] = writeAndReceive(0);

    DATA[DATALEN] = 0; // add null at end of string // add null at end of string
    unselect();
    setMode(RF69_MODE_RX);
  }
  RSSI = readRSSI(false);
}

// checks if a packet was received and/or puts transceiver in receive (ie RX or listen) mode
static bool receiveDone(void) {
  if (haveData)
    {
      haveData = false;
      interruptHandler();}
  noInterrupts();
  if (mode == RF69_MODE_RX && PAYLOADLEN > 0)
  {
    setMode(RF69_MODE_STANDBY); // enables interrupts
    return true;
  }
  else if (mode == RF69_MODE_RX) // already in RX no payload yet
  {
    interrupts();
    return false;
  }
  receiveBegin();
  return false;
}

// Updated for ATC
static void sendFrame(uint16_t toAddress, const void *buffer, uint8_t bufferSize,
               bool requestACK, bool sendACK, bool sendRSSI, int16_t lastRSSI) {

  setMode(RF69_MODE_STANDBY);
  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00){} // wait for ModeReady
  //writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"
  bufferSize += (sendACK && sendRSSI) ? 1 : 0;   // if sending ACK_RSSI then increase data size by 1
  if (bufferSize > RF69_MAX_DATA_LEN) bufferSize = RF69_MAX_DATA_LEN;


  // // control byte
  //  uint8_t CTLbyte = 0x00;
  //  if (sendACK)
  //    CTLbyte = RFM69_CTL_SENDACK;
  //  else if (requestACK)
  //    CTLbyte = RFM69_CTL_REQACK;
  //
  //  if (toAddress > 0xFF) CTLbyte |= (toAddress & 0x300) >> 6; //assign last 2 bits of address if > 255
  //  if (address > 0xFF) CTLbyte |= (address & 0x300) >> 8;   //assign last 2 bits of address if > 255

  select();
  spi_transfer(REG_FIFO | 0x80);
  spi_transfer(bufferSize + 3);
  spi_transfer((uint8_t)toAddress);
  spi_transfer((uint8_t)address);
  //spi_transfer(CTLbyte);

  // CTL (control byte)
  uint8_t CTLbyte=0x0;
  if (toAddress > 0xFF) CTLbyte |= (toAddress & 0x300) >> 6; //assign last 2 bits of address if > 255
  if (address > 0xFF) CTLbyte |= (address & 0x300) >> 8;   //assign last 2 bits of address if > 255
  if (sendACK) {                   // TomWS1: adding logic to return ACK_RSSI if requested
    spi_transfer(CTLbyte | RFM69_CTL_SENDACK | (sendRSSI?RFM69_CTL_RESERVE1:0));  // TomWS1  TODO: Replace with EXT1
    if (sendRSSI) {
      spi_transfer(abs(lastRSSI)); //RSSI dBm is negative expected between [-100 .. -20], convert to positive and pass along as single extra header byte
      bufferSize -=1;              // account for the extra ACK-RSSI 'data' byte
    }
  }
  else if (requestACK) {  // TODO: add logic to request ackRSSI with ACK - this is when both ends of a transmission would dial power down. May not work well for gateways in multi node networks
    spi_transfer(CTLbyte | (targetRSSI ? RFM69_CTL_REQACK | RFM69_CTL_RESERVE1 : RFM69_CTL_REQACK));
  }
  else spi_transfer(CTLbyte);

  for (uint8_t i = 0; i < bufferSize; i++)
    spi_transfer(((uint8_t*) buffer)[i]);
  unselect();
  
  // no need to wait for transmit mode to be ready since its handled by the radio
  setMode(RF69_MODE_TX);
  //uint32_t txStart = millis();
  //while (digitalRead(_interruptPin) == 0 && millis() - txStart < RF69_TX_LIMIT_MS); // wait for DIO0 to turn HIGH signalling transmission finish
  while ((readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT) == 0x00){} // wait for PacketSent
  setMode(RF69_MODE_STANDBY);
}

static void send(uint16_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK)
{
  writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  uint32_t now = timer_get_sys_tick();
  //while (!canSend()) receiveDone();
  while (!canSend() && timer_get_sys_tick() - now < RF69_CSMA_LIMIT_MS) receiveDone();
  // TODO: Start again from implementing the below.
  sendFrame(toAddress, buffer, bufferSize, requestACK, false, false, 0);
}

// should be polled immediately after sending a packet with ACK request
static bool ACKReceived(uint16_t fromNodeID) {
  if (receiveDone())
    return (SENDERID == fromNodeID || fromNodeID == RF69_BROADCAST_ADDR) && ACK_RECEIVED;
  return false;
}


// This has been updated for ATC (at this moment, the ATC is untested)
bool rfm69_sendWithRetry(uint16_t toAddress, const void *buffer,
                         uint8_t bufferSize, uint8_t retries,
                         uint8_t retryWaitTime) {
  uint32_t sentTime;
  for (uint8_t i = 0; i <= retries; i++) {
    send(toAddress, buffer, bufferSize, true);
    sentTime = timer_get_sys_tick();
    while ((timer_get_sys_tick() - sentTime) < retryWaitTime) {
      //  while(1) {
      if (ACKReceived(toAddress)) {
	return true;
      }
    }
    if (transmitLevel < 31) {
      transmitLevel += transmitLevelStep;
      if (transmitLevel > 31)
	{
	  transmitLevel = 31;
	}
    }
  }
  return false; 
}

static void listenModeReset(void)
{
  DATALEN = 0;
  SENDERID = 0;
  TARGETID = 0;
  PAYLOADLEN = 0;
  ACK_REQUESTED = 0;
  ACK_RECEIVED = 0;
  RF69_LISTEN_BURST_REMAINING_MS = 0;
}


void rfm69_listenModeInterruptHandler(void) {
     interruptHandler();
  return; 
  haveData = true;
  return;
  
  if (DATALEN != 0) {
    return;
  }
  
  listenModeReset();
  // TODO: Implement this feature, which I think is an Ardino thing noInterrupts();
  select();

  union // union to simplify addressing of long and short parts of time offset
  {
    uint32_t l;
    uint8_t b[4];
  } burstRemaining;

  burstRemaining.l = 0;

  spi_transfer(REG_FIFO & 0x7F);
  // TODO: Complete using the writeAndRead function written above
  PAYLOADLEN = writeAndReceive(0);
  PAYLOADLEN = PAYLOADLEN > 64 ? 64 : PAYLOADLEN;
  TARGETID = writeAndReceive(0);
  if (!(spyMode || TARGETID == address ||
	TARGETID ==
	    RF69_BROADCAST_ADDR) // match this node's address, or broadcast
				 // address or anything in spy mode
      || PAYLOADLEN < 3) // address situation could receive packets that are
			 // malformed and don't fit this library's extra fields
  {
    listenModeReset();
    goto out;
  }

  // We've read the target, and will read the sender id and two time offset bytes for a total of 4 bytes
  DATALEN = PAYLOADLEN - 4;
  SENDERID = writeAndReceive(0);
  burstRemaining.b[0] = writeAndReceive(0);   // and get the time remaining
  burstRemaining.b[1] = writeAndReceive(0);
  RF69_LISTEN_BURST_REMAINING_MS = burstRemaining.l;
  
  for (uint8_t i = 0; i < DATALEN; i++) {
    DATA[i] = writeAndReceive(0);
  }

  if (DATALEN < RF69_MAX_DATA_LEN) {
    DATA[DATALEN] = 0;  // add null at the end of string
  }

 out:
   unselect();
   // This is something from Arduino that will need to be implemented interrupts();
}

void RFM69_setPowerLevel(uint8_t powerLevel_param) {
  powerLevel = (powerLevel_param > 31 ? 31 : powerLevel_param);
  if (isRFM69HW) powerLevel /= 2;
  writeReg(REG_PALEVEL, (readReg(REG_PALEVEL) & 0xE0) | powerLevel);
}

void rfm69_setPowerLevel(uint8_t powerLevel_param) {
  powerLevel = (powerLevel_param > 31 ? 31 : powerLevel);
  if (isRFM69HW) powerLevel /= 2;
  writeReg(REG_PALEVEL, (readReg(REG_PALEVEL) & 0xE0) | powerLevel);
}

uint8_t RFM69_getPowerLevel(void) {
return transmitLevel;  
}

void RFM69_enableAutoPower(int16_t targetRSSI_param) {
  targetRSSI = targetRSSI_param;
}

//=============================================================================
// getAckRSSI() - returns the RSSI value ack'd by the far end.
//=============================================================================
static int16_t  getAckRSSI(void){                     // TomWS1: New method to retrieve the ack'd RSSI (if any)
  return (targetRSSI==0?0:ackRSSI);
}
