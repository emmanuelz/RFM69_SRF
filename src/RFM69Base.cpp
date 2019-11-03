// Copyright Emmanuel ZURMELY <emmanu.zurmely@evc.net> - 07/2019
// This software is distributed under the terms of the GNU General Public License version 3.
//
// This code is mainly based on the library RFM69_LowPowerLab from LowPowerLab <lowpowerlab.com>

#include "RFM69Base.h"

#include "RFM69registers.h"

#define RF69_MODE_SLEEP         0 // XTAL OFF
#define RF69_MODE_STANDBY       1 // XTAL ON
#define RF69_MODE_SYNTH         2 // PLL ON
#define RF69_MODE_RX            3 // RX MODE
#define RF69_MODE_TX            4 // TX MODE

// upper RX signal sensitivity threshold in dBm for carrier sense access
#define CSMA_LIMIT              -90

#define RF69_CSMA_LIMIT_MS 1000
#define RF69_TX_LIMIT_MS   1000

#define RFM69_FOSC 32.0e6f
#define RFM69_FSTEP (RFM69_FOSC / (1L << 19))
#define RFM69_FREQUENCY(x) ((x) / RFM69_FSTEP)
#define RFM69_BITRATE(x) (RFM69_FOSC / (x))

#define SPI_CLOCK_DIVIDER SPI_CLOCK_DIV2
#define SPI_SPEED 4000000

RFM69Base::RFM69Base(uint8_t pinSlaveSelect, uint8_t pinInterruption) {
  this->pinSlaveSelect = pinSlaveSelect;
  this->pinInterruption = pinInterruption;
  mode = RF69_MODE_STANDBY;
}

void RFM69Base::initialize() {
  initializePins();
  initializeRadio();
}

void RFM69Base::initializeRadio() {
  setDefaultSettings();
  setDefaultEncryptionKey();

  setMode(RF69_MODE_STANDBY);
  const uint8_t timeout = 50;
  long start = millis();
  // wait for ModeReady
  while (((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) && millis() - start < timeout);
  if (millis() - start >= timeout) {
    goto error;
  }

  {
    uint8_t interrupt = digitalPinToInterrupt(pinInterruption);
    if (interrupt == NOT_AN_INTERRUPT) {
      goto error;
    }
    instance = this;
    attachInterrupt(interrupt, RFM69Base::isr0, RISING);
  }
  return;

error:
  error = RFM69BaseError::ErrorInitialization;
}

void RFM69Base::initializePins() {
  digitalWrite(pinSlaveSelect, HIGH);
  pinMode(pinSlaveSelect, OUTPUT);

  SPI.begin();
  spiSettings = SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0);
}

bool RFM69Base::isDataAvailable() {
  if (dataReceived) {
    dataReceived = false;
    readReceivedData();
  }
  if ((mode == RF69_MODE_RX) && (rssi != 0)) {
    setMode(RF69_MODE_STANDBY);
    return true;
  } else if (mode == RF69_MODE_RX) {
    // already in RX no payload yet
    return false;
  }
  receiveBegin();
  return false;
}

void RFM69Base::receiveBegin() {
  buffer[0] = 0;
  rssi = 0;
  if (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY) {
    // avoid RX deadlocks
    writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART);
  }
  // set DIO0 to "PAYLOADREADY" in receive mode
  writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01);
  setMode(RF69_MODE_RX);
}

void RFM69Base::readReceivedData() {
  if (mode == RF69_MODE_RX && (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)) {
    uint8_t n = packetSize;
    setMode(RF69_MODE_STANDBY);
    select();
    SPI.transfer(REG_FIFO & 0x7F);
    if (n == 0) {
      // variable length packet, first byte is size
      n = SPI.transfer(0);
      if (n > MAX_DATA_LENGTH) {
        n = MAX_DATA_LENGTH;
      }
    }
    for (uint8_t i = 0; i < n; i++) {
      buffer[i] = SPI.transfer(0);
    }
    unselect();
    setMode(RF69_MODE_RX);
  }
  rssi = readRSSI();
}

uint8_t RFM69Base::getByteToSend(const uint8_t *data, uint8_t dataLength, uint8_t index) {
  if (index >= dataLength) {
    return 0;
  } else {
    return data[index];
  }
}

void RFM69Base::sendPacket(const uint8_t *data, uint8_t dataLength) {
  while (dataLength > 0) {
    // avoid RX deadlocks
    writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART);
    uint32_t now = millis();
    while (!canSend() && millis() - now < RF69_CSMA_LIMIT_MS) {
      isDataAvailable();
    }
    uint8_t sent = sendFrame(data, dataLength);
    dataLength -= sent;
    data += sent;
    delay(1);
  }
}

uint8_t RFM69Base::sendFrame(const uint8_t *data, uint8_t dataLength) {
  // turn off receiver to prevent reception while filling fifo
  setMode(RF69_MODE_STANDBY);
  // wait for ModeReady
  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00);
  // DIO0 is "Packet Sent"
  writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00);
  // write to FIFO
  select();
  SPI.transfer(REG_FIFO | 0x80);
  for (uint8_t i = 0; i < packetSize; i++) {
    uint8_t value = getByteToSend(data, dataLength, i);
    SPI.transfer(value);
  }
  unselect();

  // no need to wait for transmit mode to be ready since its handled by the radio
  setMode(RF69_MODE_TX);
  uint32_t txStart = millis();
  // wait for DIO0 to turn HIGH signalling transmission finish
  while (digitalRead(pinInterruption) == 0 && millis() - txStart < RF69_TX_LIMIT_MS) { }
  setMode(RF69_MODE_STANDBY);

  uint8_t payloadLength = getPayloadLength();
  if (payloadLength > dataLength) {
    payloadLength = dataLength;
  }
  return payloadLength;
}

bool RFM69Base::canSend() {
  // if signal stronger than -100dBm is detected assume channel activity
  if (mode == RF69_MODE_RX && rssi == 0 && readRSSI() < CSMA_LIMIT) {
    setMode(RF69_MODE_STANDBY);
    return true;
  }
  return false;
}

void RFM69Base::isr0() {
  if (instance != NULL) {
    instance->dataReceived = true;
  }
}

void RFM69Base::setDefaultEncryptionKey() {
  setEncryptionKey(NULL);
}

int8_t RFM69Base::readRSSI(bool forceTrigger) {
  // get the received signal strength indicator (RSSI)
  if (forceTrigger) {
    // RSSI trigger not needed if DAGC is in continuous mode
    writeReg(REG_RSSICONFIG, RF_RSSI_START);
    // wait for RSSI_Ready
    while ((readReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00);
  }
  int8_t rssi = -(readReg(REG_RSSIVALUE) >> 1);
  return rssi;
}

void RFM69Base::setEncryptionKey(const char *key) {
  useEncryption = key != NULL;
  setMode(RF69_MODE_STANDBY);
  if (useEncryption) {
    select();
    SPI.transfer(REG_AESKEY1 | 0x80);
    for (uint8_t i = 0; i < 16; i++) {
      SPI.transfer(key[i]);
    }
    unselect();
  }
  uint8_t reg = readReg(REG_PACKETCONFIG2) & ~RF_PACKET2_MASK_AES;
  writeReg(REG_PACKETCONFIG2, reg | (useEncryption ? RF_PACKET2_AES_ON : RF_PACKET2_AES_OFF));
}

void RFM69Base::setDefaultSettings() {
  // 0x01
  writeReg(REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY);

  // 0x02
  // Packet mode, FSK, Gaussian filter BT=1
  writeReg(REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_01);

  // 0x03, 0x04
  // Bit rate = 250 kbps
  setDataRateBps(250000);

  // 0x05, 0x06
  // Deviation = bit rate / 2
  setFrequencyDeviationHz(250000 / 2);

  // 0x07, 0x08, 0x09
  // Frequency = 868.3 MHz
  setFrequencyHz(868300000);

  // 0x19
  // Rx Bandwidth > bit rate / 2
  setRxBandwidthHz(250000 / 2);

  // 0x25
  // DIO0 is the only IRQ we're using
  writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01);

  // 0x26
  // DIO5 ClkOut disable for power saving
  writeReg(REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF);

  // 0x28
  // writing to this bit ensures that the FIFO & status flags are reset
  writeReg(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);

  // 0x29
  // Sensitivity = -110 dBm
  writeReg(REG_RSSITHRESH, 220);

  // 0x2D
  // Preamble = 4 bytes (0xAAAAAAAA)
  writeReg(REG_PREAMBLELSB, 4);

  // 0x2E
  // Sync bytes = 4 bytes, tolerance = 30/32 bits
  writeReg(REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_4 | RF_SYNC_TOL_2);

  // 0x2F, 0x30, 0x31, 0x32
  // Sync bytes = 0xD391 repeated one time
  writeReg(REG_SYNCVALUE1, 0xD3);
  writeReg(REG_SYNCVALUE2, 0x91);
  writeReg(REG_SYNCVALUE3, 0xD3);
  writeReg(REG_SYNCVALUE4, 0x91);

  // 0x37
  // Format fixed length, dcfree off, no crc, no crc filter, no address filter
  writeReg(REG_PACKETCONFIG1, RF_PACKET1_FORMAT_FIXED | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_OFF | RF_PACKET1_CRCAUTOCLEAR_OFF | RF_PACKET1_ADRSFILTERING_OFF);

  // 0x38
  // Packet size = 12 (payload) + 5 (1 byte length, 2 bytes network id, 2 bytes crc) = 17
  setPacketSize(17);

  // 0x3C
  // TX on FIFO not empty
  writeReg(REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE);

  // 0x3D
  writeReg(REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF);

  // 0x6F
  // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
  writeReg(REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0);

  // Puissance maximum
  setPowerLevelDBm(13);
  setCurrentLimit(120);
}

void RFM69Base::writeReg(uint8_t addr, uint8_t value)
{
  select();
  SPI.transfer(addr | 0x80);
  SPI.transfer(value);
  unselect();
}

uint8_t RFM69Base::readReg(uint8_t addr)
{
  select();
  SPI.transfer(addr & 0x7F);
  uint8_t value = SPI.transfer(0);
  unselect();
  return value;
}

void RFM69Base::select() {
  // select the RFM69 transceiver (set CS low)
  SPI.beginTransaction(spiSettings);

  // set RFM69 SPI settings
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIVIDER);
  digitalWrite(pinSlaveSelect, LOW);
}

void RFM69Base::unselect() {
  // unselect the RFM69 transceiver (set CS high, restore SPI settings)
  digitalWrite(pinSlaveSelect, HIGH);
  SPI.endTransaction();
}

void RFM69Base::setFrequencyHz(uint32_t baseFrequencyHz) {
  uint8_t oldMode = mode;
  if (oldMode == RF69_MODE_TX) {
    setMode(RF69_MODE_RX);
  }
  uint32_t value = RFM69_FREQUENCY(baseFrequencyHz);
  writeReg(REG_FRFMSB, (value >> 16) & 0xFF);
  writeReg(REG_FRFMID, (value >> 8) & 0xFF);
  writeReg(REG_FRFLSB, value & 0xFF);
  if (oldMode == RF69_MODE_RX) {
    setMode(RF69_MODE_SYNTH);
  }
  setMode(oldMode);
}

void RFM69Base::setDataRateBps(uint32_t dataRateBps) {
  uint16_t value = RFM69_BITRATE(dataRateBps);
  writeReg(REG_BITRATEMSB, (value >> 8) & 0xFF);
  writeReg(REG_BITRATELSB, value & 0xFF);
}

void RFM69Base::setFrequencyDeviationHz(uint32_t deviationHz) {
  uint16_t value = RFM69_FREQUENCY(deviationHz);
  writeReg(REG_FDEVMSB, (value >> 8) & 0xFF);
  writeReg(REG_FDEVLSB, value & 0xFF);
}

uint8_t RFM69Base::getPowerLevelDBm() {
  int8_t value = readReg(REG_PALEVEL) - 18;
  return value;
}

void RFM69Base::setPowerLevelDBm(int8_t powerDBm) {
  // supports only PA0 mode from -18 to 13 dBm
  if (powerDBm < -18 || powerDBm > 13) {
    return;
  }
  uint8_t value = (unsigned) (powerDBm + 18);
  writeReg(REG_PALEVEL, RF_PALEVEL_PA0_OFF | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_OFF | (value & 0x1F));
}

void RFM69Base::setCurrentLimit(int8_t currentMA) {
  if (currentMA < 45) {
    currentMA = 45;
  } else if (currentMA > 120) {
    writeReg(REG_OCP, RF_OCP_OFF);
    return;
  }
  currentMA = (currentMA - 45) / 5;
  writeReg(REG_OCP, RF_OCP_ON | (currentMA & 0xF));
}

void RFM69Base::setMode(uint8_t newMode) {
  if (newMode == mode) {
    return;
  }

  uint8_t reg = readReg(REG_OPMODE) & ~RF_OPMODE_MASK_MODE;

  switch (newMode) {
    case RF69_MODE_TX:
      writeReg(REG_OPMODE, (reg | RF_OPMODE_TRANSMITTER));
      break;

    case RF69_MODE_RX:
      writeReg(REG_OPMODE, (reg | RF_OPMODE_RECEIVER));
      break;

    case RF69_MODE_SYNTH:
      writeReg(REG_OPMODE, (reg | RF_OPMODE_SYNTHESIZER));
      break;

    case RF69_MODE_STANDBY:
      writeReg(REG_OPMODE, (reg | RF_OPMODE_STANDBY));
      break;

    case RF69_MODE_SLEEP:
      writeReg(REG_OPMODE, (reg | RF_OPMODE_SLEEP));
      break;

    default:
      return;
  }

  // we are using packet mode, so this check is not really needed
  // but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
  // wait for ModeReady
  while (mode == RF69_MODE_SLEEP && (readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) { }
  mode = newMode;
}

void RFM69Base::sleep() {
  setMode(RF69_MODE_SLEEP);
}

void RFM69Base::wake() {
  isDataAvailable();
}

int8_t RFM69Base::getRSSI() const {
  return rssi;
}

void RFM69Base::setPacketSize(uint8_t packetSize) {
  if (packetSize > MAX_DATA_LENGTH) {
    packetSize = MAX_DATA_LENGTH;
  }
  this->packetSize = packetSize;

  uint8_t reg = readReg(REG_PACKETCONFIG1) & ~RF_PACKET1_MASK_FORMAT;
  writeReg(REG_PACKETCONFIG1, reg | (packetSize > 0 ? RF_PACKET1_FORMAT_FIXED : RF_PACKET1_FORMAT_VARIABLE));
  writeReg(REG_PAYLOADLENGTH, packetSize);
}

RFM69BaseError RFM69Base::getError() const {
  return error;
}

uint32_t RFM69Base::getFrequencyHz() {
  return RFM69_FSTEP * (((uint32_t) readReg(REG_FRFMSB) << 16) + ((uint16_t) readReg(REG_FRFMID) << 8) + readReg(REG_FRFLSB));
}

void RFM69Base::begin() {
  initialize();
}

void RFM69Base::readAllRegs() {
  uint8_t regVal;

  Serial.println(F("Address - HEX - BIN"));
  for (uint8_t regAddr = 1; regAddr <= 0x4F; regAddr++) {
    select();
    // send address + r/w bit
    SPI.transfer(regAddr & 0x7F);
    regVal = SPI.transfer(0);
    unselect();

    Serial.print(regAddr, HEX);
    Serial.print(F(" - "));
    Serial.print(regVal, HEX);
    Serial.print(F(" - "));
    Serial.println(regVal, BIN);
  }
  unselect();
}

void RFM69Base::setRxBandwidthHz(uint32_t bandWidthHz) {
  constexpr float ln2 = log(2);
  bool ook = (readReg(REG_DATAMODUL) & RF_DATAMODUL_MODULATIONTYPE_OOK) == RF_DATAMODUL_MODULATIONTYPE_OOK;
  for (uint8_t mantissa = 6; mantissa >= 4; mantissa--) {
    for (int8_t exponent = 7; exponent >= 0; exponent--) {
      uint32_t value = (uint32_t) RFM69_BITRATE(4.0f * mantissa * exp((exponent + (ook ? 3 : 2)) * ln2));
      if (value >= bandWidthHz) {
        writeReg(REG_RXBW, RF_RXBW_DCCFREQ_010 | ((mantissa - 4) << 3) | exponent);
        return;
      }
    }
  }
}
