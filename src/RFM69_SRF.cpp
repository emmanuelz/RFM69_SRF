// Copyright Emmanuel ZURMELY <emmanu.zurmely@evc.net> - 07/2019
// This software is distributed under the terms of the GNU General Public License version 3.

#include "RFM69_SRF.h"

RFM69Base* RFM69Base::instance = NULL;

RFM69_SRF::RFM69_SRF(uint8_t pinSlaveSelect, uint8_t pinInterruption) : RFM69Base(pinSlaveSelect, pinInterruption) {
}

void RFM69_SRF::applySettings() {
  settingsChanged = false;
  uint32_t frequencyHz = baseFrequencyHz + channelSpacingHz * channelNumber;
  setFrequencyHz(frequencyHz);
}

uint8_t RFM69_SRF::revertBits(uint8_t value) {
  value = (value & 0xAA) >> 1 | (value & 0x55) << 1;
  value = (value & 0xCC) >> 2 | (value & 0x33) << 2;
  value = (value & 0xF0) >> 4 | (value & 0x0F) << 4;
  return value;
}

void RFM69_SRF::computeCrc() {
  resetCrc();
  for (uint8_t i = 0; i < receivedDataLength + 1; i++) {
    updateCrc(buffer[i]);
  }
}

void RFM69_SRF::resetCrc() {
  crc = 0xFFFF;
}

void RFM69_SRF::updateCrc(uint8_t value) {
  const uint16_t poly = 0x8005;

  uint16_t databyte = value;
  crc ^= databyte << 8;

  for (uint8_t j = 0; j < 8; ++j) {
    uint16_t mix = crc & 0x8000;
    crc <<= 1;
    if (mix) {
      crc = crc ^ poly;
    }
  }
}

void RFM69_SRF::initialize() {
  RFM69Base::initialize();
  initializeWhiteningMask();
}

void RFM69_SRF::initializeWhiteningMask() {
  resetWhiteningRegister();
  for (uint8_t i = 0; i < sizeof whiteningMask; i++) {
    whiteningMask[i] = getNextWhiteningByte();
  }
}

void RFM69_SRF::resetWhiteningRegister() {
  lfsr = 0x1FF;
}

uint8_t RFM69_SRF::getNextWhiteningByte() {
  uint16_t result = lfsr;
  // Whitening mask with polynom X9 + X5 + 1, output bits reversed
  for (uint8_t j = 0; j < 8; j++) {
    lfsr = (lfsr >> 1) | ((lfsr & 1) ^ ((lfsr >> 5) & 1)) << 8;
  }
  return result;
}

uint16_t RFM69_SRF::getCrc() const {
  return crc;
}

const uint8_t* RFM69_SRF::getReceivedData(uint8_t &dataLength) const {
  // received data length byte does not include the crc
  dataLength = receivedDataLength - SRF_DATA_BYTES;
  return buffer + INDEX_PAYLOAD;
}

bool RFM69_SRF::isValidCrc() {
  computeCrc();
  uint8_t *data = buffer + receivedDataLength + 1;
  uint16_t received = (data[0] << 8) | data[1];
  bool valid = received == crc;
  return valid;
}

bool RFM69_SRF::isValidNetworkId() {
  uint8_t *data = buffer + INDEX_NETWORKID;
  uint16_t received = (data[0] << 8) | data[1];
  bool valid = received == networkId;
  return valid;
}

void RFM69_SRF::applyWhitening() {
  for (uint8_t i = 0; i < packetSize; i++) {
    buffer[i] ^= whiteningMask[i];
  }
}

bool RFM69_SRF::checkReceivedData() {
  applyWhitening();

  // received data length
  receivedDataLength = *buffer;
  if ((receivedDataLength - SRF_DATA_BYTES) > getPayloadLength()) {
    return false;
  }
  return isValidNetworkId() && isValidCrc();
}

bool RFM69_SRF::isDataAvailable() {
  if (settingsChanged) {
    applySettings();
  }
  bool value = RFM69Base::isDataAvailable() && checkReceivedData();
  return value;
}

void RFM69_SRF::setChannelNumber(uint8_t channelNumber) {
  this->channelNumber = channelNumber;
  settingsChanged = true;
}

void RFM69_SRF::setChannelSpacingHz(uint32_t channelSpacingHz) {
  this->channelSpacingHz = channelSpacingHz;
  settingsChanged = true;
}

void RFM69_SRF::setNetworkId(uint16_t networkId) {
  this->networkId = networkId;
}

uint8_t RFM69_SRF::getPayloadLength() {
  return packetSize - SRF_CONTROL_BYTES;
}

uint8_t RFM69_SRF::getByteToSend(const uint8_t *data, uint8_t dataLength, uint8_t index) {
  uint8_t result;
  switch (index) {
    case 0:
      // data size
      resetCrc();
      if (dataLength < (packetSize - SRF_CONTROL_BYTES)) {
        // if available data does not fill the packet
        // length = data length + network id (2)
        result = dataLength + 2;
      } else {
        // length = packet size - (crc (2) + length byte (1) = 3)
        result = packetSize - 3;
      }
      break;

    case 1:
      result = (networkId >> 8) & 0xFF;
      break;

    case 2:
      result = networkId & 0xFF;
      break;

    default:
      uint8_t n = min(dataLength, packetSize - SRF_CONTROL_BYTES);
      if ((index - INDEX_PAYLOAD) < n) {
        // data bytes
        result = data[index - INDEX_PAYLOAD];
      } else {
        // crc and padding
        uint8_t i = index - INDEX_PAYLOAD - n;
        if (i == 0) {
          return ((crc >> 8) & 0xFF) ^ whiteningMask[index];
        } else if (i == 1) {
          return (crc & 0xFF) ^ whiteningMask[index];
        } else {
          // padding, not included in crc
          return whiteningMask[index];
        }
      }
      break;
  }
  updateCrc(result);
  return result ^ whiteningMask[index];
}

void RFM69_SRF::setPacketSize(uint8_t packetSize) {
  if (packetSize > MAX_PAYLOAD_LENGTH) {
    packetSize = MAX_PAYLOAD_LENGTH;
  }
  RFM69Base::setPacketSize(packetSize + SRF_CONTROL_BYTES);
}

void RFM69_SRF::send(const char *data, uint8_t dataLength) {
  if (settingsChanged) {
    applySettings();
  }

  sendPacket((const uint8_t *)data, dataLength);
}

void RFM69_SRF::send(String data) {
  send(data.c_str(), data.length());
}

void RFM69_SRF::setDefaultSettings() {
  RFM69Base::setDefaultSettings();

  // default packet size
  setPacketSize(12);
}

void RFM69_SRF::setBaseFrequencyHz(uint32_t baseFrequencyHz) {
  this->baseFrequencyHz = baseFrequencyHz;
  settingsChanged = true;
}

void RFM69_SRF::setDataRateBps(uint32_t dataRateBps) {
  RFM69Base::setDataRateBps(dataRateBps);
  setFrequencyDeviationHz(dataRateBps / 2);
  setRxBandwidthHz(dataRateBps / 2);
}
