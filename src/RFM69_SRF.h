// Copyright Emmanuel ZURMELY <emmanu.zurmely@evc.net> - 07/2019
// This software is distributed under the terms of the GNU General Public License version 3.

#ifndef RFM69_SRF_H
#define RFM69_SRF_H

#include <Arduino.h>

#include "RFM69Base.h"

// control bytes: data length, network id, crc
#define SRF_CONTROL_BYTES 5
// data bytes (counted in the length field): network id
#define SRF_DATA_BYTES (SRF_CONTROL_BYTES - 3)
#define MAX_PAYLOAD_LENGTH (MAX_DATA_LENGTH - SRF_CONTROL_BYTES)

#define INDEX_SIZE 0
#define INDEX_NETWORKID 1
#define INDEX_PAYLOAD 3

class RFM69_SRF : public RFM69Base {
  private:
    uint8_t whiteningMask[MAX_DATA_LENGTH];
    uint16_t crc;
    uint16_t lfsr;
    uint16_t networkId = 0x5AA5;
    uint32_t baseFrequencyHz = 868300000;
    uint32_t channelSpacingHz = 200000;
    uint8_t channelNumber = 0;
    bool settingsChanged;
    uint8_t receivedDataLength;

    void applySettings();
    void applyWhitening();
    void computeCrc();
    bool checkReceivedData();
    uint8_t getNextWhiteningByte();
    void initializeWhiteningMask();
    bool isValidCrc();
    bool isValidNetworkId();
    void resetCrc();
    void resetWhiteningRegister();
    uint8_t revertBits(uint8_t value);
    void updateCrc(uint8_t value);

  protected:
    virtual uint8_t getByteToSend(const uint8_t *data, uint8_t dataLength, uint8_t index);
    virtual uint8_t getPayloadLength();
    virtual void initialize();
    virtual void setDefaultSettings();

  public:
    RFM69_SRF(uint8_t pinSlaveSelect = RFM69_SPI_CS, uint8_t pinInterruption = RFM69_IRQ_PIN);
    uint16_t getCrc() const;
    const uint8_t* getReceivedData(uint8_t &dataLength) const;
    virtual bool isDataAvailable();
    void setBaseFrequencyHz(uint32_t baseFrequencyHz);
    void setChannelNumber(uint8_t channelNumber);
    void setChannelSpacingHz(uint32_t channelSpacingHz);
    virtual void setDataRateBps(uint32_t dataRateBps);
    void setNetworkId(uint16_t networkId);
    virtual void setPacketSize(uint8_t packetSize);
    void send(const char *data, uint8_t dataLength);
    void send(String data);
};

#endif // RFM69_SRF_H
