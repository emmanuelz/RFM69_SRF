// Copyright Emmanuel ZURMELY <emmanu.zurmely@evc.net> - 07/2019
// This software is distributed under the terms of the GNU General Public License version 3.

#ifndef RFM69BASE_H
#define RFM69BASE_H

#include <SPI.h>

#define MAX_DATA_LENGTH 66

// SPI CS pin, arduino SS by default
#define RFM69_SPI_CS SS
// RFM69 IRQ pin, arduino D2 by default
#define RFM69_IRQ_PIN  2

enum class RFM69BaseError {
  NoError,
  ErrorInitialization
};

class RFM69Base {
  private:
    SPISettings spiSettings;
    uint8_t pinInterruption;
    uint8_t pinSlaveSelect;
    uint8_t mode;
    bool useEncryption;
    RFM69BaseError error = RFM69BaseError::NoError;
    uint8_t rssi;

    static void isr0();
    static RFM69Base *instance;

    bool canSend();
    void initializePins();
    void initializeRadio();
    void readReceivedData();
    uint8_t readReg(uint8_t addr);
    void select();
    uint8_t sendFrame(const uint8_t *data, uint8_t dataLength);
    void setMode(uint8_t newMode);
    void unselect();
    void writeReg(uint8_t addr, uint8_t value);

  protected:
    uint8_t buffer[MAX_DATA_LENGTH];
    uint8_t packetSize;
    bool dataReceived;

    int8_t readRSSI(bool forceTrigger = false);
    void sendPacket(const uint8_t *data, uint8_t dataLength);

    virtual uint8_t getByteToSend(const uint8_t *data, uint8_t dataLength, uint8_t index);
    virtual uint8_t getPayloadLength();
    virtual void initialize();
    virtual void receiveBegin();
    virtual void setDefaultSettings();
    virtual void setDefaultEncryptionKey();

  public:
    RFM69Base(uint8_t pinSlaveSelect = RFM69_SPI_CS, uint8_t pinInterruption = RFM69_IRQ_PIN);
    void begin();
    RFM69BaseError getError() const;
    uint32_t getFrequencyHz();
    uint8_t getPowerLevelDBm();
    int8_t getRSSI() const;
    virtual bool isDataAvailable();
    void readAllRegs();
    void setCurrentLimit(int8_t currentMA);
    virtual void setDataRateBps(uint32_t dataRateKbps);
    void setEncryptionKey(const char *key);
    void setFrequencyDeviationHz(uint32_t deviationHz);
    void setFrequencyHz(uint32_t baseFrequencyHz);
    virtual void setPacketSize(uint8_t packetSize);
    void setPowerLevelDBm(int8_t powerDBm);
    void setRxBandwidthHz(uint32_t bandWidthHz);
    void sleep();
    void wake();
};

#endif // RFM69BASE_H
