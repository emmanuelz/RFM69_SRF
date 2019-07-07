// Copyright Emmanuel ZURMELY <emmanu.zurmely@evc.net> - 07/2019
// This software is distributed under the terms of the GNU General Public License version 3.

#include "RFM69_SRF.h"

#define SERIAL_BPS 115200

RFM69_SRF srf;

void setup() {
  Serial.begin(SERIAL_BPS);
  srf.begin();
  // default settings: 868.3 MHz (ATCH = 5), channel spacing 200 kHz (ATCS = C8), channel 0 (ATCN), pan id 0x5AA5 (ATID), packet size 12 (ATPK = C)
  // uncomment to adjust non-default settings:
  //srf.setFrequencyHz(868300000);
  //srf.setChannelSpacingHz(200000);
  //srf.setChannelNumber(0);
  //srf.setNetworkId(0x5AA5);
  //srf.setPacketSize(12);
  Serial.println(F("Setup done\nListening..."));
}

void loop() {
  // process serial input
  if (Serial.available() > 0) {
    char input = Serial.read();
    if (input == 'r') {
      // display all RFM69 registers' values
      srf.readAllRegs();
    } else if (input == 't') {
      // send a standard packet (12 bytes of payload)
      srf.send("aTEST-------");
      // send a short packet
      srf.send("TEST");
      // send multiple packets
      srf.send("Test sending multiple data packets");
    }
  }

  // process received data
  if (srf.isDataAvailable()) {
    uint8_t l;
    const uint8_t *data = srf.getReceivedData(l);
    Serial.print(F("received bytes: "));
    Serial.println(l);
    Serial.print(F("rssi: "));
    Serial.println(srf.getRSSI());
    Serial.print(F("payload: "));
    for (uint8_t i = 0; i < l; i++) {
      Serial.print((char)data[i]);
    }
    Serial.println();
  }
}