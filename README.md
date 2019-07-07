# RFM69_SRF Library
[![GitHub release](https://img.shields.io/github/release/emmanuelz/RFM69_SRF.svg)](https://github.com/emmanuelz/RFM69_SRF/releases)
[![GitHub issues](https://img.shields.io/github/issues/emmanuelz/RFM69_SRF.svg)](https://github.com/emmanuelz/RFM69_SRF/issues)
[![GitHub pull requests](https://img.shields.io/github/issues-pr/emmanuelz/RFM69_SRF.svg)](https://github.com/emmanuelz/RFM69_SRF/pulls)
[![license](https://img.shields.io/github/license/emmanuelz/RFM69_SRF.svg)](https://github.com/emmanuelz/RFM69_SRF/blob/master/license.txt)


By Emmanuel ZURMELY,
<br/>
Arduino communication library between RFM69 transceivers and discontinued Ciseco SRF / XRF / URF modules based on TI CC1110 and TI CC1111 MCUs.
<br/>

## License
GPL 3.0, please see the [license.txt](https://github.com/emmanuelz/RFM69_SRF/blob/master/license.txt) file for details. Be sure to include the same license with any fork or redistribution of this library.

## Features
- receive from and send to Ciseco SRF, XRF or URF devices
- support for configurable settings of Ciseco devices: base frequency (ATCH), channel spacing (ATCS), channel number (ATCS), pan id (ATID), packet size (ATPK)
- default settings: ATCH=5 (868.3 MHz), ATCS=C8 (200 kHz), ATCN=0, ATID=5AA5, ATPK=C (12 bytes)
- ability to read signal strength (RSSI)
- automatically split sent data into packets
- tested with default configuration with XRF and URF nodes

### Library Installation (Arduino IDE)
Copy the content of this library in the "Arduino/libraries/RFM69_SRF" folder.
<br />
To find your Arduino folder go to File>Preferences in the Arduino IDE.
<br/>
See [this tutorial](http://learn.adafruit.com/arduino-tips-tricks-and-techniques/arduino-libraries) on Arduino libraries.

### Basic sample usage
- The [SimpleTest](https://github.com/emmanuelz/RFM69_SRF/blob/master/examples/SimpleTest/SimpleTest.ino) example listens for incoming data and sends text of different sizes when receiving 't' from the serial console.
