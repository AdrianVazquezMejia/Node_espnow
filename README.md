# Node espnow
This is a code construction of a Mesh espnow node that use MODBUS RTU

![](https://github.com/AdrianVazquezMejia/Node_espnow/workflows/C/badge.svg)

# Description

This is the source code of a Wi-Fi mesh's node based on ESP-32 MCU. This networks was designed to work along side Modbus
RTU over serial line. Each node has a serial interface which is used to connect itself with Modbus devices and to integrate them into the network.

# Installation

* Download a stable realase, I recommend `v3.0` using  `git clonehttps://github.com/AdrianVazquezMejia/Node_espnow.git`
* Go to the folder `cd ../Node_espnow`
* Build the menuconfig and set the serial port `make menuconfig`
* Compile `make all`
* Flash the MCU `make flash`
* Set to factory default keeping pressed BOOT button for 5 s.

# License

MIT License

Copyright (c) [2020] [Adrian Vazquez]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
