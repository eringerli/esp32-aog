// MIT License
//
// Copyright (c) 2019 Christian Riggenbach
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef MAIN_HPP
#define MAIN_HPP

#include <WiFi.h>
#include <WiFiMulti.h>
#include <Wire.h>
#include <HTTPClient.h>
#include <AsyncUDP.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Preferences.h>


extern SemaphoreHandle_t i2cMutex;

struct Status {
  enum class Network : uint8_t {
    disconnected,
    connecting,
    connected,
    accessPoint
  } networkStatus = Network::disconnected;

  enum class Hardware : uint8_t {
    unknown,
    ok,
    error
  } hardwareStatus = Hardware::unknown;
  byte statusPort = 255;
};
extern Status status;
extern Preferences preferences;

extern HardwareSerial usb;
extern HardwareSerial gps1;
extern SoftwareSerial gps2;
extern HardwareSerial rs232;


///////////////////////////////////////////////////////////////////////////
// ioAccess
///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////
// external Libraries
///////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////
// Threads
///////////////////////////////////////////////////////////////////////////
void statusLedWorker( void* z );
void statusWebWorker( void* z );

///////////////////////////////////////////////////////////////////////////
// Helper Functions
///////////////////////////////////////////////////////////////////////////

#endif
