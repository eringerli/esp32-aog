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

#include <WiFi.h>
#include <WiFiMulti.h>

#include <HTTPClient.h>

#include <AsyncUDP.h>

#include <EEPROM32_Rotate.h>
#include <ESPUI.h>

#include <Adafruit_MMA8451.h>
#include <Adafruit_BNO055.h>

#include "average.hpp"

#ifndef MAIN_HPP
#define MAIN_HPP

extern uint16_t labelLoad;
extern uint16_t labelHeading;
extern uint16_t labelRoll;
extern uint16_t labelWheelAngle;
extern uint16_t textNmeaToSend;

extern uint16_t labelStatusOutput;
extern uint16_t labelStatusAdc;
extern uint16_t labelStatusImu;
extern uint16_t labelStatusInclino;
extern uint16_t labelStatusGps;
extern uint16_t labelStatusNtrip;

extern SemaphoreHandle_t i2cMutex;

///////////////////////////////////////////////////////////////////////////
// Configuration
///////////////////////////////////////////////////////////////////////////

struct SteerConfig {

  enum class Gpio : uint8_t {
    None        = 0,
    Esp32Gpio4  = 4,
    Esp32Gpio12 = 12,
    Esp32Gpio13 = 13,
    Esp32Gpio14 = 14,
    Esp32Gpio15 = 15,
    Esp32Gpio21 = 21,
    Esp32Gpio25 = 25,
    Esp32Gpio26 = 26,
    Esp32Gpio27 = 27,
    Esp32Gpio32 = 32,
    Esp32Gpio33 = 33,
    Esp32Gpio34 = 34,
    Esp32Gpio36 = 36,
    Esp32Gpio39 = 39
  };

  enum class AnalogIn : uint8_t {
    None                    = 0,
    Esp32GpioA2             = 2,
    Esp32GpioA3             = 3,
    Esp32GpioA4             = 4,
    Esp32GpioA7             = 7,
    Esp32GpioA9             = 9,
    Esp32GpioA12            = 12,
    ADS1115A0Single         = 100,
    ADS1115A1Single         = 101,
    ADS1115A2Single         = 102,
    ADS1115A3Single         = 103,
    ADS1115A0A1Differential = 200,
    ADS1115A2A3Differential = 202
  };

  char ssid[24] = "NetzRosegghof3";
  char password[24] = "gghof080";
  char hostname[24] = "ESP32-AGO";

  //set to 1  if you want to use Steering Motor + Cytron MD30C Driver
  //set to 2  if you want to use Steering Motor + IBT 2  Driver
  //set to 3  if you want to use IBT 2  Driver + PWM 2-Coil Valve
  //set to 4  if you want to use IBT 2  Driver + Danfoss Valve PVE A/H/M
  enum class OutputType : uint8_t {
    None = 0,
    SteeringMotorCytron = 1,
    SteeringMotorIBT2,
    HydraulicPwm2Coil,
    HydraulicDanfoss
  } outputType = OutputType::None;

  uint16_t pwmFrequency = 1000;
  bool invertOutput = false;
  SteerConfig::Gpio gpioPwm = SteerConfig::Gpio::Esp32Gpio15;
  SteerConfig::Gpio gpioDir = SteerConfig::Gpio::Esp32Gpio32;
  SteerConfig::Gpio gpioEn = SteerConfig::Gpio::Esp32Gpio14;

  bool allowPidOverwrite = true;
  double steeringPidKp = 5;
  double steeringPidKi = 0;
  double steeringPidKd = 0;
  double steeringPidBangOn = 20;
  double steeringPidBangOff = 0.5;
  uint16_t steeringPidDflTurnIdOff = 40;

  SteerConfig::Gpio gpioWorkswitch = SteerConfig::Gpio::None;
  SteerConfig::Gpio gpioSteerswitch = SteerConfig::Gpio::None;
  bool autosteerButton = true;

  SteerConfig::AnalogIn analogInWheelAngleSensor = SteerConfig::AnalogIn::None;
  bool allowWheelAngleCenterAndCountsOverwrite = false;
  bool invertWheelAngleSensor = false;
  float steerSensorCountsPerDegree = 118;
  uint16_t steeringPositionZero = 13333;

  bool steeringWheelEncoder = false;
  SteerConfig::Gpio gpioWheelencoderA = SteerConfig::Gpio::None;
  SteerConfig::Gpio gpioWheelencoderB = SteerConfig::Gpio::None;

  uint8_t pulseCountMax = 3;



  enum class ImuType : uint8_t {
    None = 0,
    BNO055 = 1
  } imuType = ImuType::None;

  enum class InclinoType : uint8_t {
    None = 0,
    MMA8451 = 1,
    DOGS2
  } inclinoType = InclinoType::None;

  float rollOffset = 0;


  enum class RtkCorrectionType : uint8_t {
    None = 0,
    Ntrip = 1,
    udp,
    tcp
  } rtkCorrectionType = RtkCorrectionType::None;

  char rtkCorrectionURL[120] = "http://gps:eringer@192.168.11.63:2101/STALL";
  char rtkCorrectionNmeaToSend[120] = "";

  uint32_t rtkCorrectionBaudrate = 38400;

  uint8_t ntripPositionSendIntervall = 30;

  enum class SendNmeaDataTo : uint8_t {
    None = 0,
    UDP = 1,
    TCP,
    Serial,
    Serial1,
    Serial2,
    Bluetooth
  } sendNmeaDataTo = SendNmeaDataTo::None;

  uint16_t portSendFrom = 5577;
  uint16_t portListenTo = 8888;
  uint16_t portSendTo = 9999;

};
extern SteerConfig steerConfig;

extern adafruit_bno055_offsets_t bno055CalibrationData;

struct Initialisation {
  SteerConfig::OutputType outputType = SteerConfig::OutputType::None;
  SteerConfig::AnalogIn analogInWheelAngleSensor = SteerConfig::AnalogIn::None;
  SteerConfig::ImuType imuType = SteerConfig::ImuType::None;
  SteerConfig::InclinoType inclinoType = SteerConfig::InclinoType::None;

  uint16_t portSendFrom = 5577;
  uint16_t portListenTo = 8888;
  uint16_t portSendTo = 9999;
};
extern Initialisation initialisation;


///////////////////////////////////////////////////////////////////////////
// Global Data
///////////////////////////////////////////////////////////////////////////

enum class EepromAddresses : uint16_t {
  CRC = 0,
  Validator = 5,
  SizeOfConfig = 7,
  SteerSettings = 9,
  Bno055CalibrationData = SteerSettings + sizeof( SteerConfig )
};

struct SteerSettings {
  float Ko = 0.0f;  //overall gain
  float Kp = 0.0f;  //proportional gain
  float Ki = 0.0f;//integral gain
  float Kd = 0.0f;  //derivative gain
  uint8_t minPWMValue = 10;
  int maxIntegralValue = 20; //max PWM value for integral PID component
  float steerSensorCountsPerDegree = 118;
  int16_t steeringPositionZero = 0;

  time_t lastPacketReceived = 0;
};
extern SteerSettings steerSettings;

struct SteerSetpoints {
  uint8_t relais = 0;
  float speed = 0;
  uint16_t distanceFromLine = 32020;
  double requestedSteerAngle = 0;

  bool enabled = false;
  float receivedRoll = 0;
  double actualSteerAngle = 0;
  float correction = 0;

  time_t lastPacketReceived = 0;
};
extern SteerSetpoints steerSetpoints;

struct SteerMachineControl {
  uint8_t pedalControl = 0;
  float speed = 0;
  uint8_t relais = 0;
  uint8_t youTurn = 0;

  time_t lastPacketReceived = 0;
};
extern SteerMachineControl steerMachineControl;

struct SteerImuInclinometerData {
  Average<float, float, 10> bnoAverageHeading;

  float roll;
  float pitch;
};
extern SteerImuInclinometerData steerImuInclinometerData;




///////////////////////////////////////////////////////////////////////////
// external Libraries
///////////////////////////////////////////////////////////////////////////

extern ESPUIClass ESPUI;
extern EEPROM32_Rotate EEPROM;

// extern AsyncUDP udpLocalPort;
// extern AsyncUDP udpRemotePort;
extern AsyncUDP udpSendFrom;

extern Adafruit_MMA8451 mma;
extern Adafruit_BNO055 bno;

///////////////////////////////////////////////////////////////////////////
// Helper Classes
///////////////////////////////////////////////////////////////////////////
extern portMUX_TYPE mux;
class TCritSect {
    TCritSect() {
      portENTER_CRITICAL( &mux );
    }
    ~TCritSect() {
      portEXIT_CRITICAL( &mux );
    }
};


///////////////////////////////////////////////////////////////////////////
// Threads
///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////
// Helper Functions
///////////////////////////////////////////////////////////////////////////

extern void writeEeprom();
extern void initIdleStats();
extern void initSensors();
extern void initRtkCorrection();
extern void initAutosteer();

#endif
