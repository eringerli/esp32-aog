// MIT License
//
// Copyright (c) 2020 Christian Riggenbach
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include "CborQueueSelector.hpp"
#include "average.hpp"

#include <AsyncUDP.h>
#include <ESPUI.h>
#include <HTTPClient.h>
#include <Stream.h>
#include <WiFi.h>
#include <WiFiMulti.h>

extern CborQueueSelector cborQueueSelector;

extern ControlTreeIterator labelLoad;
extern ControlTreeIterator labelOrientation;
extern ControlTreeIterator labelWheelAngle;
extern ControlTreeIterator textNmeaToSend;

extern ControlTreeIterator labelStatusOutput;
extern ControlTreeIterator labelStatusAdc;
extern ControlTreeIterator labelStatusCan;
extern ControlTreeIterator labelStatusImu;
extern ControlTreeIterator labelStatusInclino;
extern ControlTreeIterator labelStatusGps;
extern ControlTreeIterator labelStatusNtrip;
extern ControlTreeIterator labelStatusRtos;

extern SemaphoreHandle_t i2cMutex;

///////////////////////////////////////////////////////////////////////////
// Configuration
///////////////////////////////////////////////////////////////////////////

struct SteerConfig {
  enum class Gpio : int8_t {
    Default     = 127,
    None        = -1,
    Esp32Gpio4  = 4,
    Esp32Gpio5  = 5,
    Esp32Gpio12 = 12,
    Esp32Gpio13 = 13,
    Esp32Gpio14 = 14,
    Esp32Gpio15 = 15,
    Esp32Gpio16 = 16,
    Esp32Gpio17 = 17,
    Esp32Gpio18 = 18,
    Esp32Gpio19 = 19,
    Esp32Gpio21 = 21,
    Esp32Gpio22 = 22,
    Esp32Gpio23 = 23,
    Esp32Gpio25 = 25,
    Esp32Gpio26 = 26,
    Esp32Gpio27 = 27,
    Esp32Gpio32 = 32,
    Esp32Gpio33 = 33,
    Esp32Gpio34 = 34,
    Esp32Gpio35 = 35,
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

  enum class Mode : uint8_t { QtOpenGuidance = 0, AgOpenGps = 1 } mode = Mode::AgOpenGps;

  char hostname[24] = "ESP32-QOG";

  char              ssidAp[24]     = "NetzRosegghof3";
  char              passwordAp[24] = "gghof080";
  SteerConfig::Gpio apModePin      = SteerConfig::Gpio::Esp32Gpio13;

  char ssidSta[24]     = "qog";
  char passwordSta[24] = "gghof080";

  uint32_t baudrate = 115200;

  bool enableOTA = false;

  // set to 1  if you want to use Steering Motor + Cytron MD30C Driver
  // set to 2  if you want to use Steering Motor + IBT 2  Driver
  // set to 3  if you want to use IBT 2  Driver + PWM 2-Coil Valve
  // set to 4  if you want to use IBT 2  Driver + Danfoss Valve PVE A/H/M
  enum class OutputType : uint8_t {
    None                = 0,
    SteeringMotorCytron = 1,
    SteeringMotorIBT2,
    HydraulicPwm2Coil,
    HydraulicDanfoss
  } outputType = OutputType::None;

  uint16_t          pwmFrequency = 1000;
  bool              invertOutput = false;
  SteerConfig::Gpio gpioPwm      = SteerConfig::Gpio::Esp32Gpio15;
  SteerConfig::Gpio gpioDir      = SteerConfig::Gpio::Esp32Gpio32;
  SteerConfig::Gpio gpioEn       = SteerConfig::Gpio::Esp32Gpio14;

  bool    allowPidOverwrite           = false;
  double  steeringPidKp               = 20;
  double  steeringPidKi               = 0.5;
  double  steeringPidKd               = 1;
  double  steeringPidAutoBangOnFactor = 2;
  double  steeringPidBangOn           = 40;
  double  steeringPidBangOff          = 0.1;
  uint8_t steeringPidPwmThreshold     = 5;
  uint8_t steeringPidMinPwm           = 20;

  enum class WorkswitchType : uint8_t {
    None = 0,
    Gpio,
    RearHitchPosition,
    FrontHitchPosition,
    RearPtoRpm,
    FrontPtoRpm,
    MotorRpm
  } workswitchType                                         = WorkswitchType::None;
  SteerConfig::Gpio gpioWorkswitch                         = SteerConfig::Gpio::None;
  SteerConfig::Gpio gpioSteerswitch                        = SteerConfig::Gpio::None;
  uint16_t          autoRecogniseSteerGpioAsSwitchOrButton = 500;
  bool              workswitchActiveLow                    = true;
  bool              steerswitchActiveLow                   = true;

  enum class WheelAngleSensorType : uint8_t {
    WheelAngle = 0,
    TieRodDisplacement
  } wheelAngleSensorType = WheelAngleSensorType::WheelAngle;

  uint8_t wheelAngleInput = 0;

  bool  invertWheelAngleSensor    = false;
  float wheelAngleCountsPerDegree = 118;
  float wheelAnglePositionZero    = 5450;

  float wheelAngleOffset = 0;

  SteerConfig::Gpio gpioSDA     = SteerConfig::Gpio::Default;
  SteerConfig::Gpio gpioSCL     = SteerConfig::Gpio::Default;
  uint32_t          i2cBusSpeed = 400000;

  SteerConfig::Gpio gpioSck     = SteerConfig::Gpio::Default;
  SteerConfig::Gpio gpioMiso    = SteerConfig::Gpio::Default;
  SteerConfig::Gpio gpioMosi    = SteerConfig::Gpio::Default;
  uint32_t          spiBusSpeed = 10000000;

  SteerConfig::Gpio gpioAds131m04Cs   = SteerConfig::Gpio::Esp32Gpio4;
  SteerConfig::Gpio gpioAds131m04Drdy = SteerConfig::Gpio::Esp32Gpio34;

  SteerConfig::Gpio gpioAccGyroCs   = SteerConfig::Gpio::Esp32Gpio33;
  SteerConfig::Gpio gpioAccGyroDrdy = SteerConfig::Gpio::Esp32Gpio36;
  SteerConfig::Gpio gpioMagCs       = SteerConfig::Gpio::Esp32Gpio13;
  SteerConfig::Gpio gpioMagDrdy     = SteerConfig::Gpio::Esp32Gpio39;

  enum class Ads131m04SampleRate : uint8_t {
    Rate250Sps = 7,
    Rate500Sps = 6,
    Rate1kSps  = 5,
    Rate2kSps  = 4,
    Rate4kSps  = 3,
    Rate8kSps  = 2,
    Rate16kSps = 1
  } ads131m04SampleRate = Ads131m04SampleRate::Rate2kSps;

  float filterCutOffFrequencies[4] = { 5.0f, 5.0f, 5.0f, 5.0f };

  bool invertRoll = false;

  bool              canBusEnabled = false;
  SteerConfig::Gpio canBusRx      = SteerConfig::Gpio::Esp32Gpio21;
  SteerConfig::Gpio canBusTx      = SteerConfig::Gpio::Esp32Gpio27;
  enum class CanBusSpeed : uint16_t {
    Speed250kbs = 250,
    Speed500kbs = 500
  } canBusSpeed = CanBusSpeed::Speed500kbs;

  uint8_t canBusHitchThreshold           = 50;
  uint8_t canBusHitchThresholdHysteresis = 6;

  uint16_t canBusRpmThreshold           = 400;
  uint16_t canBusRpmThresholdHysteresis = 100;

  enum class RtkCorrectionType : uint8_t {
    None  = 0,
    Ntrip = 1,
    udp,
    tcp
  } rtkCorrectionType = RtkCorrectionType::None;

  char     rtkCorrectionServer[48]     = "example.com";
  uint16_t rtkCorrectionPort           = 2101;
  char     rtkCorrectionUsername[24]   = "gps";
  char     rtkCorrectionPassword[24]   = "gps";
  char     rtkCorrectionMountpoint[24] = "STALL";

  char rtkCorrectionNmeaToSend[120] = "";

  uint32_t rtkCorrectionBaudrate = 115200;

  uint8_t ntripPositionSendIntervall = 30;

  enum class SendNmeaDataTo : uint8_t {
    None = 0,
    UDP  = 1,
    TCP,
    Serial,
    Serial1,
    Serial2,
    Bluetooth
  } sendNmeaDataTo = SendNmeaDataTo::None;

  uint16_t sendNmeaDataTcpPort     = 0;
  uint16_t sendNmeaDataUdpPort     = 0;
  uint16_t sendNmeaDataUdpPortFrom = 0;

  uint16_t aogPortSendFrom = 5577;
  uint16_t aogPortListenTo = 8888;
  uint16_t aogPortSendTo   = 9999;

  uint16_t qogPortListenTo = 1337;
  uint16_t qogPortSendTo   = 1338;

  uint16_t qogChannelIdAutosteerEnable    = 1000; // in
  uint16_t qogChannelIdWorkswitch         = 2000;
  uint16_t qogChannelIdSteerswitch        = 2001;
  uint16_t qogChannelIdWheelAngle         = 3000;
  uint16_t qogChannelIdSetpointSteerAngle = 4000; // in
  uint16_t qogChannelIdOrientation        = 5000;
  uint16_t qogChannelIdGpsDataIn          = 6000; // in
  uint16_t qogChannelIdGpsDataOut         = 6001;
  uint16_t qogChannelIdCanRearHitch       = 7000;
  uint16_t qogChannelIdCanFrontHitch      = 7001;
  uint16_t qogChannelIdCanRearPtoRpm      = 7002;
  uint16_t qogChannelIdCanFrontPtoRpm     = 7003;
  uint16_t qogChannelIdCanMotorRpm        = 7004;
  uint16_t qogChannelIdCanWheelbasedSpeed = 7005;

  bool retainWifiSettings = true;
};
extern SteerConfig steerConfig, steerConfigDefaults;

struct Initialisation {
  SteerConfig::OutputType outputType      = SteerConfig::OutputType::None;
  SteerConfig::AnalogIn   wheelAngleInput = SteerConfig::AnalogIn::None;

  uint16_t portSendFrom = 5577;
  uint16_t portListenTo = 8888;
  uint16_t portSendTo   = 9999;

  uint16_t sendNmeaDataUdpPort = 0;

  String rtkCorrectionURL = "";
};
extern Initialisation initialisation;

///////////////////////////////////////////////////////////////////////////
// Global Data
///////////////////////////////////////////////////////////////////////////

struct SteerSettings {
  float    Ko                        = 0.0f; // overall gain
  float    Kp                        = 0.0f; // proportional gain
  float    Ki                        = 0.0f; // integral gain
  float    Kd                        = 0.0f; // derivative gain
  uint8_t  minPWMValue               = 10;
  int      maxIntegralValue          = 20; // max PWM value for integral PID component
  float    wheelAngleCountsPerDegree = 118;
  uint32_t wheelAnglePositionZero    = 0;

  time_t lastPacketReceived = 0;
};
extern SteerSettings steerSettings;

struct SteerSetpoints {
  uint8_t  relais              = 0;
  float    speed               = 0;
  uint16_t distanceFromLine    = 32020;
  double   requestedSteerAngle = 0;

  bool   enabled                       = false;
  float  receivedRoll                  = 0;
  double actualSteerAngle              = 0;
  double wheelAngleCurrentDisplacement = 0;
  double wheelAngleRaw                 = 0;
  float  correction                    = 0;

  time_t lastPacketReceived = 0;
};
extern SteerSetpoints steerSetpoints;

struct SteerCanData {
  float    speed;
  uint16_t motorRpm;
  uint8_t  frontHitchPosition;
  uint8_t  rearHitchPosition;
  uint16_t frontPtoRpm;
  uint16_t rearPtoRpm;
};
extern SteerCanData steerCanData;

struct ImuDataAccGyrMag {
  float acc[3] = { 0 };

  float gyr[3] = { 0 };

  float mag[3] = { 0 };
};

///////////////////////////////////////////////////////////////////////////
// external Libraries
///////////////////////////////////////////////////////////////////////////

extern ESPUIClass ESPUI;

extern AsyncUDP udpSendFrom;

///////////////////////////////////////////////////////////////////////////
// Helper Classes
///////////////////////////////////////////////////////////////////////////
class TCritSect {
public:
  TCritSect() { portENTER_CRITICAL( &mux ); }
  ~TCritSect() { portEXIT_CRITICAL( &mux ); }

private:
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
};

enum class SensorNotifyBits : uint32_t {
  AdcDataReady             = 0x02,
  AccGyroDataReady         = 0x04,
  MagDataReady             = 0x08,
  RefreshFiltersSamplerate = 0x10,
  RefreshPidValues         = 0x20,
};

typedef float AdcFloatingPoint;

extern AdcFloatingPoint adcValues[4];

///////////////////////////////////////////////////////////////////////////
// Threads
///////////////////////////////////////////////////////////////////////////
extern TaskHandle_t sensorWorkerTask;

///////////////////////////////////////////////////////////////////////////
// Helper Functions
///////////////////////////////////////////////////////////////////////////
extern void setResetButtonToRed();

extern void initIdleStats();
extern void initSensors();
extern void calculateMountingCorrection();
extern void initRtkCorrection();
extern void initCan();
extern void initAutosteer();
