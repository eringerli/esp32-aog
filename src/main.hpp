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

#pragma once

#include <WiFi.h>
#include <WiFiMulti.h>

#include <HTTPClient.h>

#include <AsyncUDP.h>

#include <ESPUI.h>

#include <Adafruit_MMA8451.h>

#include <utility/quaternion.h>

#include "average.hpp"

#include "jsonqueueselector.h"

extern JsonQueueSelector jsonQueueSelector;

extern uint16_t labelLoad;
extern uint16_t labelOrientation;
extern uint16_t labelWheelAngle;
extern uint16_t textNmeaToSend;

extern uint16_t labelStatusOutput;
extern uint16_t labelStatusAdc;
extern uint16_t labelStatusCan;
extern uint16_t labelStatusImu;
extern uint16_t labelStatusInclino;
extern uint16_t labelStatusGps;
extern uint16_t labelStatusNtrip;

extern SemaphoreHandle_t i2cMutex;

///////////////////////////////////////////////////////////////////////////
// Configuration
///////////////////////////////////////////////////////////////////////////

struct SteerConfig {

  enum class Gpio : int8_t {
    Default     = -1,
    None        = 0,
    Esp32Gpio4  = 4,
    Esp32Gpio5  = 5,
    Esp32Gpio12 = 12,
    Esp32Gpio13 = 13,
    Esp32Gpio14 = 14,
    Esp32Gpio15 = 15,
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

  enum class Mode : uint8_t {
    QtOpenGuidance = 0,
    AgOpenGps = 1
  } mode = Mode::AgOpenGps;

  char ssid[24] = "NetzRosegghof3";
  char password[24] = "gghof080";
  char hostname[24] = "ESP32-QOG";
  SteerConfig::Gpio apModePin = SteerConfig::Gpio::Esp32Gpio13;

  uint32_t baudrate = 115200;

  bool enableOTA = false;

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

  bool allowPidOverwrite = false;
  double steeringPidKp = 20;
  double steeringPidKi = 0.5;
  double steeringPidKd = 1;
  double steeringPidAutoBangOnFactor = 2;
  double steeringPidBangOn = 40;
  double steeringPidBangOff = 0.1;
//   uint16_t steeringPidDflTurnIdOff = 40;
  uint8_t steeringPidMinPwm = 20;


  enum class WorkswitchType : uint8_t {
    None = 0,
    Gpio,
    RearHitchPosition,
    FrontHitchPosition,
    RearPtoRpm,
    FrontPtoRpm,
    MotorRpm
  } workswitchType = WorkswitchType::None;
  SteerConfig::Gpio gpioWorkswitch = SteerConfig::Gpio::None;
  SteerConfig::Gpio gpioSteerswitch = SteerConfig::Gpio::None;
  uint16_t autoRecogniseSteerGpioAsSwitchOrButton = 500;
  bool workswitchActiveLow = true;
  bool steerswitchActiveLow = true;

  enum class WheelAngleSensorType : uint8_t {
    WheelAngle = 0,
    TieRodDisplacement
  } wheelAngleSensorType = WheelAngleSensorType::WheelAngle;

  SteerConfig::AnalogIn wheelAngleInput = SteerConfig::AnalogIn::None;

  bool invertWheelAngleSensor = false;
  float wheelAngleCountsPerDegree = 118;
  uint16_t wheelAnglePositionZero = 5450;

  float wheelAngleOffset = 0;

  float wheelAngleFirstArmLenght = 92;
  float wheelAngleSecondArmLenght = 308;
  float wheelAngleTieRodStroke = 210;
  float wheelAngleMinimumAngle = 37;
  float wheelAngleTrackArmLenght = 165;

  SteerConfig::Gpio gpioSDA = SteerConfig::Gpio::Default;
  SteerConfig::Gpio gpioSCL = SteerConfig::Gpio::Default;
  uint32_t i2cBusSpeed = 400000;
  enum class ImuType : uint8_t {
    None = 0,
//     BNO055 = 1,
    Fxos8700Fxas21002 = 2
  } imuType = ImuType::None;

  enum class InclinoType : uint8_t {
    None = 0,
    MMA8451 = 1,
    DOGS2,
    Fxos8700Fxas21002
  } inclinoType = InclinoType::None;

  bool invertRoll = false;

  float mountCorrectionImuRoll = 0;
  float mountCorrectionImuPitch = 0;
  float mountCorrectionImuYaw = 0;

  bool canBusEnabled = false;
  SteerConfig::Gpio canBusRx = SteerConfig::Gpio::Esp32Gpio26;
  SteerConfig::Gpio canBusTx = SteerConfig::Gpio::Esp32Gpio25;
  enum class CanBusSpeed : uint16_t {
    Speed250kbs = 250,
    Speed500kbs = 500
  } canBusSpeed = CanBusSpeed::Speed500kbs;

  uint8_t canBusHitchThreshold = 50;
  uint8_t canBusHitchThresholdHysteresis = 6;

  uint16_t canBusRpmThreshold = 400;
  uint16_t canBusRpmThresholdHysteresis = 100;

  enum class RtkCorrectionType : uint8_t {
    None = 0,
    Ntrip = 1,
    udp,
    tcp
  } rtkCorrectionType = RtkCorrectionType::None;

  char rtkCorrectionServer[48] = "example.com";
  uint16_t rtkCorrectionPort = 2101;
  char rtkCorrectionUsername[24] = "gps";
  char rtkCorrectionPassword[24] = "gps";
  char rtkCorrectionMountpoint[24] = "STALL";

  char rtkCorrectionNmeaToSend[120] = "";

  uint32_t rtkCorrectionBaudrate = 115200;

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

  uint16_t sendNmeaDataTcpPort = 0;
  uint16_t sendNmeaDataUdpPort = 0;
  uint16_t sendNmeaDataUdpPortFrom = 0;

  uint16_t aogPortSendFrom = 5577;
  uint16_t aogPortListenTo = 8888;
  uint16_t aogPortSendTo = 9999;

  uint16_t qogPortListenTo = 1337;
  uint16_t qogPortSendTo = 1338;

  uint16_t qogChannelIdAutosteerEnable = 1000;    // in
  uint16_t qogChannelIdWorkswitch = 2000;
  uint16_t qogChannelIdSteerswitch = 2001;
  uint16_t qogChannelIdWheelAngle = 3000;
  uint16_t qogChannelIdSetpointSteerAngle = 4000; // in
  uint16_t qogChannelIdOrientation = 5000;
  uint16_t qogChannelIdGpsDataIn = 6000;          // in
  uint16_t qogChannelIdGpsDataOut = 6001;
  uint16_t qogChannelIdCanRearHitch = 7000;
  uint16_t qogChannelIdCanFrontHitch = 7001;
  uint16_t qogChannelIdCanRearPtoRpm = 7002;
  uint16_t qogChannelIdCanFrontPtoRpm = 7003;
  uint16_t qogChannelIdCanMotorRpm = 7004;
  uint16_t qogChannelIdCanWheelbasedSpeed = 7005;

  bool retainWifiSettings = true;
};
extern SteerConfig steerConfig, steerConfigDefaults;

struct Fxos8700Fxas21002CalibrationData {

  Fxos8700Fxas21002CalibrationData() {
    mag_offsets[0] = -13.56f;
    mag_offsets[1] = -11.98f;
    mag_offsets[2] = -85.02f;

    mag_softiron_matrix[0][0] =  0.998;
    mag_softiron_matrix[0][1] = -0.048;
    mag_softiron_matrix[0][2] = -0.009;
    mag_softiron_matrix[1][0] = -0.048;
    mag_softiron_matrix[1][1] =  1.022;
    mag_softiron_matrix[1][2] =  0.016;
    mag_softiron_matrix[2][0] = -0.009;
    mag_softiron_matrix[2][1] =  0.016;
    mag_softiron_matrix[2][2] =  0.983;

    mag_field_strength = 53.21f;

    gyro_zero_offsets[0] = 0;
    gyro_zero_offsets[1] = 0;
    gyro_zero_offsets[2] = 0;
  };

  // Offsets applied to raw x/y/z mag values
  float mag_offsets[3];

  // Soft iron error compensation matrix
  float mag_softiron_matrix[3][3];

  float mag_field_strength;

  // Offsets applied to compensate for gyro zero-drift error for x/y/z
  float gyro_zero_offsets[3];
};
extern Fxos8700Fxas21002CalibrationData fxos8700Fxas21002CalibrationData, fxos8700Fxas21002CalibrationDefault;

struct Initialisation {
  SteerConfig::OutputType outputType = SteerConfig::OutputType::None;
  SteerConfig::AnalogIn wheelAngleInput = SteerConfig::AnalogIn::None;
  SteerConfig::ImuType imuType = SteerConfig::ImuType::None;
  SteerConfig::InclinoType inclinoType = SteerConfig::InclinoType::None;

  uint16_t portSendFrom = 5577;
  uint16_t portListenTo = 8888;
  uint16_t portSendTo = 9999;

  uint16_t sendNmeaDataUdpPort = 0;

  String rtkCorrectionURL = "";
};
extern Initialisation initialisation;


///////////////////////////////////////////////////////////////////////////
// Global Data
///////////////////////////////////////////////////////////////////////////

struct SteerSettings {
  float Ko = 0.0f;  //overall gain
  float Kp = 0.0f;  //proportional gain
  float Ki = 0.0f;//integral gain
  float Kd = 0.0f;  //derivative gain
  uint8_t minPWMValue = 10;
  int maxIntegralValue = 20; //max PWM value for integral PID component
  float wheelAngleCountsPerDegree = 118;
  uint16_t wheelAnglePositionZero = 0;

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
  double wheelAngleCurrentDisplacement = 0;
  double wheelAngleRaw = 0;
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
  bool sendCalibrationDataFromImu = false;

  float heading;
  float roll;
  float pitch;

  imu::Quaternion orientation;
};
extern SteerImuInclinometerData steerImuInclinometerData;

struct SteerCanData {
  float speed;
  uint16_t motorRpm;
  uint8_t frontHitchPosition;
  uint8_t rearHitchPosition;
  uint16_t frontPtoRpm;
  uint16_t rearPtoRpm;
};
extern SteerCanData steerCanData;

///////////////////////////////////////////////////////////////////////////
// external Libraries
///////////////////////////////////////////////////////////////////////////

extern ESPUIClass ESPUI;

// extern AsyncUDP udpLocalPort;
// extern AsyncUDP udpRemotePort;
extern AsyncUDP udpSendFrom;

extern Adafruit_MMA8451 mma;

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
extern void setResetButtonToRed();

extern void initIdleStats();
extern void initSensors();
extern void calculateMountingCorrection();
extern void initRtkCorrection();
extern void initCan();
extern void initAutosteer();
