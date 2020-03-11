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

#include <memory>

#include <FS.h>
#include <SPIFFS.h>

#include "main.hpp"
#include "settings.hpp"

void loadSavedConfig() {
  auto j = loadJsonFromFile( "/config.json" );
  parseJsonToSteerConfig( j, steerConfig );
}

void saveConfig() {
  const auto j = parseSteerConfigToJson( steerConfig );
  saveJsonToFile( j, "/config.json" );
}

json loadJsonFromFile( const char* fileName ) {
  json j;

  if( SPIFFS.exists( fileName ) ) {
    File file = SPIFFS.open( fileName, "r" );

    if( file ) {
      std::vector<uint8_t> data;
      data.resize( file.size() );

      file.read( data.data(), file.size() );

      try {
        j = json::parse( data/*, nullptr, false*/ );
      } catch( json::exception& e ) {
        // output exception information
        Serial.print( "message: " );
        Serial.println( e.what() );
        Serial.print( "exception id: " );
        Serial.println( e.id );
      }

      file.close();
    }
  }

  return j;
}

void saveJsonToFile( const json& json, const char* fileName ) {
  // pretty print with 2 spaces indentation
  auto data = json.dump( 2 );

  File file = SPIFFS.open( fileName, "w" );

  if( file && !file.isDirectory() ) {
    file.write( ( uint8_t* )data.c_str(), data.size() );
  }

  file.close();
}

json parseSteerConfigToJson( const SteerConfig& config ) {
  json j;

  j["wifi"]["ssid"] = config.ssid;
  j["wifi"]["password"] = config.password;
  j["wifi"]["hostname"] = config.hostname;
  j["wifi"]["apModePin"] = int( config.apModePin );
  j["wifi"]["retainSettings"] = config.retainWifiSettings;

  j["output"]["type"] = int( config.outputType );
  j["output"]["pwmFrequency"] = config.pwmFrequency;
  j["output"]["minPWM"] = config.steeringPidMinPwm;
  j["output"]["gpioPwm"] = int( config.gpioPwm );
  j["output"]["gpioDir"] = int( config.gpioDir );
  j["output"]["gpioEn"] = int( config.gpioEn );

  j["PID"]["P"] = config.steeringPidKp;
  j["PID"]["I"] = config.steeringPidKi;
  j["PID"]["D"] = config.steeringPidKd;
  j["PID"]["autoBangOnFactor"] = config.steeringPidAutoBangOnFactor;
  j["PID"]["bangOn"] = config.steeringPidBangOn;
  j["PID"]["bangOff"] = config.steeringPidBangOff;

  j["workswitch"]["workswitchType"] = int( config.workswitchType );
  j["workswitch"]["gpioWorkswitch"] = int( config.gpioWorkswitch );
  j["workswitch"]["gpioSteerswitch"] = int( config.gpioSteerswitch );
  j["workswitch"]["msAutoRecogniseSteerGpioAsSwitch"] = config.autoRecogniseSteerGpioAsSwitchOrButton;
  j["workswitch"]["workswitchActiveLow"] = config.workswitchActiveLow;
  j["workswitch"]["steerswitchActiveLow"] = config.steerswitchActiveLow;

  j["wheelangle"]["input"] = config.wheelAngleInput;
  j["wheelangle"]["sensorType"] = int( config.wheelAngleSensorType );
  j["wheelangle"]["invert"] = config.invertWheelAngleSensor;
  j["wheelangle"]["countsPerDegree"] = config.wheelAngleCountsPerDegree;
  j["wheelangle"]["positionZero"] = config.wheelAnglePositionZero;

  j["wheelangle"]["tierod"]["offset"] = int( config.wheelAngleInput );
  j["wheelangle"]["tierod"]["FirstArmLenght"] = config.wheelAngleFirstArmLenght;
  j["wheelangle"]["tierod"]["SecondArmLenght"] = config.wheelAngleSecondArmLenght;
  j["wheelangle"]["tierod"]["TieRodStroke"] = config.wheelAngleTieRodStroke;
  j["wheelangle"]["tierod"]["MinimumAngle"] = config.wheelAngleMinimumAngle;
  j["wheelangle"]["tierod"]["TrackArmLenght"] = config.wheelAngleTrackArmLenght;

  j["i2c"]["sda"] = int( config.gpioSDA );
  j["i2c"]["scl"] = int( config.gpioSCL );
  j["i2c"]["speed"] = config.i2cBusSpeed;

  j["imu"]["type"] = int( config.imuType );

  j["inclinomeer"]["type"] = int( config.inclinoType );
  j["inclinomeer"]["invertRoll"] = config.invertRoll;

  j["mountingCorrection"]["roll"] = config.mountCorrectionImuRoll;
  j["mountingCorrection"]["pitch"] = config.mountCorrectionImuPitch;
  j["mountingCorrection"]["yaw"] = config.mountCorrectionImuYaw;

  j["canBus"]["enabled"] = config.canBusEnabled;
  j["canBus"]["rxPin"] = int( config.canBusRx );
  j["canBus"]["txPin"] = int( config.canBusTx );
  j["canBus"]["speed"] = int( config.canBusSpeed );
  j["canBus"]["hitchThreshold"] = config.canBusHitchThreshold;
  j["canBus"]["hitchThresholdHysteresis"] = config.canBusHitchThresholdHysteresis;
  j["canBus"]["rpmThreshold"] = config.canBusRpmThreshold;
  j["canBus"]["rpmThresholdHysteresis"] = config.canBusRpmThresholdHysteresis;

  j["gps"]["correctionSource"] = int( config.rtkCorrectionType );
  j["gps"]["ntrip"]["server"] = config.rtkCorrectionServer;
  j["gps"]["ntrip"]["port"] = config.rtkCorrectionPort;
  j["gps"]["ntrip"]["username"] = config.rtkCorrectionUsername;
  j["gps"]["ntrip"]["password"] = config.rtkCorrectionPassword;
  j["gps"]["ntrip"]["mountpoint"] = config.rtkCorrectionMountpoint;
  j["gps"]["ntrip"]["NMEAToSend"] = config.rtkCorrectionNmeaToSend;
  j["gps"]["ntrip"]["intervalSendPosition"] = config.ntripPositionSendIntervall;
  j["gps"]["baudrate"] = config.rtkCorrectionBaudrate;
  j["gps"]["outputTo"] = int( config.sendNmeaDataTo );
  j["gps"]["tcpPort"] = config.sendNmeaDataTcpPort;

  j["connection"]["mode"] = int( config.mode );
  j["connection"]["aog"]["sendFrom"] = config.aogPortSendFrom;
  j["connection"]["aog"]["listenTo"] = config.aogPortListenTo;
  j["connection"]["aog"]["sendTo"] = config.aogPortSendTo;

  j["connection"]["qog"]["listenTo"] = config.qogPortListenTo;
  j["connection"]["qog"]["sendTo"] = config.qogPortSendTo;
  j["connection"]["qog"]["channelId"]["Workswitch"] = config.qogChannelIdWorkswitch;
  j["connection"]["qog"]["channelId"]["Steerswitch"] = config.qogChannelIdSteerswitch;
  j["connection"]["qog"]["channelId"]["WheelAngle"] = config.qogChannelIdWheelAngle;
  j["connection"]["qog"]["channelId"]["SetpointSteerAngle"] = config.qogChannelIdSetpointSteerAngle;
  j["connection"]["qog"]["channelId"]["Orientation"] = config.qogChannelIdOrientation;
  j["connection"]["qog"]["channelId"]["GpsData"] = config.qogChannelIdGpsData;
  j["connection"]["qog"]["channelId"]["CanRearHitch"] = config.qogChannelIdCanRearHitch;
  j["connection"]["qog"]["channelId"]["CanFrontHitch"] = config.qogChannelIdCanFrontHitch;
  j["connection"]["qog"]["channelId"]["CanRearPtoRpm"] = config.qogChannelIdCanRearPtoRpm;
  j["connection"]["qog"]["channelId"]["CanFrontPtoRpm"] = config.qogChannelIdCanFrontPtoRpm;
  j["connection"]["qog"]["channelId"]["CanMotorRpm"] = config.qogChannelIdCanMotorRpm;

  return j;
}

void parseJsonToSteerConfig( json& j, SteerConfig& config ) {
  if( j.is_object() ) {
    try {
      {
        std::string str = j.value( "/wifi/ssid"_json_pointer, steerConfigDefaults.ssid );
        memcpy( config.ssid, str.c_str(), std::min( str.size(), sizeof( config.ssid ) ) );
      }
      {
        std::string str = j.value( "/wifi/password"_json_pointer, steerConfigDefaults.password );
        memcpy( config.password, str.c_str(), std::min( str.size(), sizeof( config.password ) ) );
      }
      {
        std::string str = j.value( "/wifi/hostname"_json_pointer, steerConfigDefaults.hostname );
        memcpy( config.hostname, str.c_str(), std::min( str.size(), sizeof( config.hostname ) ) );
      }
      config.apModePin = j.value( "/wifi/apModePin"_json_pointer, steerConfigDefaults.apModePin );
      config.retainWifiSettings = j.value( "/wifi/retainSettings"_json_pointer, steerConfigDefaults.retainWifiSettings );

      config.outputType = j.value( "/output/type"_json_pointer, steerConfigDefaults.outputType );
      config.pwmFrequency = j.value( "/output/pwmFrequency"_json_pointer, steerConfigDefaults.pwmFrequency );
      config.steeringPidMinPwm = j.value( "/output/minPWM"_json_pointer, steerConfigDefaults.steeringPidMinPwm );
      config.gpioPwm = j.value( "/output/gpioPwm"_json_pointer, steerConfigDefaults.gpioPwm );
      config.gpioDir = j.value( "/output/gpioDir"_json_pointer, steerConfigDefaults.gpioDir );
      config.gpioEn = j.value( "/output/gpioEn"_json_pointer, steerConfigDefaults.gpioEn );

      config.steeringPidKp = j.value( "/PID/P"_json_pointer, steerConfigDefaults.steeringPidKp );
      config.steeringPidKi = j.value( "/PID/I"_json_pointer, steerConfigDefaults.steeringPidKi );
      config.steeringPidKd = j.value( "/PID/D"_json_pointer, steerConfigDefaults.steeringPidKd );
      config.steeringPidAutoBangOnFactor = j.value( "/PID/autoBangOnFactor"_json_pointer, steerConfigDefaults.steeringPidAutoBangOnFactor );
      config.steeringPidBangOn = j.value( "/PID/bangOn"_json_pointer, steerConfigDefaults.steeringPidBangOn );
      config.steeringPidBangOff = j.value( "/PID/bangOff"_json_pointer, steerConfigDefaults.steeringPidBangOff );

      config.workswitchType = j.value( "/workswitch/workswitchType"_json_pointer, steerConfigDefaults.workswitchType );
      config.gpioWorkswitch = j.value( "/workswitch/gpioWorkswitch"_json_pointer, steerConfigDefaults.gpioWorkswitch );
      config.gpioSteerswitch = j.value( "/workswitch/gpioSteerswitch"_json_pointer, steerConfigDefaults.gpioSteerswitch );
      config.autoRecogniseSteerGpioAsSwitchOrButton = j.value( "/workswitch/msAutoRecogniseSteerGpioAsSwitch"_json_pointer, steerConfigDefaults.autoRecogniseSteerGpioAsSwitchOrButton );
      config.workswitchActiveLow = j.value( "/workswitch/workswitchActiveLow"_json_pointer, steerConfigDefaults.workswitchActiveLow );
      config.steerswitchActiveLow = j.value( "/workswitch/steerswitchActiveLow"_json_pointer, steerConfigDefaults.steerswitchActiveLow );

      config.wheelAngleInput = j.value( "/wheelangle/input"_json_pointer, steerConfigDefaults.wheelAngleInput );
      config.wheelAngleSensorType = j.value( "/wheelangle/sensorType"_json_pointer, steerConfigDefaults.wheelAngleSensorType );
      config.invertWheelAngleSensor = j.value( "/wheelangle/invert"_json_pointer, steerConfigDefaults.invertWheelAngleSensor );
      config.wheelAngleCountsPerDegree = j.value( "/wheelangle/countsPerDegree"_json_pointer, steerConfigDefaults.wheelAngleCountsPerDegree );
      config.wheelAnglePositionZero = j.value( "/wheelangle/positionZero"_json_pointer, steerConfigDefaults.wheelAnglePositionZero );

      config.wheelAngleFirstArmLenght = j.value( "/wheelangle/tierod/FirstArmLenght"_json_pointer, steerConfigDefaults.wheelAngleFirstArmLenght );
      config.wheelAngleSecondArmLenght = j.value( "/wheelangle/tierod/SecondArmLenght"_json_pointer, steerConfigDefaults.wheelAngleSecondArmLenght );
      config.wheelAngleTieRodStroke = j.value( "/wheelangle/tierod/TieRodStroke"_json_pointer, steerConfigDefaults.wheelAngleTieRodStroke );
      config.wheelAngleMinimumAngle = j.value( "/wheelangle/tierod/MinimumAngle"_json_pointer, steerConfigDefaults.wheelAngleMinimumAngle );
      config.wheelAngleTrackArmLenght = j.value( "/wheelangle/tierod/TrackArmLenght"_json_pointer, steerConfigDefaults.wheelAngleTrackArmLenght );

      config.gpioSDA = j.value( "/i2c/sda"_json_pointer, steerConfigDefaults.gpioSDA );
      config.gpioSCL = j.value( "/i2c/scl"_json_pointer, steerConfigDefaults.gpioSCL );
      config.i2cBusSpeed = j.value( "/i2c/speed"_json_pointer, steerConfigDefaults.i2cBusSpeed );

      config.imuType = j.value( "/imu/type"_json_pointer, steerConfigDefaults.imuType );

      config.inclinoType = j.value( "/inclinomeer/type"_json_pointer, steerConfigDefaults.inclinoType );
      config.invertRoll = j.value( "/inclinomeer/invertRoll"_json_pointer, steerConfigDefaults.invertRoll );

      config.mountCorrectionImuRoll = j.value( "/mountingCorrection/roll"_json_pointer, steerConfigDefaults.mountCorrectionImuRoll );
      config.mountCorrectionImuPitch = j.value( "/mountingCorrection/pitch"_json_pointer, steerConfigDefaults.mountCorrectionImuPitch );
      config.mountCorrectionImuYaw = j.value( "/mountingCorrection/yaw"_json_pointer, steerConfigDefaults.mountCorrectionImuYaw );

      config.canBusEnabled = j.value( "/canBus/enabled"_json_pointer, steerConfigDefaults.canBusEnabled );
      config.canBusRx = j.value( "/canBus/rxPin"_json_pointer, steerConfigDefaults.canBusRx );
      config.canBusTx = j.value( "/canBus/txPin"_json_pointer, steerConfigDefaults.canBusTx );
      config.canBusSpeed = j.value( "/canBus/speed"_json_pointer, steerConfigDefaults.canBusSpeed );
      config.canBusHitchThreshold = j.value( "/canBus/hitchThreshold"_json_pointer, steerConfigDefaults.canBusHitchThreshold );
      config.canBusHitchThresholdHysteresis = j.value( "/canBus/hitchThresholdHysteresis"_json_pointer, steerConfigDefaults.canBusHitchThresholdHysteresis );
      config.canBusRpmThreshold = j.value( "/canBus/rpmThreshold"_json_pointer, steerConfigDefaults.canBusRpmThreshold );
      config.canBusRpmThresholdHysteresis = j.value( "/canBus/rpmThresholdHysteresis"_json_pointer, steerConfigDefaults.canBusRpmThresholdHysteresis );

      config.rtkCorrectionType = j.value( "/gps/correctionSource"_json_pointer, steerConfigDefaults.rtkCorrectionType );
      {
        std::string str = j.value( "/gps/ntrip/server"_json_pointer, steerConfigDefaults.rtkCorrectionServer );
        memcpy( config.rtkCorrectionServer, str.c_str(), std::min( str.size(), sizeof( config.rtkCorrectionServer ) ) );
      }
      config.rtkCorrectionPort = j.value( "/gps/ntrip/port"_json_pointer, steerConfigDefaults.rtkCorrectionPort );
      {
        std::string str = j.value( "/gps/ntrip/username"_json_pointer, steerConfigDefaults.rtkCorrectionUsername );
        memcpy( config.rtkCorrectionUsername, str.c_str(), std::min( str.size(), sizeof( config.rtkCorrectionUsername ) ) );
      }
      {
        std::string str = j.value( "/gps/ntrip/password"_json_pointer, steerConfigDefaults.rtkCorrectionPassword );
        memcpy( config.rtkCorrectionPassword, str.c_str(), std::min( str.size(), sizeof( config.rtkCorrectionPassword ) ) );
      }
      {
        std::string str = j.value( "/gps/ntrip/mountpoint"_json_pointer, steerConfigDefaults.rtkCorrectionMountpoint );
        memcpy( config.rtkCorrectionMountpoint, str.c_str(), std::min( str.size(), sizeof( config.rtkCorrectionMountpoint ) ) );
      }
      {
        std::string str = j.value( "/gps/ntrip/NMEAToSend"_json_pointer, steerConfigDefaults.rtkCorrectionNmeaToSend );
        memcpy( config.rtkCorrectionNmeaToSend, str.c_str(), std::min( str.size(), sizeof( config.rtkCorrectionNmeaToSend ) ) );
      }

      config.ntripPositionSendIntervall = j.value( "/gps/ntrip/intervalSendPosition"_json_pointer, steerConfigDefaults.ntripPositionSendIntervall );
      config.rtkCorrectionBaudrate = j.value( "/gps/baudrate"_json_pointer, steerConfigDefaults.rtkCorrectionBaudrate );
      config.sendNmeaDataTo = j.value( "/gps/outputTo"_json_pointer, steerConfigDefaults.sendNmeaDataTo );
      config.sendNmeaDataTcpPort = j.value( "/gps/tcpPort"_json_pointer, steerConfigDefaults.sendNmeaDataTcpPort );

      config.mode = j.value( "/connection/mode"_json_pointer, steerConfigDefaults.mode );
      config.aogPortSendFrom = j.value( "/connection/aog/sendFrom"_json_pointer, steerConfigDefaults.aogPortSendFrom );
      config.aogPortListenTo = j.value( "/connection/aog/listenTo"_json_pointer, steerConfigDefaults.aogPortListenTo );
      config.aogPortSendTo = j.value( "/connection/aog/sendTo"_json_pointer, steerConfigDefaults.aogPortSendTo );

      config.qogPortListenTo = j.value( "/connection/qog/listenTo"_json_pointer, steerConfigDefaults.qogPortListenTo );
      config.qogPortSendTo = j.value( "/connection/qog/sendTo"_json_pointer, steerConfigDefaults.qogPortSendTo );

      config.qogChannelIdWorkswitch = j.value( "/connection/qog/channelId/Workswitch"_json_pointer, steerConfigDefaults.qogChannelIdWorkswitch );
      config.qogChannelIdSteerswitch = j.value( "/connection/qog/channelId/Steerswitch"_json_pointer, steerConfigDefaults.qogChannelIdSteerswitch );
      config.qogChannelIdWheelAngle = j.value( "/connection/qog/channelId/WheelAngle"_json_pointer, steerConfigDefaults.qogChannelIdWheelAngle );
      config.qogChannelIdSetpointSteerAngle = j.value( "/connection/qog/channelId/SetpointSteerAngle"_json_pointer, steerConfigDefaults.qogChannelIdSetpointSteerAngle );
      config.qogChannelIdOrientation = j.value( "/connection/qog/channelId/Orientation"_json_pointer, steerConfigDefaults.qogChannelIdOrientation );
      config.qogChannelIdGpsData = j.value( "/connection/qog/channelId/GpsData"_json_pointer, steerConfigDefaults.qogChannelIdGpsData );
      config.qogChannelIdCanRearHitch = j.value( "/connection/qog/channelId/CanRearHitch"_json_pointer, steerConfigDefaults.qogChannelIdCanRearHitch );
      config.qogChannelIdCanFrontHitch = j.value( "/connection/qog/channelId/CanFrontHitch"_json_pointer, steerConfigDefaults.qogChannelIdCanFrontHitch );
      config.qogChannelIdCanRearPtoRpm = j.value( "/connection/qog/channelId/CanRearPtoRpm"_json_pointer, steerConfigDefaults.qogChannelIdCanRearPtoRpm );
      config.qogChannelIdCanFrontPtoRpm = j.value( "/connection/qog/channelId/CanFrontPtoRpm"_json_pointer, steerConfigDefaults.qogChannelIdCanFrontPtoRpm );
      config.qogChannelIdCanMotorRpm = j.value( "/connection/qog/channelId/CanMotorRpm"_json_pointer, steerConfigDefaults.qogChannelIdCanMotorRpm );

    } catch( json::exception& e ) {
      // output exception information
      Serial.print( "message: " );
      Serial.println( e.what() );
      Serial.print( "exception id: " );
      Serial.println( e.id );
    }
  }
}

extern void parseJsonToFxos8700Fxas21002Calibration( json& json, Fxos8700Fxas21002CalibrationData& calibration );
extern void parseFxos8700Fxas21002CalibrationToJson( Fxos8700Fxas21002CalibrationData& calibration, json& json );
