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

#include <memory>

#include <FS.h>
#include <SPIFFS.h>

extern "C" {
#include <crypto/base64.h>
}

#include "main.hpp"
#include "jsonFunctions.hpp"

void loadSavedConfig() {
  {
    auto j = loadJsonFromFile( "/config.json" );
    parseJsonToSteerConfig( j, steerConfig );
  }

  {
    auto j = loadJsonFromFile( "/calibration.json" );
    parseJsonToFxos8700Fxas21002Calibration( j, fxos8700Fxas21002CalibrationData );
  }
}

void saveConfig() {
  {
    const auto j = parseSteerConfigToJson( steerConfig );
    saveJsonToFile( j, "/config.json" );
  }

  {
    const auto j = parseFxos8700Fxas21002CalibrationToJson( fxos8700Fxas21002CalibrationData );
    saveJsonToFile( j, "/calibration.json" );
  }
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
    } else {
      Serial.print( "Could not open file for reading: " );
      Serial.println( fileName );
      Serial.flush();
    }

    file.close();
  }

  return j;
}

void saveJsonToFile( const json& json, const char* fileName ) {
  // pretty print with 2 spaces indentation
  auto data = json.dump( 2 );

  File file = SPIFFS.open( fileName, "w" );

  if( file && !file.isDirectory() ) {
    file.write( ( uint8_t* )data.c_str(), data.size() );
  } else {
    Serial.print( "Could not open file for writing: " );
    Serial.println( fileName );
    Serial.flush();
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
  j["gps"]["sendNmeaDataUdpPort"] = config.sendNmeaDataUdpPort;
  j["gps"]["sendNmeaDataUdpPortFrom"] = config.sendNmeaDataUdpPortFrom;

  j["connection"]["mode"] = int( config.mode );
  j["connection"]["baudrate"] = config.baudrate;
  j["connection"]["enableOTA"] = config.enableOTA;

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
  j["connection"]["qog"]["channelId"]["GpsDataIn"] = config.qogChannelIdGpsDataIn;
  j["connection"]["qog"]["channelId"]["GpsDataOut"] = config.qogChannelIdGpsDataOut;
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
      config.sendNmeaDataUdpPort = j.value( "/gps/sendNmeaDataUdpPort"_json_pointer, steerConfigDefaults.sendNmeaDataUdpPort );
      config.sendNmeaDataUdpPortFrom = j.value( "/gps/sendNmeaDataUdpPortFrom"_json_pointer, steerConfigDefaults.sendNmeaDataUdpPortFrom );

      config.baudrate = j.value( "/connection/baudrate"_json_pointer, steerConfigDefaults.baudrate );
      config.enableOTA = j.value( "/connection/enableOTA"_json_pointer, steerConfigDefaults.enableOTA );

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
      config.qogChannelIdGpsDataIn = j.value( "/connection/qog/channelId/GpsDataIn"_json_pointer, steerConfigDefaults.qogChannelIdGpsDataIn );
      config.qogChannelIdGpsDataOut = j.value( "/connection/qog/channelId/GpsDataOut"_json_pointer, steerConfigDefaults.qogChannelIdGpsDataOut );
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
      Serial.flush();
    }
  }
}

void sendBase64DataTransmission( uint16_t channelId, const char* data, size_t len ) {
  json j;
  j["channelId"] = channelId;

  size_t outputLength;
  char* encoded = ( char* )base64_encode( ( const unsigned char* )data, len, &outputLength );

  if( encoded ) {
    j["data"] = std::string( encoded, outputLength );
    free( encoded );

    std::vector<std::uint8_t> cbor = json::to_cbor( j );
    udpSendFrom.broadcastTo( cbor.data(), cbor.size(), initialisation.portSendTo );
  }
}

void sendStateTransmission( uint16_t channelId, bool state ) {
  json j;
  j["channelId"] = channelId;
  j["state"] = state;

  std::vector<std::uint8_t> cbor = json::to_cbor( j );
  udpSendFrom.broadcastTo( cbor.data(), cbor.size(), initialisation.portSendTo );
}

void sendNumberTransmission( uint16_t channelId, double number ) {
  json j;
  j["channelId"] = channelId;
  j["number"] = number;

  std::vector<std::uint8_t> cbor = json::to_cbor( j );
  udpSendFrom.broadcastTo( cbor.data(), cbor.size(), initialisation.portSendTo );
}

void sendQuaternionTransmission( uint16_t channelId, imu::Quaternion quaterion ) {
  json j;
  j["channelId"] = channelId;
  j["x"] = quaterion.x();
  j["y"] = quaterion.y();
  j["z"] = quaterion.z();
  j["w"] = quaterion.w();

  std::vector<std::uint8_t> cbor = json::to_cbor( j );
  udpSendFrom.broadcastTo( cbor.data(), cbor.size(), initialisation.portSendTo );
}

void parseJsonToFxos8700Fxas21002Calibration( json& config, Fxos8700Fxas21002CalibrationData& calibration ) {
  if( config.is_object() ) {
    try {
      calibration.mag_offsets[0] = config.value( "/Fxos8700Fxas21002Calibration/mag_offsets/0"_json_pointer, fxos8700Fxas21002CalibrationDefault.mag_offsets[0] );
      calibration.mag_offsets[1] = config.value( "/Fxos8700Fxas21002Calibration/mag_offsets/1"_json_pointer, fxos8700Fxas21002CalibrationDefault.mag_offsets[1] );
      calibration.mag_offsets[2] = config.value( "/Fxos8700Fxas21002Calibration/mag_offsets/2"_json_pointer, fxos8700Fxas21002CalibrationDefault.mag_offsets[2] );
      calibration.mag_softiron_matrix[0][0] = config.value( "/Fxos8700Fxas21002Calibration/mag_softiron_matrix/0/0"_json_pointer, fxos8700Fxas21002CalibrationDefault.mag_softiron_matrix[0][0] );
      calibration.mag_softiron_matrix[0][1] = config.value( "/Fxos8700Fxas21002Calibration/mag_softiron_matrix/0/1"_json_pointer, fxos8700Fxas21002CalibrationDefault.mag_softiron_matrix[0][1] );
      calibration.mag_softiron_matrix[0][2] = config.value( "/Fxos8700Fxas21002Calibration/mag_softiron_matrix/0/2"_json_pointer, fxos8700Fxas21002CalibrationDefault.mag_softiron_matrix[0][2] );
      calibration.mag_softiron_matrix[1][0] = config.value( "/Fxos8700Fxas21002Calibration/mag_softiron_matrix/1/0"_json_pointer, fxos8700Fxas21002CalibrationDefault.mag_softiron_matrix[1][0] );
      calibration.mag_softiron_matrix[1][1] = config.value( "/Fxos8700Fxas21002Calibration/mag_softiron_matrix/1/1"_json_pointer, fxos8700Fxas21002CalibrationDefault.mag_softiron_matrix[1][1] );
      calibration.mag_softiron_matrix[1][2] = config.value( "/Fxos8700Fxas21002Calibration/mag_softiron_matrix/1/2"_json_pointer, fxos8700Fxas21002CalibrationDefault.mag_softiron_matrix[1][2] );
      calibration.mag_softiron_matrix[2][0] = config.value( "/Fxos8700Fxas21002Calibration/mag_softiron_matrix/2/0"_json_pointer, fxos8700Fxas21002CalibrationDefault.mag_softiron_matrix[2][0] );
      calibration.mag_softiron_matrix[2][1] = config.value( "/Fxos8700Fxas21002Calibration/mag_softiron_matrix/2/1"_json_pointer, fxos8700Fxas21002CalibrationDefault.mag_softiron_matrix[2][1] );
      calibration.mag_softiron_matrix[2][2] = config.value( "/Fxos8700Fxas21002Calibration/mag_softiron_matrix/2/2"_json_pointer, fxos8700Fxas21002CalibrationDefault.mag_softiron_matrix[2][2] );
      calibration.mag_field_strength = config.value( "/Fxos8700Fxas21002Calibration/mag_field_strength"_json_pointer, fxos8700Fxas21002CalibrationDefault.mag_field_strength );
      calibration.gyro_zero_offsets[0] = config.value( "/Fxos8700Fxas21002Calibration/gyro_zero_offsets/0"_json_pointer, fxos8700Fxas21002CalibrationDefault.gyro_zero_offsets[0] );
      calibration.gyro_zero_offsets[1] = config.value( "/Fxos8700Fxas21002Calibration/gyro_zero_offsets/1"_json_pointer, fxos8700Fxas21002CalibrationDefault.gyro_zero_offsets[1] );
      calibration.gyro_zero_offsets[2] = config.value( "/Fxos8700Fxas21002Calibration/gyro_zero_offsets/2"_json_pointer, fxos8700Fxas21002CalibrationDefault.gyro_zero_offsets[2] );
    } catch( json::exception& e ) {
      // output exception information
      Serial.print( "message: " );
      Serial.println( e.what() );
      Serial.print( "exception id: " );
      Serial.println( e.id );
    }
  }
}

json parseFxos8700Fxas21002CalibrationToJson( Fxos8700Fxas21002CalibrationData& calibration ) {
  json j;

  j["Fxos8700Fxas21002Calibration"]["mag_offsets"][0] = calibration.mag_offsets[0];
  j["Fxos8700Fxas21002Calibration"]["mag_offsets"][1] = calibration.mag_offsets[1];
  j["Fxos8700Fxas21002Calibration"]["mag_offsets"][2] = calibration.mag_offsets[2];
  j["Fxos8700Fxas21002Calibration"]["mag_softiron_matrix"][0][0] = calibration.mag_softiron_matrix[0][0];
  j["Fxos8700Fxas21002Calibration"]["mag_softiron_matrix"][0][1] = calibration.mag_softiron_matrix[0][1];
  j["Fxos8700Fxas21002Calibration"]["mag_softiron_matrix"][0][2] = calibration.mag_softiron_matrix[0][2];
  j["Fxos8700Fxas21002Calibration"]["mag_softiron_matrix"][1][0] = calibration.mag_softiron_matrix[1][0];
  j["Fxos8700Fxas21002Calibration"]["mag_softiron_matrix"][1][1] = calibration.mag_softiron_matrix[1][1];
  j["Fxos8700Fxas21002Calibration"]["mag_softiron_matrix"][1][2] = calibration.mag_softiron_matrix[1][2];
  j["Fxos8700Fxas21002Calibration"]["mag_softiron_matrix"][2][0] = calibration.mag_softiron_matrix[2][0];
  j["Fxos8700Fxas21002Calibration"]["mag_softiron_matrix"][2][1] = calibration.mag_softiron_matrix[2][1];
  j["Fxos8700Fxas21002Calibration"]["mag_softiron_matrix"][2][2] = calibration.mag_softiron_matrix[2][2];
  j["Fxos8700Fxas21002Calibration"]["mag_field_strength"] = calibration.mag_field_strength;
  j["Fxos8700Fxas21002Calibration"]["gyro_zero_offsets"][0] = calibration.gyro_zero_offsets[0];
  j["Fxos8700Fxas21002Calibration"]["gyro_zero_offsets"][1] = calibration.gyro_zero_offsets[1];
  j["Fxos8700Fxas21002Calibration"]["gyro_zero_offsets"][2] = calibration.gyro_zero_offsets[2];

  return j;
}
