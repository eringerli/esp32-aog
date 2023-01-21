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

#include <memory>

#include <FS.h>
#include <SPIFFS.h>

#include "jsonFunctions.hpp"
#include "main.hpp"

#include <StreamUtils.h>

void
loadSavedConfig() {
  auto j = loadJsonFromFile( "/config.json" );
  parseJsonToSteerConfig( j, steerConfig );
}

void
saveConfig() {
  const auto j = parseSteerConfigToJson( steerConfig );
  saveJsonToFile( j, "/config.json" );
}

DynamicJsonDocument
loadJsonFromFile( const char* fileName ) {
  DynamicJsonDocument doc( 3072 );

  if( SPIFFS.exists( fileName ) ) {
    auto file = SPIFFS.open( fileName, "r" );

    if( file ) {
      ReadBufferingStream bufferedFile( file, 128 );

      DeserializationError error = deserializeJson( doc, bufferedFile );
      if( error ) {
        Serial.println( F( "Failed to read file, using default configuration" ) );
      }

    } else {
      Serial.print( "Could not open file for reading: " );
      Serial.println( fileName );
      Serial.flush();
    }

    file.close();
  }

  return doc;
}

void
saveJsonToFile( const DynamicJsonDocument& doc, const char* fileName ) {
  File file = SPIFFS.open( fileName, "w" );

  if( file && !file.isDirectory() ) {
    WriteBufferingStream bufferedFile( file, 128 );
    serializeJsonPretty( doc, bufferedFile );
  } else {
    Serial.print( "Could not open file for writing: " );
    Serial.println( fileName );
    Serial.flush();
  }

  file.close();
}

DynamicJsonDocument
parseSteerConfigToJson( const SteerConfig& config ) {
  DynamicJsonDocument j( 3072 );

  j["wifi"]["ssidAp"]         = config.ssidAp;
  j["wifi"]["passwordAp"]     = config.passwordAp;
  j["wifi"]["ssidSta"]        = config.ssidSta;
  j["wifi"]["passwordSta"]    = config.passwordSta;
  j["wifi"]["hostname"]       = config.hostname;
  j["wifi"]["apModePin"]      = int( config.apModePin );
  j["wifi"]["retainSettings"] = config.retainWifiSettings;

  j["output"]["type"]         = int( config.outputType );
  j["output"]["pwmFrequency"] = config.pwmFrequency;
  j["output"]["minPWM"]       = config.steeringPidMinPwm;
  j["output"]["gpioPwm"]      = int( config.gpioPwm );
  j["output"]["gpioDir"]      = int( config.gpioDir );
  j["output"]["gpioEn"]       = int( config.gpioEn );

  j["PID"]["P"]                = config.steeringPidKp;
  j["PID"]["I"]                = config.steeringPidKi;
  j["PID"]["D"]                = config.steeringPidKd;
  j["PID"]["autoBangOnFactor"] = config.steeringPidAutoBangOnFactor;
  j["PID"]["bangOn"]           = config.steeringPidBangOn;
  j["PID"]["bangOff"]          = config.steeringPidBangOff;

  j["workswitch"]["workswitchType"]  = int( config.workswitchType );
  j["workswitch"]["gpioWorkswitch"]  = int( config.gpioWorkswitch );
  j["workswitch"]["gpioSteerswitch"] = int( config.gpioSteerswitch );
  j["workswitch"]["msAutoRecogniseSteerGpioAsSwitch"] =
    config.autoRecogniseSteerGpioAsSwitchOrButton;
  j["workswitch"]["workswitchActiveLow"]  = config.workswitchActiveLow;
  j["workswitch"]["steerswitchActiveLow"] = config.steerswitchActiveLow;

  j["wheelangle"]["input"]           = config.wheelAngleInput;
  j["wheelangle"]["sensorType"]      = int( config.wheelAngleSensorType );
  j["wheelangle"]["invert"]          = config.invertWheelAngleSensor;
  j["wheelangle"]["countsPerDegree"] = config.wheelAngleCountsPerDegree;
  j["wheelangle"]["positionZero"]    = config.wheelAnglePositionZero;

  j["i2c"]["sda"]   = int( config.gpioSDA );
  j["i2c"]["scl"]   = int( config.gpioSCL );
  j["i2c"]["speed"] = config.i2cBusSpeed;

  j["spi"]["sck"]   = int( config.gpioSck );
  j["spi"]["miso"]  = int( config.gpioMiso );
  j["spi"]["mosi"]  = int( config.gpioMosi );
  j["spi"]["speed"] = config.spiBusSpeed;

  j["adcimu"]["ads131m04Cs"]   = int( config.gpioAds131m04Cs );
  j["adcimu"]["ads131m04Drdy"] = int( config.gpioAds131m04Drdy );
  j["adcimu"]["accGyroCs"]     = int( config.gpioAccGyroCs );
  j["adcimu"]["accGyroDrdy"]   = int( config.gpioAccGyroDrdy );
  j["adcimu"]["magCs"]         = int( config.gpioMagCs );
  j["adcimu"]["magDrdy"]       = int( config.gpioMagDrdy );

  j["filter"]["samplerate"]       = ( int )( config.ads131m04SampleRate );
  j["filter"]["cutoffFrequency1"] = ( int )( config.filterCutOffFrequencies[0] );
  j["filter"]["cutoffFrequency2"] = ( int )( config.filterCutOffFrequencies[1] );
  j["filter"]["cutoffFrequency3"] = ( int )( config.filterCutOffFrequencies[2] );
  j["filter"]["cutoffFrequency4"] = ( int )( config.filterCutOffFrequencies[3] );

  j["canBus"]["enabled"]                  = config.canBusEnabled;
  j["canBus"]["rxPin"]                    = int( config.canBusRx );
  j["canBus"]["txPin"]                    = int( config.canBusTx );
  j["canBus"]["speed"]                    = int( config.canBusSpeed );
  j["canBus"]["hitchThreshold"]           = config.canBusHitchThreshold;
  j["canBus"]["hitchThresholdHysteresis"] = config.canBusHitchThresholdHysteresis;
  j["canBus"]["rpmThreshold"]             = config.canBusRpmThreshold;
  j["canBus"]["rpmThresholdHysteresis"]   = config.canBusRpmThresholdHysteresis;

  j["gps"]["correctionSource"]           = int( config.rtkCorrectionType );
  j["gps"]["rtkCorrectionServer"]        = config.rtkCorrectionServer;
  j["gps"]["rtkCorrectionPort"]          = config.rtkCorrectionPort;
  j["gps"]["rtkCorrectionUsername"]      = config.rtkCorrectionUsername;
  j["gps"]["rtkCorrectionPassword"]      = config.rtkCorrectionPassword;
  j["gps"]["rtkCorrectionMountpoint"]    = config.rtkCorrectionMountpoint;
  j["gps"]["rtkCorrectionNmeaToSend"]    = config.rtkCorrectionNmeaToSend;
  j["gps"]["ntripPositionSendIntervall"] = config.ntripPositionSendIntervall;
  j["gps"]["rtkCorrectionBaudrate"]      = config.rtkCorrectionBaudrate;
  j["gps"]["sendNmeaDataTo"]             = int( config.sendNmeaDataTo );
  j["gps"]["sendNmeaDataTcpPort"]        = config.sendNmeaDataTcpPort;
  j["gps"]["sendNmeaDataUdpPort"]        = config.sendNmeaDataUdpPort;
  j["gps"]["sendNmeaDataUdpPortFrom"]    = config.sendNmeaDataUdpPortFrom;

  j["connection"]["mode"]      = int( config.mode );
  j["connection"]["baudrate"]  = config.baudrate;
  j["connection"]["enableOTA"] = config.enableOTA;

  j["connection"]["aogPortSendFrom"] = config.aogPortSendFrom;
  j["connection"]["aogPortListenTo"] = config.aogPortListenTo;
  j["connection"]["aogPortSendTo"]   = config.aogPortSendTo;

  j["connection"]["qogPortListenTo"] = config.qogPortListenTo;
  j["connection"]["qogPortSendTo"]   = config.qogPortSendTo;

  j["channelId"]["Workswitch"]         = config.qogChannelIdWorkswitch;
  j["channelId"]["Steerswitch"]        = config.qogChannelIdSteerswitch;
  j["channelId"]["WheelAngle"]         = config.qogChannelIdWheelAngle;
  j["channelId"]["SetpointSteerAngle"] = config.qogChannelIdSetpointSteerAngle;
  j["channelId"]["Orientation"]        = config.qogChannelIdOrientation;
  j["channelId"]["GpsDataIn"]          = config.qogChannelIdGpsDataIn;
  j["channelId"]["GpsDataOut"]         = config.qogChannelIdGpsDataOut;
  j["channelId"]["CanRearHitch"]       = config.qogChannelIdCanRearHitch;
  j["channelId"]["CanFrontHitch"]      = config.qogChannelIdCanFrontHitch;
  j["channelId"]["CanRearPtoRpm"]      = config.qogChannelIdCanRearPtoRpm;
  j["channelId"]["CanFrontPtoRpm"]     = config.qogChannelIdCanFrontPtoRpm;
  j["channelId"]["CanMotorRpm"]        = config.qogChannelIdCanMotorRpm;

  return j;
}

void
parseJsonToSteerConfig( DynamicJsonDocument& doc, SteerConfig& config ) {
  if( !doc.isNull() ) {
    {
      strlcpy( config.ssidAp,
               doc["wifi"]["ssidAp"] | steerConfigDefaults.ssidAp,
               sizeof( config.ssidAp ) );
      strlcpy( config.passwordAp,
               doc["wifi"]["passwordAp"] | steerConfigDefaults.passwordAp,
               sizeof( config.passwordAp ) );
      strlcpy( config.ssidSta,
               doc["wifi"]["ssidSta"] | steerConfigDefaults.ssidSta,
               sizeof( config.ssidSta ) );
      strlcpy( config.passwordSta,
               doc["wifi"]["passwordSta"] | steerConfigDefaults.passwordSta,
               sizeof( config.passwordSta ) );
      strlcpy( config.hostname,
               doc["wifi"]["hostname"] | steerConfigDefaults.hostname,
               sizeof( config.hostname ) );

      config.apModePin = ( SteerConfig::Gpio )( doc["wifi"]["apModePin"] |
                                                int( steerConfigDefaults.apModePin ) );
      config.retainWifiSettings =
        doc["wifi"]["retainWifiSettings"] | steerConfigDefaults.retainWifiSettings;
    }

    {
      config.outputType = ( SteerConfig::OutputType )(
        doc["output"]["type"] | int( steerConfigDefaults.outputType ) );
      config.pwmFrequency =
        doc["output"]["pwmFrequency"] | steerConfigDefaults.pwmFrequency;
      config.steeringPidMinPwm =
        doc["output"]["minPWM"] | steerConfigDefaults.steeringPidMinPwm;
      config.gpioPwm = ( SteerConfig::Gpio )( doc["output"]["gpioPwm"] |
                                              int( steerConfigDefaults.gpioPwm ) );
      config.gpioDir = ( SteerConfig::Gpio )( doc["output"]["gpioDir"] |
                                              int( steerConfigDefaults.gpioDir ) );
      config.gpioEn  = ( SteerConfig::Gpio )( doc["output"]["gpioEn"] |
                                             int( steerConfigDefaults.gpioEn ) );
    }

    {
      config.steeringPidKp = doc["steeringPid"]["P"] | steerConfigDefaults.steeringPidKp;
      config.steeringPidKp = doc["steeringPid"]["I"] | steerConfigDefaults.steeringPidKp;
      config.steeringPidKp = doc["steeringPid"]["D"] | steerConfigDefaults.steeringPidKp;
      config.steeringPidAutoBangOnFactor =
        doc["steeringPid"]["autoBangOnFactor"] |
        steerConfigDefaults.steeringPidAutoBangOnFactor;
      config.steeringPidBangOn =
        doc["steeringPid"]["bangOn"] | steerConfigDefaults.steeringPidBangOn;
      config.steeringPidBangOff =
        doc["steeringPid"]["bangOff"] | steerConfigDefaults.steeringPidBangOff;
    }

    {
      config.workswitchType = ( SteerConfig::WorkswitchType )(
        doc["workswitch"]["workswitchType"] | int( steerConfigDefaults.workswitchType ) );
      config.gpioWorkswitch = ( SteerConfig::Gpio )(
        doc["workswitch"]["gpioWorkswitch"] | int( steerConfigDefaults.gpioWorkswitch ) );
      config.gpioSteerswitch =
        ( SteerConfig::Gpio )( doc["workswitch"]["gpioSteerswitch"] |
                               int( steerConfigDefaults.gpioSteerswitch ) );
      config.autoRecogniseSteerGpioAsSwitchOrButton =
        doc["workswitch"]["autoRecogniseSteerGpioAsSwitchOrButton"] |
        steerConfigDefaults.autoRecogniseSteerGpioAsSwitchOrButton;
      config.workswitchActiveLow = doc["workswitch"]["workswitchActiveLow"] |
                                   steerConfigDefaults.workswitchActiveLow;
      config.steerswitchActiveLow = doc["workswitch"]["steerswitchActiveLow"] |
                                    steerConfigDefaults.steerswitchActiveLow;
    }

    {
      config.wheelAngleInput =
        doc["wheelangle"]["input"] | steerConfigDefaults.wheelAngleInput;
      config.wheelAngleSensorType = ( SteerConfig::WheelAngleSensorType )(
        doc["wheelangle"]["sensorType"] |
        int( steerConfigDefaults.wheelAngleSensorType ) );
      config.invertWheelAngleSensor =
        doc["wheelangle"]["invert"] | steerConfigDefaults.invertWheelAngleSensor;
      config.wheelAngleCountsPerDegree = doc["wheelangle"]["countsPerDegree"] |
                                         steerConfigDefaults.wheelAngleCountsPerDegree;
      config.wheelAnglePositionZero =
        doc["wheelangle"]["positionZero"] | steerConfigDefaults.wheelAnglePositionZero;
    }

    {
      config.gpioSDA =
        ( SteerConfig::Gpio )( doc["i2c"]["sda"] | int( steerConfigDefaults.gpioSDA ) );
      config.gpioSCL =
        ( SteerConfig::Gpio )( doc["i2c"]["scl"] | int( steerConfigDefaults.gpioSCL ) );
      config.i2cBusSpeed = doc["i2c"]["speed"] | steerConfigDefaults.i2cBusSpeed;
    }

    {
      config.gpioSck =
        ( SteerConfig::Gpio )( doc["spi"]["sck"] | int( steerConfigDefaults.gpioSck ) );
      config.gpioMiso =
        ( SteerConfig::Gpio )( doc["spi"]["miso"] | int( steerConfigDefaults.gpioMiso ) );
      config.gpioMosi =
        ( SteerConfig::Gpio )( doc["spi"]["mosi"] | int( steerConfigDefaults.gpioMosi ) );
      config.spiBusSpeed = doc["spi"]["speed"] | steerConfigDefaults.spiBusSpeed;
    }

    {
      config.gpioAds131m04Cs = ( SteerConfig::Gpio )(
        doc["adcimu"]["ads131m04Cs"] | int( steerConfigDefaults.gpioAds131m04Cs ) );
      config.gpioAds131m04Drdy = ( SteerConfig::Gpio )(
        doc["adcimu"]["ads131m04Drdy"] | int( steerConfigDefaults.gpioAds131m04Drdy ) );
      config.gpioAccGyroCs = ( SteerConfig::Gpio )(
        doc["adcimu"]["accGyroCs"] | int( steerConfigDefaults.gpioAccGyroCs ) );
      config.gpioAccGyroDrdy = ( SteerConfig::Gpio )(
        doc["adcimu"]["accGyroDrdy"] | int( steerConfigDefaults.gpioAccGyroDrdy ) );
      config.gpioMagCs   = ( SteerConfig::Gpio )( doc["adcimu"]["magCs"] |
                                                int( steerConfigDefaults.gpioMagCs ) );
      config.gpioMagDrdy = ( SteerConfig::Gpio )(
        doc["adcimu"]["magDrdy"] | int( steerConfigDefaults.gpioMagDrdy ) );
    }

    {
      config.ads131m04SampleRate = ( SteerConfig::Ads131m04SampleRate )(
        doc["filter"]["samplerate"] | int( steerConfigDefaults.ads131m04SampleRate ) );
      config.filterCutOffFrequencies[0] = doc["filter"]["cutoffFrequency1"] |
                                          steerConfigDefaults.filterCutOffFrequencies[0];
      config.filterCutOffFrequencies[1] = doc["filter"]["cutoffFrequency2"] |
                                          steerConfigDefaults.filterCutOffFrequencies[1];
      config.filterCutOffFrequencies[2] = doc["filter"]["cutoffFrequency3"] |
                                          steerConfigDefaults.filterCutOffFrequencies[2];
      config.filterCutOffFrequencies[3] = doc["filter"]["cutoffFrequency4"] |
                                          steerConfigDefaults.filterCutOffFrequencies[3];
    }

    {
      config.canBusEnabled = doc["canBus"]["enabled"] | steerConfigDefaults.canBusEnabled;
      config.canBusRx      = ( SteerConfig::Gpio )( doc["canBus"]["rxPin"] |
                                               int( steerConfigDefaults.canBusRx ) );
      config.canBusTx      = ( SteerConfig::Gpio )( doc["canBus"]["txPin"] |
                                               int( steerConfigDefaults.canBusTx ) );
      config.canBusSpeed   = ( SteerConfig::CanBusSpeed )(
        doc["canBus"]["speed"] | int( steerConfigDefaults.canBusSpeed ) );
      config.canBusHitchThreshold =
        doc["canBus"]["hitchThreshold"] | steerConfigDefaults.canBusHitchThreshold;
      config.canBusHitchThresholdHysteresis =
        doc["canBus"]["hitchThresholdHysteresis"] |
        steerConfigDefaults.canBusHitchThresholdHysteresis;
      config.canBusRpmThreshold =
        doc["canBus"]["rpmThreshold"] | steerConfigDefaults.canBusRpmThreshold;
      config.canBusRpmThresholdHysteresis =
        doc["canBus"]["rpmThresholdHysteresis"] |
        steerConfigDefaults.canBusRpmThresholdHysteresis;
    }

    {
      config.rtkCorrectionType = ( SteerConfig::RtkCorrectionType )(
        doc["gps"]["correctionSource"] | int( steerConfigDefaults.rtkCorrectionType ) );
      strlcpy( config.rtkCorrectionServer,
               doc["gps"]["rtkCorrectionServer"] |
                 steerConfigDefaults.rtkCorrectionServer,
               sizeof( config.rtkCorrectionServer ) );
      config.rtkCorrectionPort =
        doc["gps"]["rtkCorrectionPort"] | steerConfigDefaults.rtkCorrectionPort;
      strlcpy( config.rtkCorrectionUsername,
               doc["gps"]["rtkCorrectionUsername"] |
                 steerConfigDefaults.rtkCorrectionUsername,
               sizeof( config.rtkCorrectionUsername ) );
      strlcpy( config.rtkCorrectionPassword,
               doc["gps"]["rtkCorrectionPassword"] |
                 steerConfigDefaults.rtkCorrectionPassword,
               sizeof( config.rtkCorrectionPassword ) );
      strlcpy( config.rtkCorrectionMountpoint,
               doc["gps"]["rtkCorrectionMountpoint"] |
                 steerConfigDefaults.rtkCorrectionMountpoint,
               sizeof( config.rtkCorrectionMountpoint ) );
      strlcpy( config.rtkCorrectionNmeaToSend,
               doc["gps"]["rtkCorrectionNmeaToSend"] |
                 steerConfigDefaults.rtkCorrectionNmeaToSend,
               sizeof( config.rtkCorrectionNmeaToSend ) );
      config.ntripPositionSendIntervall = doc["gps"]["ntripPositionSendIntervall"] |
                                          steerConfigDefaults.ntripPositionSendIntervall;
      config.rtkCorrectionBaudrate =
        doc["gps"]["rtkCorrectionBaudrate"] | steerConfigDefaults.rtkCorrectionBaudrate;
      config.sendNmeaDataTo = ( SteerConfig::SendNmeaDataTo )(
        doc["gps"]["sendNmeaDataTo"] | int( steerConfigDefaults.sendNmeaDataTo ) );
      config.sendNmeaDataTcpPort =
        doc["gps"]["sendNmeaDataTcpPort"] | steerConfigDefaults.sendNmeaDataTcpPort;
      config.sendNmeaDataUdpPort =
        doc["gps"]["sendNmeaDataUdpPort"] | steerConfigDefaults.sendNmeaDataUdpPort;
      config.sendNmeaDataUdpPortFrom = doc["gps"]["sendNmeaDataUdpPortFrom"] |
                                       steerConfigDefaults.sendNmeaDataUdpPortFrom;
    }

    {
      config.baudrate  = doc["connection"]["baudrate"] | steerConfigDefaults.baudrate;
      config.enableOTA = doc["connection"]["enableOTA"] | steerConfigDefaults.enableOTA;
      config.mode      = ( SteerConfig::Mode )( doc["connection"]["mode"] |
                                           int( steerConfigDefaults.mode ) );
      config.aogPortSendFrom =
        doc["connection"]["aogPortSendFrom"] | steerConfigDefaults.aogPortSendFrom;
      config.aogPortListenTo =
        doc["connection"]["aogPortListenTo"] | steerConfigDefaults.aogPortListenTo;
      config.aogPortSendTo =
        doc["connection"]["aogPortSendTo"] | steerConfigDefaults.aogPortSendTo;
      config.qogPortListenTo =
        doc["connection"]["qogPortListenTo"] | steerConfigDefaults.qogPortListenTo;
      config.qogPortSendTo =
        doc["connection"]["qogPortSendTo"] | steerConfigDefaults.qogPortSendTo;
    }

    {
      config.qogChannelIdWorkswitch =
        doc["channelId"]["Workswitch"] | steerConfigDefaults.qogChannelIdWorkswitch;
      config.qogChannelIdSteerswitch =
        doc["channelId"]["Steerswitch"] | steerConfigDefaults.qogChannelIdSteerswitch;
      config.qogChannelIdWheelAngle =
        doc["channelId"]["WheelAngle"] | steerConfigDefaults.qogChannelIdWheelAngle;
      config.qogChannelIdSetpointSteerAngle =
        doc["channelId"]["SetpointSteerAngle"] |
        steerConfigDefaults.qogChannelIdSetpointSteerAngle;
      config.qogChannelIdOrientation =
        doc["channelId"]["Orientation"] | steerConfigDefaults.qogChannelIdOrientation;
      config.qogChannelIdGpsDataIn =
        doc["channelId"]["GpsDataIn"] | steerConfigDefaults.qogChannelIdGpsDataIn;
      config.qogChannelIdGpsDataOut =
        doc["channelId"]["GpsDataOut"] | steerConfigDefaults.qogChannelIdGpsDataOut;
      config.qogChannelIdCanRearHitch =
        doc["channelId"]["CanRearHitch"] | steerConfigDefaults.qogChannelIdCanRearHitch;
      config.qogChannelIdCanFrontHitch =
        doc["channelId"]["CanFrontHitch"] | steerConfigDefaults.qogChannelIdCanFrontHitch;
      config.qogChannelIdCanRearPtoRpm =
        doc["channelId"]["CanRearPtoRpm"] | steerConfigDefaults.qogChannelIdCanRearPtoRpm;
      config.qogChannelIdCanFrontPtoRpm = doc["channelId"]["CanFrontPtoRpm"] |
                                          steerConfigDefaults.qogChannelIdCanFrontPtoRpm;
      config.qogChannelIdCanMotorRpm =
        doc["channelId"]["CanMotorRpm"] | steerConfigDefaults.qogChannelIdCanMotorRpm;
    }
  }
}

void
sendDataTransmission( const uint16_t channelId, const char* data, const size_t len ) {
  std::vector< uint8_t > buffer;
  buffer.reserve( len + 20 );

  std::vector< uint8_t > vector( data, data + len );

  CborLite::encodeMapSize( buffer, uint8_t( 2 ) );
  CborLite::encodeText( buffer, std::string( "cid" ) );
  CborLite::encodeUnsigned( buffer, channelId );
  CborLite::encodeText( buffer, std::string( "d" ) );
  CborLite::encodeBytes( buffer, vector );

  udpSendFrom.broadcastTo( buffer.data(), buffer.size(), steerConfig.qogPortSendTo );
}
