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

#include <stdio.h>
#include <string.h>

#include "main.hpp"

#if defined(ESP32)
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif

#include <DNSServer.h>
#include <ESPUI.h>
#include <EEPROM32_Rotate.h>

///////////////////////////////////////////////////////////////////////////
// global data
///////////////////////////////////////////////////////////////////////////
SteerConfig steerConfig;
Initialisation initialisation;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t i2cMutex;

const byte DNS_PORT = 53;
IPAddress apIP( 192, 168, 1, 1 );

uint16_t labelLoad;
uint16_t labelHeading;
uint16_t labelRoll;
uint16_t labelWheelAngle;
uint16_t buttonReset;
uint16_t textNmeaToSend;

uint16_t labelWheelAngleDisplacement;

uint16_t labelStatusOutput;
uint16_t labelStatusAdc;
uint16_t labelStatusImu;
uint16_t labelStatusInclino;
uint16_t labelStatusGps;
uint16_t labelStatusNtrip;

///////////////////////////////////////////////////////////////////////////
// external Libraries
///////////////////////////////////////////////////////////////////////////
EEPROM32_Rotate EEPROM;

ESPUIClass ESPUI( Verbosity::Quiet );
DNSServer dnsServer;

///////////////////////////////////////////////////////////////////////////
// helper functions
///////////////////////////////////////////////////////////////////////////
void setResetButtonToRed() {
  ESPUI.getControl( buttonReset )->color = ControlColor::Alizarin;
  ESPUI.updateControl( buttonReset );
}

void writeEeprom() {
  EEPROM.writeUChar( ( uint16_t )EepromAddresses::Validator, 0 );
  EEPROM.writeUShort( ( uint16_t )EepromAddresses::SizeOfConfig, ( uint16_t )sizeof( SteerConfig ) );
  EEPROM.put( ( uint16_t )EepromAddresses::SteerSettings, steerConfig );
  EEPROM.put( ( uint16_t )EepromAddresses::Bno055CalibrationData, bno055CalibrationData );
  EEPROM.commit();
}

void addGpioOutput( uint16_t parent ) {
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 4",        String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio4 ),  ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 12",       String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio12 ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 13 / A12", String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio13 ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 14",       String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio14 ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 15",       String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio15 ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 21",       String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio21 ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 25",       String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio25 ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 26",       String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio26 ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 27",       String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio27 ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 32 / A7",  String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio32 ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 33 / A9",  String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio33 ), ControlColor::Alizarin, parent );
}
void addGpioInput( uint16_t parent ) {
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 34 / A2 (only input)", String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio34 ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 36 / A4 (only input)", String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio36 ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 39 / A3 (only input)", String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio39 ), ControlColor::Alizarin, parent );
}
void addAnalogInput( uint16_t parent ) {
  ESPUI.addControl( ControlType::Option, "ESP32 A2",  String( ( uint8_t )SteerConfig::AnalogIn::Esp32GpioA2 ),  ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 A3",  String( ( uint8_t )SteerConfig::AnalogIn::Esp32GpioA3 ),  ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 A4",  String( ( uint8_t )SteerConfig::AnalogIn::Esp32GpioA4 ),  ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 A7",  String( ( uint8_t )SteerConfig::AnalogIn::Esp32GpioA7 ),  ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 A9",  String( ( uint8_t )SteerConfig::AnalogIn::Esp32GpioA9 ),  ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 A12", String( ( uint8_t )SteerConfig::AnalogIn::Esp32GpioA12 ), ControlColor::Alizarin, parent );
}
void addAnalogInputADS1115( uint16_t parent ) {
  ESPUI.addControl( ControlType::Option, "ADS1115 A0 Single",       String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A0Single ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ADS1115 A1 Single",       String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A1Single ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ADS1115 A2 Single",       String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A2Single ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ADS1115 A3 Single",       String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A3Single ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ADS1115 A0 Differential", String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A0A1Differential ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ADS1115 A2 Differential", String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A2A3Differential ), ControlColor::Alizarin, parent );
}

///////////////////////////////////////////////////////////////////////////
// Application
///////////////////////////////////////////////////////////////////////////
void setup( void ) {
//   Serial.begin( 921600 );
  Serial.begin( 115200 );
  Serial.println( "Setup()" );

  pinMode( 13, OUTPUT );
  digitalWrite( 13, LOW );

  // add all the partitions for the EEPROM emulation
  EEPROM.add_by_subtype( 0x99 );
  EEPROM.begin( 4096 );

  // restore the settings from EEPROM
  if ( ( EEPROM.readUChar( ( uint16_t )EepromAddresses::Validator ) != 0xff ) &&
       ( EEPROM.readUShort( ( uint16_t )EepromAddresses::SizeOfConfig ) == sizeof( SteerConfig ) ) ) {
    Serial.println( "Read from EEPROM" );
    EEPROM.get( ( uint16_t )EepromAddresses::SteerSettings, steerConfig );
    EEPROM.get( ( uint16_t )EepromAddresses::Bno055CalibrationData, bno055CalibrationData );
  } else {
    Serial.println( "Not read from EEPROM" );
    writeEeprom();
  }

#if defined(ESP32)
  WiFi.setHostname( steerConfig.hostname );
#else
  WiFi.hostname( steerConfig.hostname );
#endif

// try to connect to existing network
  WiFi.begin( steerConfig.ssid, steerConfig.password );
  Serial.print( "\n\nTry to connect to existing network " );
  Serial.print( steerConfig.ssid );
  Serial.print( " with password " );
  Serial.print( steerConfig.password );


  {
    uint8_t timeout = 5;

    // Wait for connection, 2.5s timeout
    do {
      delay( 500 );
      Serial.print( "." );
      timeout--;
    } while ( timeout && WiFi.status() != WL_CONNECTED );

    // not connected -> create hotspot
    if ( WiFi.status() != WL_CONNECTED ) {
      Serial.print( "\n\nCreating hotspot" );

      digitalWrite( 13, LOW );
      WiFi.mode( WIFI_AP );
      WiFi.softAPConfig( apIP, apIP, IPAddress( 255, 255, 255, 0 ) );
      WiFi.softAP( steerConfig.ssid );

      timeout = 5;

      do {
        delay( 500 );
        Serial.print( "." );
        timeout--;
      } while ( timeout );
    } else {
      digitalWrite( 13, HIGH );
    }
  }

  dnsServer.start( DNS_PORT, "*", apIP );

  Serial.println( "\n\nWiFi parameters:" );
  Serial.print( "Mode: " );
  Serial.println( WiFi.getMode() == WIFI_AP ? "Station" : "Client" );
  Serial.print( "IP address: " );
  Serial.println( WiFi.getMode() == WIFI_AP ? WiFi.softAPIP() : WiFi.localIP() );

// Comment in to update the Files in SPIFFS

  labelLoad       = ESPUI.addControl( ControlType::Label, "Load:", "", ControlColor::Turquoise );
  labelHeading    = ESPUI.addControl( ControlType::Label, "Heading:", "0°", ControlColor::Emerald );
  labelRoll       = ESPUI.addControl( ControlType::Label, "Roll:", "0°", ControlColor::Emerald );
  labelWheelAngle = ESPUI.addControl( ControlType::Label, "Wheel Angle:", "0°", ControlColor::Emerald );

  buttonReset = ESPUI.addControl( ControlType::Button, "Store the Settings", "Apply", ControlColor::Emerald, Control::noParent,
  []( Control * control, int id ) {
    if ( id == B_UP ) {
      writeEeprom();
    }
  } );

  buttonReset = ESPUI.addControl( ControlType::Button, "If this turn red, you have to", "Apply & Reset", ControlColor::Emerald, Control::noParent,
  []( Control * control, int id ) {
    if ( id == B_UP ) {
      writeEeprom();
      ESP.restart();
    }
  } );

  // Status Tab
  {
    uint16_t tab  = ESPUI.addControl( ControlType::Tab, "Status", "Status" );

    labelStatusOutput = ESPUI.addControl( ControlType::Label, "Output:", "No Output configured", ControlColor::Turquoise, tab );
    labelStatusAdc = ESPUI.addControl( ControlType::Label, "ADC:", "No ADC configured", ControlColor::Turquoise, tab );
    labelStatusImu = ESPUI.addControl( ControlType::Label, "IMU:", "No IMU configured", ControlColor::Turquoise, tab );
    labelStatusInclino = ESPUI.addControl( ControlType::Label, "Inclinometer:", "No Inclinometer configured", ControlColor::Turquoise, tab );
    labelStatusGps = ESPUI.addControl( ControlType::Label, "GPS:", "Not configured", ControlColor::Turquoise, tab );
    labelStatusNtrip = ESPUI.addControl( ControlType::Label, "NTRIP:", "Not configured", ControlColor::Turquoise, tab );
  }

  // Network Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Network", "Network" );

    ESPUI.addControl( ControlType::Text, "SSID*", String( steerConfig.ssid ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( steerConfig.ssid, sizeof( steerConfig.ssid ) );
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Text, "Password*", String( steerConfig.password ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( steerConfig.password, sizeof( steerConfig.password ) );
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Text, "Hostname*", String( steerConfig.hostname ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( steerConfig.hostname, sizeof( steerConfig.hostname ) );
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Number, "Port to send from*", String( steerConfig.portSendFrom ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      steerConfig.portSendFrom = control->value.toInt();
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Number, "Port to send to*", String( steerConfig.portSendTo ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      steerConfig.portSendTo = control->value.toInt();
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Number, "Port to listen to*", String( steerConfig.portListenTo ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      steerConfig.portListenTo = control->value.toInt();
      setResetButtonToRed();
    } );
  }

  // Switches/Buttons Tab
  {
    uint16_t tab  = ESPUI.addControl( ControlType::Tab, "Switches/Buttons", "Switches/Buttons" );

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Input Workswitch*", String( ( int )steerConfig.gpioWorkswitch ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.gpioWorkswitch = ( SteerConfig::Gpio )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      addGpioInput( sel );
      addGpioOutput( sel );
    }
    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Workswitch LED*", String( ( int )steerConfig.gpioWorkswitch ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.gpioWorkswitch = ( SteerConfig::Gpio )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      addGpioOutput( sel );
    }

    ESPUI.addControl( ControlType::Switcher, "Autosteer input as Button*", String( steerConfig.autosteerButton ? "1" : "0" ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      steerConfig.autosteerButton = control->value.toInt() == 1;
    } );
    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Input Autosteer switch/button*", String( ( int )steerConfig.gpioSteerswitch ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.gpioSteerswitch = ( SteerConfig::Gpio )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      addGpioInput( sel );
      addGpioOutput( sel );
    }
    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Autosteer LED*", String( ( int )steerConfig.gpioSteerswitch ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.gpioSteerswitch = ( SteerConfig::Gpio )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      addGpioOutput( sel );
    }
  }

  // Steering Tab
  {
    uint16_t tab  = ESPUI.addControl( ControlType::Tab, "Steering", "Steering" );

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Output Type*", String( ( int )steerConfig.outputType ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.outputType = ( SteerConfig::OutputType )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Motor: Cytron MD30C", "1", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Motor: IBT 2", "2", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Hydraulic: IBT 2 + PWM 2-Coil Valve", "3", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Hydraulic: IBT 2 + Danfoss Valve PVE A/H/M", "4", ControlColor::Alizarin, sel );
    }

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Output Pin for PWM (or right coil)*", String( ( int )steerConfig.gpioPwm ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.gpioPwm = ( SteerConfig::Gpio )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      addGpioOutput( sel );
    }
    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Output Pin for Dir (or left coil)*", String( ( int )steerConfig.gpioDir ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.gpioDir = ( SteerConfig::Gpio )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      addGpioOutput( sel );
    }
    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Output Pin for Enable*", String( ( int )steerConfig.gpioEn ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.gpioEn = ( SteerConfig::Gpio )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      addGpioOutput( sel );
    }

    ESPUI.addControl( ControlType::Switcher, "Invert Output", String( steerConfig.invertOutput ? "1" : "0" ), ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      steerConfig.invertOutput = control->value.toInt() == 1;
    } );

    {
      uint16_t num = ESPUI.addControl( ControlType::Slider, "PWM Frequency", String( steerConfig.pwmFrequency ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.pwmFrequency = control->value.toInt();

        switch ( initialisation.outputType ) {
          case SteerConfig::OutputType::SteeringMotorIBT2: {
            ledcWriteTone( 0, steerConfig.pwmFrequency );
            ledcWriteTone( 1, steerConfig.pwmFrequency );
          }
          break;
        }
      } );
      ESPUI.addControl( ControlType::Min, "Min", String( "1000" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", String( "5000" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", String( "100" ), ControlColor::Peterriver, num );
    }
  }

  // Steering PID Tab
  {
    uint16_t tab  = ESPUI.addControl( ControlType::Tab, "Steering PID", "Steering PID" );

    ESPUI.addControl( ControlType::Switcher, "Allow AgOpenGPS to overwrite PID values", String( steerConfig.allowPidOverwrite ? "1" : "0" ), ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      steerConfig.allowPidOverwrite = control->value.toInt() == 1;
    } );

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "PID Kp", String( steerConfig.steeringPidKp, 4 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidKp = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", String( "0" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", String( "50" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", String( "0.1" ), ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "PID Ki", String( steerConfig.steeringPidKi, 4 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidKi = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", String( "0" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", String( "50" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", String( "0.01" ), ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "PID Kd", String( steerConfig.steeringPidKd, 4 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidKd = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", String( "0" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", String( "50" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", String( "0.01" ), ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Turn Output on if error is greater (BangOn)", String( steerConfig.steeringPidBangOn, 4 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidBangOn = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", String( "0" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", String( "50" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", String( "0.01" ), ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Turn Output off if error is smaller (BangOff)", String( steerConfig.steeringPidBangOff, 4 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidBangOff = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", String( "0" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", String( "50" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", String( "0.01" ), ControlColor::Peterriver, num );
    }

//     {
//       uint16_t num = ESPUI.addControl( ControlType::Number, "Distance from Line to turn Integral/Derivative of PID off", String( steerConfig.steeringPidDflTurnIdOff ), ControlColor::Peterriver, tab,
//       []( Control * control, int id ) {
//         steerConfig.steeringPidDflTurnIdOff = control->value.toInt();
//       } );
//       ESPUI.addControl( ControlType::Min, "Min", String( "0" ), ControlColor::Peterriver, num );
//       ESPUI.addControl( ControlType::Max, "Max", String( "200" ), ControlColor::Peterriver, num );
//       ESPUI.addControl( ControlType::Step, "Step", String( "1" ), ControlColor::Peterriver, num );
//     }
  }

  // Wheel Angle Sensor Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Wheel Angle Sensor", "Wheel Angle Sensor" );

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Wheel Angle Sensor*", String( ( int )steerConfig.wheelAngleInput ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleInput = ( SteerConfig::AnalogIn )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      addAnalogInputADS1115( sel );
      addAnalogInput( sel );
    }

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Wheel Angle Sensor Type", String( ( int )steerConfig.wheelAngleSensorType ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleSensorType = ( SteerConfig::WheelAngleSensorType )control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Option, "Direct Wheel Angle", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Two Arms connected to tie rod", "1", ControlColor::Alizarin, sel );
    }

    ESPUI.addControl( ControlType::Switcher, "Allow AgOpenGPS to overwrite Counts per Degree and Steer Angle Center", String( steerConfig.allowWheelAngleCenterAndCountsOverwrite ? "1" : "0" ), ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      steerConfig.allowWheelAngleCenterAndCountsOverwrite = control->value.toInt() == 1;
    } );

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Wheel Angle Sensor Center", String( steerConfig.wheelAnglePositionZero ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAnglePositionZero = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", String( "0" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", String( "26000" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", String( "1" ), ControlColor::Peterriver, num );
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Wheel Angle Counts per Degree", String( steerConfig.wheelAngleCountsPerDegree ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleCountsPerDegree = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", String( "0" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", String( "250" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", String( "0.1" ), ControlColor::Peterriver, num );
    }

    ESPUI.addControl( ControlType::Switcher, "Invert Wheel Angle Sensor", String( steerConfig.invertWheelAngleSensor ? "1" : "0" ), ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      steerConfig.invertWheelAngleSensor = control->value.toInt() == 1;
    } );

    {
      uint16_t num = ESPUI.addControl( ControlType::Slider, "Wheel Angle Offset", String( steerConfig.wheelAngleOffset ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleOffset = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Roll Min", String( "-40" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Roll Max", String( "40" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Roll Step", String( "0.1" ), ControlColor::Peterriver, num );
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "1. Arm connect to sensor (mm)", String( steerConfig.wheelAngleFirstArmLenght ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleFirstArmLenght = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", String( "0" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", String( "500" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", String( "1" ), ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "2. Arm connect to sensor (mm)", String( steerConfig.wheelAngleSecondArmLenght ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleSecondArmLenght = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", String( "0" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", String( "500" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", String( "1" ), ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Tie rod stroke (mm)", String( steerConfig.wheelAngleTieRodStroke ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleTieRodStroke = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", String( "0" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", String( "500" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", String( "1" ), ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Minimum Angle", String( steerConfig.wheelAngleMinimumAngle ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleMinimumAngle = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", String( "0" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", String( "180" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", String( "1" ), ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Lenght of Track Arm (mm)", String( steerConfig.wheelAngleTrackArmLenght ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleTrackArmLenght = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", String( "0" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", String( "500" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", String( "1" ), ControlColor::Peterriver, num );
    }
    labelWheelAngleDisplacement = ESPUI.addControl( ControlType::Label, "Displacement:", "0°", ControlColor::Emerald );
  }

  // Sensors Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Sensors", "Sensors" );

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "IMU*", String( ( int )steerConfig.imuType ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.imuType = ( SteerConfig::ImuType )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "No IMU", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "BNO055", "1", ControlColor::Alizarin, sel );
    }
    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "IMU Orientation", String( ( int )steerConfig.imuOrientation ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.imuOrientation = ( SteerConfig::ImuOrientation )control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Option, "Forwards", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Right", "1", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Backwards", "2", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Left", "3", ControlColor::Alizarin, sel );
    }

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Inclinometer*", String( ( int )steerConfig.inclinoType ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.inclinoType = ( SteerConfig::InclinoType )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "No Inclinometer", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "MMA8451", "1", ControlColor::Alizarin, sel );
//       ESPUI.addControl( ControlType::Option, "DOGS2", "2", ControlColor::Alizarin, sel );
    }
    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Inclinometer Orientation (Y-axis points:)", String( ( int )steerConfig.inclinoOrientation ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.inclinoOrientation = ( SteerConfig::InclinoOrientation )control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Option, "Forwards", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Backwards", "1", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Left", "2", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Right", "3", ControlColor::Alizarin, sel );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Slider, "Roll Offset", String( steerConfig.rollOffset ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.rollOffset = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Roll Min", String( "-10" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Roll Max", String( "10" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Roll Step", String( "0.01" ), ControlColor::Peterriver, num );
    }
  }

  // Steering Wheel Encoder Tab
  {
    uint16_t tab  = ESPUI.addControl( ControlType::Tab, "Steering Wheel Encoder", "Steering Wheel Encoder" );

    ESPUI.addControl( ControlType::Switcher, "Steering Wheel Encoder*", String( steerConfig.steeringWheelEncoder ? "1" : "0" ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      steerConfig.steeringWheelEncoder = control->value.toInt() == 1;
      setResetButtonToRed();
    } );

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Steering Wheel Encoder Input A*", String( ( int )steerConfig.gpioWheelencoderA ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.gpioWheelencoderA = ( SteerConfig::Gpio )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      addGpioInput( sel );
      addGpioOutput( sel );
    }
    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Steering Wheel Encoder Input B*", String( ( int )steerConfig.gpioWheelencoderB ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.gpioWheelencoderB = ( SteerConfig::Gpio )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      addGpioInput( sel );
      addGpioOutput( sel );
    }

    ESPUI.addControl( ControlType::Number, "Steering Wheel Encoder max Counts", String( steerConfig.wheelEncoderPulseCountMax ), ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      steerConfig.wheelEncoderPulseCountMax = control->value.toInt();
    } );
  }

  // NTRIP/GPS Tab
  {
    uint16_t tab     = ESPUI.addControl( ControlType::Tab, "NTRIP/GPS", "NTRIP/GPS" );

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "RTK Correction*", String( ( int )steerConfig.rtkCorrectionType ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.rtkCorrectionType = ( SteerConfig::RtkCorrectionType )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "No Correction", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "NTRIP", "1", ControlColor::Alizarin, sel );
//       ESPUI.addControl( ControlType::Option, "UDP", "2", ControlColor::Alizarin, sel );
//       ESPUI.addControl( ControlType::Option, "TCP", "3", ControlColor::Alizarin, sel );
    }

    ESPUI.addControl( ControlType::Text, "Server*", String( steerConfig.rtkCorrectionServer ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( steerConfig.rtkCorrectionServer, sizeof( steerConfig.rtkCorrectionServer ) );
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Text, "Username*", String( steerConfig.rtkCorrectionUsername ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( steerConfig.rtkCorrectionUsername, sizeof( steerConfig.rtkCorrectionUsername ) );
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Text, "Password*", String( steerConfig.rtkCorrectionPassword ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( steerConfig.rtkCorrectionPassword, sizeof( steerConfig.rtkCorrectionPassword ) );
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Text, "Mountpoint*", String( steerConfig.rtkCorrectionMountpoint ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( steerConfig.rtkCorrectionMountpoint, sizeof( steerConfig.rtkCorrectionMountpoint ) );
      setResetButtonToRed();
    } );
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Port*",  String( steerConfig.rtkCorrectionPort ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.rtkCorrectionPort = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", String( "1" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", String( "65535" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", String( "1" ), ControlColor::Peterriver, num );
    }

    {
      uint16_t baudrate = ESPUI.addControl( ControlType::Select, "Baudrate*", String( steerConfig.rtkCorrectionBaudrate ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        uint32_t baudrate = control->value.toInt();
        steerConfig.rtkCorrectionBaudrate = baudrate;
        Serial2.flush();
        Serial2.begin( baudrate );
      } );
      ESPUI.addControl( ControlType::Option, "9600", "9600", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "19200", "19200", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "38400", "38400", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "57600", "57600", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "115200", "115200", ControlColor::Alizarin, baudrate );
    }

    ESPUI.addControl( ControlType::Number, "Intervall to send Position", String( steerConfig.ntripPositionSendIntervall ), ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      steerConfig.ntripPositionSendIntervall = control->value.toInt();
    } );

    textNmeaToSend = ESPUI.addControl( ControlType::Text, "NMEA-String to send (leave empty to send live position)", String( steerConfig.rtkCorrectionNmeaToSend ), ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      control->value.toCharArray( steerConfig.rtkCorrectionNmeaToSend, sizeof( steerConfig.rtkCorrectionNmeaToSend ) );
    } );

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Send NMEA-data to", String( ( int )steerConfig.sendNmeaDataTo ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.sendNmeaDataTo = ( SteerConfig::SendNmeaDataTo )control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Option, "Nowhere", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "UDP", "1", ControlColor::Alizarin, sel );
//       ESPUI.addControl( ControlType::Option, "TCP", "2", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Serial", "3", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Serial1", "4", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Serial2", "5", ControlColor::Alizarin, sel );
//       ESPUI.addControl( ControlType::Option, "Bluetooth", "6", ControlColor::Alizarin, sel );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Port for TCP (set to 0 to deactivate)*",  String( steerConfig.sendNmeaDataTcpPort ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.sendNmeaDataTcpPort = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", String( "0" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", String( "65535" ), ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", String( "1" ), ControlColor::Peterriver, num );
    }
  }

  i2cMutex = xSemaphoreCreateMutex();

  /*
   * .begin loads and serves all files from PROGMEM directly.
   * If you want to serve the files from SPIFFS use ESPUI.beginSPIFFS
   * (.prepareFileSystem has to be run in an empty sketch before)
   */

  /*
   * Optionally you can use HTTP BasicAuth. Keep in mind that this is NOT a
   * SECURE way of limiting access.
   * Anyone who is able to sniff traffic will be able to intercept your password
   * since it is transmitted in cleartext. Just add a username and password,
   * for example begin("ESPUI Control", "username", "password")
   */
  String title = "AOG Control :: ";
  title += steerConfig.hostname;
  ESPUI.begin( title.c_str() );

  initIdleStats();

  initSensors();
  initRtkCorrection();

  initAutosteer();
}

void loop( void ) {
  dnsServer.processNextRequest();
  vTaskDelay( 100 );
}

