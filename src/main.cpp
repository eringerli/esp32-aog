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

#include "settings.hpp"
#include "main.hpp"

#if defined(ESP32)
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif

#include <DNSServer.h>
#include <ESPUI.h>

///////////////////////////////////////////////////////////////////////////
// global data
///////////////////////////////////////////////////////////////////////////
SteerConfig steerConfig, steerConfigDefaults;
Initialisation initialisation;
SteerCanData steerCanData = {0};

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t i2cMutex;

const byte DNS_PORT = 53;
IPAddress apIP( 192, 168, 1, 1 );

uint16_t labelLoad;
uint16_t labelOrientation;
uint16_t labelWheelAngle;
uint16_t buttonReset;
uint16_t textNmeaToSend;

uint16_t labelWheelAngleDisplacement;

uint16_t labelStatusOutput;
uint16_t labelStatusAdc;
uint16_t labelStatusCan;
uint16_t labelStatusImu;
uint16_t labelStatusInclino;
uint16_t labelStatusGps;
uint16_t labelStatusNtrip;

///////////////////////////////////////////////////////////////////////////
// external Libraries
///////////////////////////////////////////////////////////////////////////
ESPUIClass ESPUI( Verbosity::Quiet );
DNSServer dnsServer;

///////////////////////////////////////////////////////////////////////////
// helper functions
///////////////////////////////////////////////////////////////////////////
void setResetButtonToRed() {
  ESPUI.getControl( buttonReset )->color = ControlColor::Alizarin;
  ESPUI.updateControlAsync( buttonReset );
}

void saveConfigToSPIFFS() {
}

void addGpioOutput( uint16_t parent ) {
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 4",        String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio4 ),  ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 5",        String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio5 ),  ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 12",       String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio12 ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 13 / A12", String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio13 ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 14",       String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio14 ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 15",       String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio15 ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 21",       String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio21 ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 22",       String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio22 ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 23",       String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio23 ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 25",       String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio25 ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 26",       String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio26 ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 27",       String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio27 ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 32 / A7",  String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio32 ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 33 / A9",  String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio33 ), ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "ESP32 GPIO 35",       String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio35 ), ControlColor::Alizarin, parent );
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

  Wire.begin( ( int )steerConfig.gpioSDA, ( int )steerConfig.gpioSCL, steerConfig.i2cBusSpeed );

  if( !SPIFFS.begin( true ) ) {
    Serial.println( "SPIFFS Mount Failed" );
    return;
  }

  loadSavedConfig();

  if( steerConfig.apModePin != SteerConfig::Gpio::None ) {
    pinMode( ( int )steerConfig.apModePin, OUTPUT );
    digitalWrite( ( int )steerConfig.apModePin, LOW );
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
    } while( timeout && WiFi.status() != WL_CONNECTED );

    // not connected -> create hotspot
    if( WiFi.status() != WL_CONNECTED ) {
      Serial.print( "\n\nCreating hotspot" );

      if( steerConfig.apModePin != SteerConfig::Gpio::None ) {
        digitalWrite( ( int )steerConfig.apModePin, LOW );
      }

      WiFi.mode( WIFI_AP );
      WiFi.softAPConfig( apIP, apIP, IPAddress( 255, 255, 255, 0 ) );
      WiFi.softAP( steerConfig.ssid );

      timeout = 5;

      do {
        delay( 500 );
        Serial.print( "." );
        timeout--;
      } while( timeout );
    } else {
      if( steerConfig.apModePin != SteerConfig::Gpio::None ) {
        digitalWrite( ( int )steerConfig.apModePin, HIGH );
      }
    }
  }

  dnsServer.start( DNS_PORT, "*", apIP );

  Serial.println( "\n\nWiFi parameters:" );
  Serial.print( "Mode: " );
  Serial.println( WiFi.getMode() == WIFI_AP ? "Station" : "Client" );
  Serial.print( "IP address: " );
  Serial.println( WiFi.getMode() == WIFI_AP ? WiFi.softAPIP() : WiFi.localIP() );


  steerImuInclinometerData.sendCalibrationDataFromImu = false;

  labelLoad       = ESPUI.addControl( ControlType::Label, "Load:", "", ControlColor::Turquoise );
  labelOrientation = ESPUI.addControl( ControlType::Label, "Orientation:", "", ControlColor::Emerald );
  labelWheelAngle = ESPUI.addControl( ControlType::Label, "Wheel Angle:", "0°", ControlColor::Emerald );
//   graphWheelAngle = ESPUI.addControl( ControlType::Graph, "Wheel Angle:", "", ControlColor::Emerald );

  buttonReset = ESPUI.addControl( ControlType::Button, "Store the Settings", "Apply", ControlColor::Emerald, Control::noParent,
  []( Control * control, int id ) {
    if( id == B_UP ) {
      saveConfig();
    }
  } );

  buttonReset = ESPUI.addControl( ControlType::Button, "If this turn red, you have to", "Apply & Reboot", ControlColor::Emerald, Control::noParent,
  []( Control * control, int id ) {
    if( id == B_UP ) {
      saveConfig();
      ESP.restart();
    }
  } );

  // Status Tab
  {
    uint16_t tab  = ESPUI.addControl( ControlType::Tab, "Status", "Status" );

    labelStatusOutput   = ESPUI.addControl( ControlType::Label, "Output:", "No Output configured", ControlColor::Turquoise, tab );
    labelStatusAdc      = ESPUI.addControl( ControlType::Label, "ADC:", "No ADC configured", ControlColor::Turquoise, tab );
    labelStatusCan      = ESPUI.addControl( ControlType::Label, "CAN:", "No CAN BUS configured", ControlColor::Turquoise, tab );
    labelStatusImu      = ESPUI.addControl( ControlType::Label, "IMU:", "No IMU configured", ControlColor::Turquoise, tab );
    labelStatusInclino  = ESPUI.addControl( ControlType::Label, "Inclinometer:", "No Inclinometer configured", ControlColor::Turquoise, tab );
    labelStatusGps      = ESPUI.addControl( ControlType::Label, "GPS:", "Not configured", ControlColor::Turquoise, tab );
    labelStatusNtrip    = ESPUI.addControl( ControlType::Label, "NTRIP:", "Not configured", ControlColor::Turquoise, tab );
  }

  // Info Tab
  {
    uint16_t tab  = ESPUI.addControl( ControlType::Tab, "Info/Help", "Info/Help" );
    ESPUI.addControl( ControlType::Label, "Basics:", "This works with setting up the different options in the panels. If an option requires a reboot (indicated by the darker blue and an asterisk after the title), press on the button \"Apply & Reboot\" and refresh the page after some time, usually 5-10 seconds. Settings with the lighter shade of blue are applied immediately, but are not saved to the permanent memory. You can do this with the \"Apply\" button. If the values are complete garbage or you want a fresh start, set the config to defaults in the \"Configurations\" tab.", ControlColor::Carrot, tab );

    ESPUI.addControl( ControlType::Label, "Network:", "Here the network is configured. Leave it on defaults, only if used as roof controller (only GPS + IMU), set \"Port to send from\" to 5544.", ControlColor::Turquoise, tab );
    ESPUI.addControl( ControlType::Label, "CAN Bus/J1939:", "Enable if used. To use data from the vehicle bus as workswitch, configure it in the next tab.", ControlColor::Turquoise, tab );
    ESPUI.addControl( ControlType::Label, "Work- and Steerswitch:", "If work- and steerswitches as physical inputs are used, enable them by configuring a GPIO. If you want to use the CAN-bus (J1939), set the type to a hitch position or RPM with a threshold.", ControlColor::Turquoise, tab );
    ESPUI.addControl( ControlType::Label, "Wheel Angle Sensor:", "To enable the wheel angle sensor, configure the input first. If you use two arms connected to the tie rod, measure them exactly and configure the values. This is to calculate out the unlinearities.", ControlColor::Turquoise, tab );
    ESPUI.addControl( ControlType::Label, "Steering:", "Set up the type and the GPIOs", ControlColor::Turquoise, tab );
    ESPUI.addControl( ControlType::Label, "Steering PID:", "This controller uses its own PID-controller. No values are taken over from AOG, so everything is entered here.", ControlColor::Turquoise, tab );
    ESPUI.addControl( ControlType::Label, "Sensors:", "Here the IMU and inclinometer are set up. The Mounting Correction is entered as three angles relative to the tractor axis, so the IMU can be mounted in every position, as long as the chips are positioned relative to each other with no difference (normaly, the manufacturer of the sensor pcb does this anyway). The FXAS2100/FXOS8700-combo is recomned, as they give the most precise roll/pitch/heading with the least amount of calibration.", ControlColor::Turquoise, tab );
    ESPUI.addControl( ControlType::Label, "NTRIP/GPS:", "Here the connection to the GPS is set up, also the NTRIP-client. Usualy, you want to send the data to AOG via UDP, a serial connection via USB is also possible. The TCP-Socket enables 3rd-party GPS-Software and configuring the GPS-Receiver with u-center.", ControlColor::Turquoise, tab );
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

    ESPUI.addControl( ControlType::Text, "Pin to show AP mode*",  String( ( int )steerConfig.apModePin ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      steerConfig.apModePin = ( SteerConfig::Gpio )control->value.toInt();
      setResetButtonToRed();
    } );

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Mode*", String( ( int )steerConfig.canBusSpeed ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.canBusSpeed = ( SteerConfig::CanBusSpeed )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "QtOpenGuidance", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "AgOpenGps", "1", ControlColor::Alizarin, sel );
    }

    ESPUI.addControl( ControlType::Number, "Port to send from*", String( steerConfig.aogPortSendFrom ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      steerConfig.aogPortSendFrom = control->value.toInt();
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Number, "Port to send to*", String( steerConfig.aogPortSendTo ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      steerConfig.aogPortSendTo = control->value.toInt();
      setResetButtonToRed();
    } );
    ESPUI.addControl( ControlType::Number, "Port to listen to*", String( steerConfig.aogPortListenTo ), ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      steerConfig.aogPortListenTo = control->value.toInt();
      setResetButtonToRed();
    } );
  }

  // CAN Bus
  {
    uint16_t tab  = ESPUI.addControl( ControlType::Tab, "CAN Bus/J1939", "CAN Bus/J1939" );

    ESPUI.addControl( ControlType::Switcher, "CAN Bus Enabled*", steerConfig.canBusEnabled ? "1" : "0", ControlColor::Wetasphalt, tab,
    []( Control * control, int id ) {
      steerConfig.canBusEnabled = control->value.toInt() == 1;
      setResetButtonToRed();
    } );

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Pin for RX*", String( ( int )steerConfig.canBusRx ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.canBusRx = ( SteerConfig::Gpio )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      addGpioOutput( sel );
    }
    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Pin for TX*", String( ( int )steerConfig.canBusTx ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.canBusTx = ( SteerConfig::Gpio )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      addGpioOutput( sel );
    }

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Bus Speed*", String( ( int )steerConfig.canBusSpeed ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.canBusSpeed = ( SteerConfig::CanBusSpeed )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "250kB/s", "250", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "500kB/s", "500", ControlColor::Alizarin, sel );
    }
  }

  // Switches/Buttons Tab
  {
    uint16_t tab  = ESPUI.addControl( ControlType::Tab, "Work- and Steerswitch", "Work- and Steerswitch" );

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Workswitch Type", String( ( int )steerConfig.workswitchType ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.workswitchType = ( SteerConfig::WorkswitchType )control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Gpio", "1", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Rear Hitch Position (from Can Bus)", "2", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Front Hitch Position (from Can Bus)", "3", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Rear Pto Rpm (from Can Bus)", "4", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Front Pto Rpm (from Can Bus)", "5", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "Motor Rpm (from Can Bus)", "6", ControlColor::Alizarin, sel );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Hitch Threshold",  String( steerConfig.canBusHitchThreshold ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.canBusHitchThreshold = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "100", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Hitch Threshold Hysteresis",  String( steerConfig.canBusHitchThresholdHysteresis ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.canBusHitchThresholdHysteresis = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "100", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "RPM Threshold",  String( steerConfig.canBusHitchThreshold ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.canBusHitchThreshold = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "3500", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "RPM Threshold Hysteresis",  String( steerConfig.canBusHitchThresholdHysteresis ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.canBusHitchThresholdHysteresis = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "1000", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
//     {
//       uint16_t sel = ESPUI.addControl( ControlType::Select, "Workswitch LED*", String( ( int )steerConfig.gpioWorkswitch ), ControlColor::Wetasphalt, tab,
//       []( Control * control, int id ) {
//         steerConfig.gpioWorkswitch = ( SteerConfig::Gpio )control->value.toInt();
//         setResetButtonToRed();
//       } );
//       ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
//       addGpioOutput( sel );
//     }

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Autosteer switch/button Gpio*", String( ( int )steerConfig.gpioSteerswitch ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.gpioSteerswitch = ( SteerConfig::Gpio )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      addGpioInput( sel );
      addGpioOutput( sel );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Auto recognise Autosteer GPIO as Switch [ms]",  String( steerConfig.autoRecogniseSteerGpioAsSwitchOrButton ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.autoRecogniseSteerGpioAsSwitchOrButton = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "16000", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "100", ControlColor::Peterriver, num );
    }
    {
      ESPUI.addControl( ControlType::Switcher, "Steerswitch Active Low", steerConfig.steerswitchActiveLow ? "1" : "0", ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steerswitchActiveLow = control->value.toInt() == 1;
      } );
    }
//     {
//       uint16_t sel = ESPUI.addControl( ControlType::Select, "Autosteer LED*", String( ( int )steerConfig.gpioSteerswitch ), ControlColor::Wetasphalt, tab,
//       []( Control * control, int id ) {
//         steerConfig.gpioSteerswitch = ( SteerConfig::Gpio )control->value.toInt();
//         setResetButtonToRed();
//       } );
//       ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
//       addGpioOutput( sel );
//     }


    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "Workswitch Gpio*", String( ( int )steerConfig.gpioWorkswitch ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.gpioWorkswitch = ( SteerConfig::Gpio )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
      addGpioInput( sel );
      addGpioOutput( sel );
    }
    {
      ESPUI.addControl( ControlType::Switcher, "Workswitch Active Low", steerConfig.workswitchActiveLow ? "1" : "0", ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.workswitchActiveLow = control->value.toInt() == 1;
      } );
    }

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

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Wheel Angle Sensor Center", String( steerConfig.wheelAnglePositionZero ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAnglePositionZero = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "26000", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Wheel Angle Counts per Degree", String( steerConfig.wheelAngleCountsPerDegree ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleCountsPerDegree = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "250", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.1", ControlColor::Peterriver, num );
    }

    ESPUI.addControl( ControlType::Switcher, "Invert Wheel Angle Sensor", steerConfig.invertWheelAngleSensor ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      steerConfig.invertWheelAngleSensor = control->value.toInt() == 1;
    } );

    {
      uint16_t num = ESPUI.addControl( ControlType::Slider, "Wheel Angle Offset", String( steerConfig.wheelAngleOffset ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleOffset = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Roll Min", "-40", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Roll Max", "40", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Roll Step", "0.1", ControlColor::Peterriver, num );
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "1. Arm connect to sensor (mm)", String( steerConfig.wheelAngleFirstArmLenght ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleFirstArmLenght = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "2. Arm connect to tie rod (mm)", String( steerConfig.wheelAngleSecondArmLenght ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleSecondArmLenght = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Tie rod stroke (mm)", String( steerConfig.wheelAngleTieRodStroke ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleTieRodStroke = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Minimum Angle of wheel angle sensor", String( steerConfig.wheelAngleMinimumAngle ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleMinimumAngle = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "180", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Lenght of Track Arm (mm)", String( steerConfig.wheelAngleTrackArmLenght ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.wheelAngleTrackArmLenght = control->value.toFloat();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "500", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
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

    ESPUI.addControl( ControlType::Switcher, "Invert Output", steerConfig.invertOutput ? "1" : "0", ControlColor::Peterriver, tab,
    []( Control * control, int id ) {
      steerConfig.invertOutput = control->value.toInt() == 1;
    } );

//     {
//       uint16_t num = ESPUI.addControl( ControlType::Slider, "PWM Frequency", String( steerConfig.pwmFrequency ), ControlColor::Peterriver, tab,
//       []( Control * control, int id ) {
//         steerConfig.pwmFrequency = control->value.toInt();
//
//         switch ( initialisation.outputType ) {
//           case SteerConfig::OutputType::SteeringMotorIBT2: {
//             ledcWriteTone( 0, steerConfig.pwmFrequency );
//             ledcWriteTone( 1, steerConfig.pwmFrequency );
//           }
//           break;
//         }
//       } );
//       ESPUI.addControl( ControlType::Min, "Min", "1000", ControlColor::Peterriver, num );
//       ESPUI.addControl( ControlType::Max, "Max", "5000", ControlColor::Peterriver, num );
//       ESPUI.addControl( ControlType::Step, "Step", "100", ControlColor::Peterriver, num );
//     }
  }

  // Steering PID Tab
  {
    uint16_t tab  = ESPUI.addControl( ControlType::Tab, "Steering PID", "Steering PID" );

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "PID Kp", String( steerConfig.steeringPidKp, 4 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidKp = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.1", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "PID Ki", String( steerConfig.steeringPidKi, 4 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidKi = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.01", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "PID Kd", String( steerConfig.steeringPidKd, 4 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidKd = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.01", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Automatic Bang On Factor (multiple of saturation with Kp, 0 to turn off)", String( steerConfig.steeringPidAutoBangOnFactor, 4 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidAutoBangOnFactor = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "10", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.1", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Turn Output on if error is greater (BangOn)", String( steerConfig.steeringPidBangOn, 4 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidBangOn = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.01", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Turn Output off if error is smaller (BangOff)", String( steerConfig.steeringPidBangOff, 4 ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidBangOff = control->value.toDouble();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "50", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.01", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Minimum PWM", String( steerConfig.steeringPidMinPwm ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.steeringPidMinPwm = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "255", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
  }

  // Sensors Tab
  {
    uint16_t tab = ESPUI.addControl( ControlType::Tab, "Sensors", "Sensors" );

    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "I2C SDA Gpio*", String( ( int )steerConfig.gpioSDA ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.gpioSDA = ( SteerConfig::Gpio )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "Board Default", "-1", ControlColor::Alizarin, sel );
      addGpioOutput( sel );
    }
    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "I2C SCL Gpio*", String( ( int )steerConfig.gpioSCL ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.gpioSCL = ( SteerConfig::Gpio )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "Board Default", "-1", ControlColor::Alizarin, sel );
      addGpioOutput( sel );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "I2C Bus Speed*",  String( steerConfig.i2cBusSpeed ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.i2cBusSpeed = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "10000", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "5000000", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1000", ControlColor::Peterriver, num );
    }
    {
      uint16_t sel = ESPUI.addControl( ControlType::Select, "IMU*", String( ( int )steerConfig.imuType ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.imuType = ( SteerConfig::ImuType )control->value.toInt();
        setResetButtonToRed();
      } );
      ESPUI.addControl( ControlType::Option, "No IMU", "0", ControlColor::Alizarin, sel );
      ESPUI.addControl( ControlType::Option, "FXOS8700/FXAS21002", "2", ControlColor::Alizarin, sel );
    }
    {
      ESPUI.addControl( ControlType::Switcher, "Send Calibration Data from IMU to USB", steerImuInclinometerData.sendCalibrationDataFromImu ? "1" : "0", ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerImuInclinometerData.sendCalibrationDataFromImu = control->value.toInt() == 1;
      } );
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
      ESPUI.addControl( ControlType::Option, "FXOS8700/FXAS21002", "3", ControlColor::Alizarin, sel );
    }

    {
      ESPUI.addControl( ControlType::Switcher, "Invert Roll Axis (enable for older versions of AgOpenGPS)", steerConfig.invertRoll ? "1" : "0", ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.invertRoll = control->value.toInt() == 1;
      } );
    }

    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Mounting Correction (Roll) of Imu", String( steerConfig.mountCorrectionImuRoll ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.mountCorrectionImuRoll = control->value.toFloat();
        calculateMountingCorrection();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "-180", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "180", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.05", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Mounting Correction (Pitch) of Imu", String( steerConfig.mountCorrectionImuPitch ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.mountCorrectionImuPitch = control->value.toFloat();
        calculateMountingCorrection();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "-180", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "180", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.05", ControlColor::Peterriver, num );
    }
    {
      uint16_t num = ESPUI.addControl( ControlType::Number, "Mounting Correction (Yaw) of Imu", String( steerConfig.mountCorrectionImuYaw ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.mountCorrectionImuYaw = control->value.toFloat();
        calculateMountingCorrection();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "-180", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "180", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "0.05", ControlColor::Peterriver, num );
    }
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
      ESPUI.addControl( ControlType::Min, "Min", "1", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "65535", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }

    {
      uint16_t baudrate = ESPUI.addControl( ControlType::Select, "Baudrate", String( steerConfig.rtkCorrectionBaudrate ), ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        uint32_t baudrate = control->value.toInt();
        steerConfig.rtkCorrectionBaudrate = baudrate;
        Serial2.updateBaudRate( baudrate );
      } );
      ESPUI.addControl( ControlType::Option, "4800", "4800", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "9600", "9600", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "19200", "19200", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "38400", "38400", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "57600", "57600", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "115200", "115200", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "230400", "230400", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "460800", "460800", ControlColor::Alizarin, baudrate );
      ESPUI.addControl( ControlType::Option, "921600", "921600", ControlColor::Alizarin, baudrate );
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
      uint16_t num = ESPUI.addControl( ControlType::Number, "TCP-Socket for a direct connection to the GPS-Receiver (set to 0 to deactivate, can be used for configuration with u-center or with 3rd-party software)*",  String( steerConfig.sendNmeaDataTcpPort ), ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        steerConfig.sendNmeaDataTcpPort = control->value.toInt();
      } );
      ESPUI.addControl( ControlType::Min, "Min", "0", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Max, "Max", "65535", ControlColor::Peterriver, num );
      ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, num );
    }
  }

  // Default Configurations Tab
  {
    uint16_t tab     = ESPUI.addControl( ControlType::Tab, "Configurations", "Configurations" );
    ESPUI.addControl( ControlType::Label, "Attention:", "These Buttons here reset the whole config. This affects the WIFI too, if not configured otherwise below. You have to press \"Apply & Reboot\" above to actualy store them.", ControlColor::Carrot, tab );

    {
      ESPUI.addControl( ControlType::Switcher, "Retain WIFI settings", steerConfig.retainWifiSettings ? "1" : "0", ControlColor::Peterriver, tab,
      []( Control * control, int id ) {
        steerConfig.retainWifiSettings = control->value.toInt() == 1;
      } );
    }
    {
      ESPUI.addControl( ControlType::Button, "Set Settings To Default*", "Defaults", ControlColor::Wetasphalt, tab,
      []( Control * control, int id ) {
        char ssid[24], password[24], hostname[24];

        if( steerConfig.retainWifiSettings ) {
          memcpy( ssid, steerConfig.ssid, sizeof( ssid ) );
          memcpy( password, steerConfig.password, sizeof( password ) );
          memcpy( hostname, steerConfig.hostname, sizeof( hostname ) );
        }

        steerConfig = steerConfigDefaults;

        if( steerConfig.retainWifiSettings ) {
          memcpy( steerConfig.ssid, ssid, sizeof( ssid ) );
          memcpy( steerConfig.password, password, sizeof( password ) );
          memcpy( steerConfig.hostname, hostname, sizeof( hostname ) );
        }

        setResetButtonToRed();
      } );
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
  static String title;

  if( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance ) {
    title = "QOG Control :: ";
  }

  if( steerConfig.mode == SteerConfig::Mode::AgOpenGps ) {
    title = "AOG Control :: ";
  }

  title += steerConfig.hostname;
  ESPUI.begin( title.c_str() );

  initIdleStats();

  initSensors();
  initRtkCorrection();

  initCan();

  initAutosteer();
}

void loop( void ) {
  dnsServer.processNextRequest();
  vTaskDelay( 100 );
}

