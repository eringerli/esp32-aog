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

#include "main.hpp"

#include "jsonFunctions.hpp"

#include <stdio.h>
#include <string.h>

#include <WiFi.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoOTA.h>
#include <AsyncElegantOTA.h>
#include <DNSServer.h>
#include <ESPUI.h>

// auto display = Adafruit_SSD1306( 128, 32, &Wire );
auto display = Adafruit_SH1107( 64, 128, &Wire, -1, 800000, 400000 );

///////////////////////////////////////////////////////////////////////////
// global data
///////////////////////////////////////////////////////////////////////////
SteerConfig    steerConfig, steerConfigDefaults;
Initialisation initialisation;
SteerCanData   steerCanData = { 0 };

portMUX_TYPE      mux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t i2cMutex;

const byte DNS_PORT = 53;
IPAddress  apIP( 192, 168, 1, 1 );

ControlTreeIterator labelLoad;
ControlTreeIterator labelOrientation;
ControlTreeIterator labelWheelAngle;
ControlTreeIterator buttonReset;
ControlTreeIterator textNmeaToSend;

ControlTreeIterator labelWheelAngleDisplacement;

ControlTreeIterator labelStatusOutput;
ControlTreeIterator labelStatusAdc;
ControlTreeIterator labelStatusCan;
ControlTreeIterator labelStatusImu;
ControlTreeIterator labelStatusInclino;
ControlTreeIterator labelStatusGps;
ControlTreeIterator labelStatusNtrip;
ControlTreeIterator labelStatusRtos;

auto spi = Adafruit_SPIDevice( 32, 10e6, SPI_BITORDER_MSBFIRST, SPI_MODE0, &SPI );

///////////////////////////////////////////////////////////////////////////
// external Libraries
///////////////////////////////////////////////////////////////////////////
ESPUIClass ESPUI( Verbosity::Quiet );
DNSServer  dnsServer;

///////////////////////////////////////////////////////////////////////////
// helper functions
///////////////////////////////////////////////////////////////////////////
void
setResetButtonToRed() {
  buttonReset->color = ControlColor::Alizarin;
  ESPUI.updateControlAsync( buttonReset );
}

void
saveConfigToSPIFFS() {}

void
addGpioOutput( ControlTreeIterator parent ) {
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 GPIO 4",
                    String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio4 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 GPIO 5",
                    String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio5 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 GPIO 12",
                    String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio12 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 GPIO 13 / A12",
                    String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio13 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 GPIO 14",
                    String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio14 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 GPIO 15",
                    String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio15 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 GPIO 16",
                    String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio16 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 GPIO 17",
                    String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio17 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 GPIO 18",
                    String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio18 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 GPIO 19",
                    String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio19 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 GPIO 21",
                    String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio21 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 GPIO 22",
                    String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio22 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 GPIO 23",
                    String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio23 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 GPIO 25",
                    String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio25 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 GPIO 26",
                    String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio26 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 GPIO 27",
                    String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio27 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 GPIO 32 / A7",
                    String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio32 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 GPIO 33 / A9",
                    String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio33 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 GPIO 35",
                    String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio35 ) );
}

void
addGpioInput( ControlTreeIterator parent ) {
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 GPIO 34 / A2 (input only)",
                    String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio34 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 GPIO 39 / A3 (input only)",
                    String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio39 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 GPIO 36 / A4 (input only)",
                    String( ( uint8_t )SteerConfig::Gpio::Esp32Gpio36 ) );
}
void
addAnalogInput( ControlTreeIterator parent ) {
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 A2",
                    String( ( uint8_t )SteerConfig::AnalogIn::Esp32GpioA2 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 A3",
                    String( ( uint8_t )SteerConfig::AnalogIn::Esp32GpioA3 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 A4",
                    String( ( uint8_t )SteerConfig::AnalogIn::Esp32GpioA4 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 A7",
                    String( ( uint8_t )SteerConfig::AnalogIn::Esp32GpioA7 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 A9",
                    String( ( uint8_t )SteerConfig::AnalogIn::Esp32GpioA9 ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ESP32 A12",
                    String( ( uint8_t )SteerConfig::AnalogIn::Esp32GpioA12 ) );
}

void
addAnalogInputADS1115( ControlTreeIterator parent ) {
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ADS1115 A0 Single",
                    String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A0Single ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ADS1115 A1 Single",
                    String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A1Single ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ADS1115 A2 Single",
                    String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A2Single ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ADS1115 A3 Single",
                    String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A3Single ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ADS1115 A0 Differential",
                    String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A0A1Differential ) );
  ESPUI.addControl( parent,
                    ControlType::Option,
                    "ADS1115 A2 Differential",
                    String( ( uint8_t )SteerConfig::AnalogIn::ADS1115A2A3Differential ) );
}

void
addAds131m04Input( ControlTreeIterator parent ) {
  ESPUI.addControl(
    parent, ControlType::Option, "ADS113M04 Input 1", String( ( uint8_t )0 ) );
  ESPUI.addControl(
    parent, ControlType::Option, "ADS113M04 Input 2", String( ( uint8_t )1 ) );
  ESPUI.addControl(
    parent, ControlType::Option, "ADS113M04 Input 3", String( ( uint8_t )2 ) );
  ESPUI.addControl(
    parent, ControlType::Option, "ADS113M04 Input 4", String( ( uint8_t )3 ) );
}

void
addGpioDefault( ControlTreeIterator parentNode ) {
  ESPUI.addControl( parentNode, ControlType::Option, "Board Default", "255" );
}

void
addGpioNone( ControlTreeIterator parentNode ) {
  ESPUI.addControl( parentNode, ControlType::Option, "None", "0" );
}

void
addBaudrateOptions( ControlTreeIterator baudrate ) {
  ESPUI.addControl( baudrate, ControlType::Option, "4800", "4800" );
  ESPUI.addControl( baudrate, ControlType::Option, "9600", "9600" );
  ESPUI.addControl( baudrate, ControlType::Option, "19200", "19200" );
  ESPUI.addControl( baudrate, ControlType::Option, "38400", "38400" );
  ESPUI.addControl( baudrate, ControlType::Option, "57600", "57600" );
  ESPUI.addControl( baudrate, ControlType::Option, "115200", "115200" );
  ESPUI.addControl( baudrate, ControlType::Option, "230400", "230400" );
  ESPUI.addControl( baudrate, ControlType::Option, "460800", "460800" );
  ESPUI.addControl( baudrate, ControlType::Option, "921600", "921600" );
}

void
addNumber( ControlTreeIterator parentNode,
           const char*         label,
           String              value,
           ControlColor        color,
           void ( *callback )( ControlTreeIterator, int ),
           float min,
           float max,
           float step ) {
  ControlTreeIterator num =
    ESPUI.addControl( parentNode, ControlType::Number, label, value, color, callback );
  ESPUI.addControl( num, ControlType::Min, "Min", String( min ) );
  ESPUI.addControl( num, ControlType::Max, "Max", String( max ) );
  ESPUI.addControl( num, ControlType::Step, "Step", String( step ) );
}

void
addSlider( ControlTreeIterator parentNode,
           const char*         label,
           String              value,
           ControlColor        color,
           void ( *callback )( ControlTreeIterator, int ),
           float min,
           float max,
           float step ) {
  ControlTreeIterator num =
    ESPUI.addControl( parentNode, ControlType::Slider, label, value, color, callback );
  ESPUI.addControl( num, ControlType::Min, "Min", String( min ) );
  ESPUI.addControl( num, ControlType::Max, "Max", String( max ) );
  ESPUI.addControl( num, ControlType::Step, "Step", String( step ) );
}

void
notifySensorTask( SensorNotifyBits sensorNotifyBits ) {
  xTaskNotify( sensorWorkerTask, ( uint32_t )( sensorNotifyBits ), eSetBits );
}

void
showDisplay() {
  display.clearDisplay();
  display.setTextSize( 1 );
  display.setTextColor( SH110X_WHITE );
  display.setCursor( 0, 0 );

  if( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance ) {
    display.println( "QtOpenGuidance Mode" );
  }

  if( steerConfig.mode == SteerConfig::Mode::AgOpenGps ) {
    display.println( "AgOpenGPS Mode" );
  }

  display.print( "STA: " );
  display.println( steerConfig.ssidAp );
  display.print( "pw:  " );
  display.println( steerConfig.passwordAp );
  display.print( "IP:  " );
  display.println( WiFi.localIP().toString() );

  display.print( "AP:  " );
  display.println( steerConfig.ssidSta );
  display.print( "pw:  " );
  display.println( steerConfig.passwordSta );
  display.print( "IP:  " );
  display.println( WiFi.softAPIP().toString() );
  display.display();
}

///////////////////////////////////////////////////////////////////////////
// Application
///////////////////////////////////////////////////////////////////////////
void
setup( void ) {
  if( !SPIFFS.begin( true ) ) {
    Serial.println( "SPIFFS Mount Failed" );
    return;
  }
  loadSavedConfig();

  Serial.begin( steerConfig.baudrate );

  Wire.begin(
    ( int )steerConfig.gpioSDA, ( int )steerConfig.gpioSCL, steerConfig.i2cBusSpeed );

  WiFi.disconnect( true );

  WiFi.onEvent( []( WiFiEvent_t event ) { showDisplay(); } );

  {
    int sck = steerConfig.gpioSck == SteerConfig::Gpio::Default ?
                SCK :
                ( int )steerConfig.gpioSck;

    int miso = steerConfig.gpioMiso == SteerConfig::Gpio::Default ?
                 MISO :
                 ( int )steerConfig.gpioMiso;

    int mosi = steerConfig.gpioMosi == SteerConfig::Gpio::Default ?
                 MOSI :
                 ( int )steerConfig.gpioMosi;

    SPI.begin( sck, miso, mosi );
    pinMode( sck, OUTPUT );
    pinMode( miso, INPUT_PULLUP );
    pinMode( mosi, OUTPUT );
  }

  Serial.print( "display.begin: " );

  auto dis = display.begin( 0x3C, false );
  Serial.println( dis );
  //   display.
  display.clearDisplay();
  display.setRotation( 1 );
  display.display();

  Serial.updateBaudRate( steerConfig.baudrate );

  if( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance ) {
    Serial.println(
      "Welcome to esp32-aog.\nThe selected mode is QtOpenGuidance.\nTo configure, please open the webui." );
  }

  if( steerConfig.mode == SteerConfig::Mode::AgOpenGps ) {
    Serial.println(
      "Welcome to esp32-aog.\nThe selected mode is AgOpenGps.\nTo configure, please open the webui." );
  }

  display.display(); // actually display all of the above

  if( steerConfig.apModePin != SteerConfig::Gpio::None ) {
    pinMode( ( int )steerConfig.apModePin, OUTPUT );
    digitalWrite( ( int )steerConfig.apModePin, LOW );
  }

  WiFi.setHostname( steerConfig.hostname );

  // try to connect to existing network
  WiFi.mode( WIFI_MODE_APSTA );
  WiFi.softAPConfig( apIP, apIP, IPAddress( 255, 255, 255, 0 ) );
  WiFi.softAP( steerConfig.ssidSta, steerConfig.passwordSta );

  WiFi.begin( steerConfig.ssidAp, steerConfig.passwordAp );

  Serial.print( "\n\nTry to connect to existing network \"" );
  Serial.print( steerConfig.ssidAp );
  Serial.print( "\" with password \"" );
  Serial.print( steerConfig.passwordAp );
  Serial.print( "\"" );

  {
    uint8_t timeout = 5;

    // Wait for connection, 2.5s timeout
    do {
      delay( 500 );
      Serial.print( "." );
      timeout--;
    } while( timeout && WiFi.status() != WL_CONNECTED );

    if( steerConfig.apModePin != SteerConfig::Gpio::None ) {
      digitalWrite( ( int )steerConfig.apModePin, WiFi.status() == WL_CONNECTED );
    }
  }

  showDisplay();

  dnsServer.start( DNS_PORT, "*", apIP );

  ArduinoOTA.begin();

  Serial.println( "\n\nWiFi parameters:" );
  Serial.print( "Mode: " );
  Serial.println( WiFi.getMode() == WIFI_AP ? "Station" : "Client" );
  Serial.print( "IP address: " );
  Serial.println( WiFi.getMode() == WIFI_AP ? WiFi.softAPIP() : WiFi.localIP() );

  labelLoad =
    ESPUI.addControl( ControlType::Label, "Load:", "", ControlColor::Turquoise );
  labelOrientation =
    ESPUI.addControl( ControlType::Label, "Orientation:", "", ControlColor::Emerald );
  labelWheelAngle =
    ESPUI.addControl( ControlType::Label, "Wheel Angle:", "0°", ControlColor::Emerald );
  // graphWheelAngle = ESPUI.addControl( ControlType::Graph, "Wheel Angle:", "",
  // ControlColor::Emerald );

  buttonReset = ESPUI.addControl( ControlType::Button,
                                  "Store the Settings",
                                  "Apply",
                                  ControlColor::Emerald,
                                  []( ControlTreeIterator control, int id ) {
                                    if( id == B_UP ) {
                                      saveConfig();
                                    }
                                  } );

  buttonReset = ESPUI.addControl( ControlType::Button,
                                  "If this turns red, you have to",
                                  "Apply & Reboot",
                                  ControlColor::Emerald,
                                  []( ControlTreeIterator control, int id ) {
                                    if( id == B_UP ) {
                                      saveConfig();
                                      SPIFFS.end();
                                      ESP.restart();
                                    }
                                  } );

  ControlTreeIterator tabConfigurations;

  // Status Tab
  {
    ControlTreeIterator tab = ESPUI.addControl( ControlType::Tab, "Status", "Status" );

    labelStatusOutput = ESPUI.addControl( tab,
                                          ControlType::Label,
                                          "Output:",
                                          "No Output configured",
                                          ControlColor::Turquoise );
    labelStatusAdc    = ESPUI.addControl(
      tab, ControlType::Label, "ADC:", "No ADC configured", ControlColor::Turquoise );
    labelStatusCan = ESPUI.addControl(
      tab, ControlType::Label, "CAN:", "No CAN BUS configured", ControlColor::Turquoise );
    labelStatusImu = ESPUI.addControl(
      tab, ControlType::Label, "IMU:", "No IMU configured", ControlColor::Turquoise );

    if( steerConfig.mode == SteerConfig::Mode::AgOpenGps ) {
      labelStatusInclino = ESPUI.addControl( tab,
                                             ControlType::Label,
                                             "Inclinometer:",
                                             "No Inclinometer configured",
                                             ControlColor::Turquoise );
    }

    labelStatusGps = ESPUI.addControl(
      tab, ControlType::Label, "GPS:", "Not configured", ControlColor::Turquoise );
    labelStatusNtrip = ESPUI.addControl(
      tab, ControlType::Label, "NTRIP:", "Not configured", ControlColor::Turquoise );
    labelStatusRtos = ESPUI.addControl(
      tab, ControlType::Label, "RTOS:", "Not configured", ControlColor::Turquoise );
  }

  // Info Tab
  {
    ControlTreeIterator tab =
      ESPUI.addControl( ControlType::Tab, "Info/Help", "Info/Help" );
    ESPUI.addControl(
      tab,
      ControlType::Label,
      "Basics:",
      "This works with setting up the different options in the panels. If an option requires a reboot (indicated by the darker blue and an asterisk after the title), press on the button \"Apply & Reboot\" and refresh the page after some time, usually 5-10 seconds. Settings with the lighter shade of blue are applied immediately, but are not saved to the permanent memory. You can do this with the \"Apply\" button. If the values are complete garbage or you want a fresh start, set the config to defaults in the \"Configurations\" tab.",
      ControlColor::Carrot );

    ESPUI.addControl(
      tab,
      ControlType::Label,
      "Network:",
      "Here the network is configured. Leave it on defaults, only if used as roof controller (only GPS + IMU), set \"Port to send from\" to 5544.",
      ControlColor::Turquoise );
    ESPUI.addControl(
      tab,
      ControlType::Label,
      "CAN Bus/J1939:",
      "Enable if used. To use data from the vehicle bus as workswitch, configure it in the next tab.",
      ControlColor::Turquoise );
    ESPUI.addControl(
      tab,
      ControlType::Label,
      "Work- and Steerswitch:",
      "If work- and steerswitches as physical inputs are used, enable them by configuring a GPIO. If you want to use the CAN-bus (J1939), set the type to a hitch position or RPM with a threshold.",
      ControlColor::Turquoise );
    ESPUI.addControl(
      tab,
      ControlType::Label,
      "Wheel Angle Sensor:",
      "To enable the wheel angle sensor, configure the input first. If you use two arms connected to the tie rod, measure them exactly and configure the values. This is to calculate out the unlinearities.",
      ControlColor::Turquoise );
    ESPUI.addControl( tab,
                      ControlType::Label,
                      "Steering:",
                      "Set up the type and the GPIOs",
                      ControlColor::Turquoise );
    ESPUI.addControl(
      tab,
      ControlType::Label,
      "Steering PID:",
      "This controller uses its own PID-controller. No values are taken over from AOG, so everything is entered here.",
      ControlColor::Turquoise );
    ESPUI.addControl(
      tab,
      ControlType::Label,
      "Sensors:",
      "Here the IMU and inclinometer are set up. The Mounting Correction is entered as three angles relative to the tractor axis, so the IMU can be mounted in every position, as long as the chips are positioned relative to each other with no difference (normaly, the manufacturer of the sensor pcb does this anyway). The FXAS2100/FXOS8700-combo is recomned, as they give the most precise roll/pitch/heading with the least amount of calibration.",
      ControlColor::Turquoise );
    ESPUI.addControl(
      tab,
      ControlType::Label,
      "NTRIP/GPS:",
      "Here the connection to the GPS is set up, also the NTRIP-client. Usualy, you want to send the data to AOG via UDP, a serial connection via USB is also possible. The TCP-Socket enables 3rd-party GPS-Software and configuring the GPS-Receiver with u-center.",
      ControlColor::Turquoise );
  }

  // Network Tab
  {
    ControlTreeIterator tab = ESPUI.addControl( ControlType::Tab, "Network", "Network" );

    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "Mode*",
                          String( ( int )steerConfig.mode ),
                          ControlColor::Wetasphalt,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.mode =
                              ( SteerConfig::Mode )control->value.toInt();
                            setResetButtonToRed();
                          } );
      ESPUI.addControl(
        sel, ControlType::Option, "QtOpenGuidance", "0", ControlColor::Alizarin );
      ESPUI.addControl(
        sel, ControlType::Option, "AgOpenGps", "1", ControlColor::Alizarin );
    }

    {
      ControlTreeIterator baudrate =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "Baudrate Serial",
                          String( steerConfig.baudrate ),
                          ControlColor::Peterriver,
                          []( ControlTreeIterator control, int id ) {
                            uint32_t baudrate    = control->value.toInt();
                            steerConfig.baudrate = baudrate;
                            Serial.updateBaudRate( baudrate );
                          } );
      addBaudrateOptions( baudrate );
    }

    ESPUI.addControl( tab,
                      ControlType::Text,
                      "Hostname*",
                      String( steerConfig.hostname ),
                      ControlColor::Wetasphalt,
                      []( ControlTreeIterator control, int id ) {
                        control->value.toCharArray( steerConfig.hostname,
                                                    sizeof( steerConfig.hostname ) );
                        setResetButtonToRed();
                      } );
    ESPUI.addControl( tab,
                      ControlType::Text,
                      "SSID of existing network*",
                      String( steerConfig.ssidAp ),
                      ControlColor::Wetasphalt,
                      []( ControlTreeIterator control, int id ) {
                        control->value.toCharArray( steerConfig.ssidAp,
                                                    sizeof( steerConfig.ssidAp ) );
                        setResetButtonToRed();
                      } );
    ESPUI.addControl( tab,
                      ControlType::Text,
                      "Password of existing network*",
                      String( steerConfig.passwordAp ),
                      ControlColor::Wetasphalt,
                      []( ControlTreeIterator control, int id ) {
                        control->value.toCharArray( steerConfig.passwordAp,
                                                    sizeof( steerConfig.passwordAp ) );
                        setResetButtonToRed();
                      } );
    ESPUI.addControl( tab,
                      ControlType::Text,
                      "SSID for hotspot mode*",
                      String( steerConfig.ssidSta ),
                      ControlColor::Wetasphalt,
                      []( ControlTreeIterator control, int id ) {
                        control->value.toCharArray( steerConfig.ssidSta,
                                                    sizeof( steerConfig.ssidSta ) );
                        setResetButtonToRed();
                      } );
    ESPUI.addControl( tab,
                      ControlType::Text,
                      "Password for hotspot mode*",
                      String( steerConfig.passwordSta ),
                      ControlColor::Wetasphalt,
                      []( ControlTreeIterator control, int id ) {
                        control->value.toCharArray( steerConfig.passwordSta,
                                                    sizeof( steerConfig.passwordSta ) );
                        setResetButtonToRed();
                      } );

    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "Pin to show AP mode*",
                          String( ( int )steerConfig.apModePin ),
                          ControlColor::Wetasphalt,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.apModePin =
                              ( SteerConfig::Gpio )control->value.toInt();
                            setResetButtonToRed();
                          } );
      addGpioNone( sel );
      addGpioOutput( sel );
    }

    ESPUI.addControl( tab,
                      ControlType::Switcher,
                      "OTA Enabled*",
                      steerConfig.enableOTA ? "1" : "0",
                      ControlColor::Wetasphalt,
                      []( ControlTreeIterator control, int id ) {
                        steerConfig.enableOTA = control->value.toInt() == 1;
                        setResetButtonToRed();
                      } );

    if( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance ) {
      addNumber(
        tab,
        "Port to send to*",
        String( steerConfig.qogPortSendTo ),
        ControlColor::Wetasphalt,
        []( ControlTreeIterator control, int id ) {
          steerConfig.qogPortSendTo = control->value.toInt();
          setResetButtonToRed();
        },
        1,
        ULONG_MAX,
        1 );
      addNumber(
        tab,
        "Port to listen to*",
        String( steerConfig.qogPortListenTo ),
        ControlColor::Wetasphalt,
        []( ControlTreeIterator control, int id ) {
          steerConfig.qogPortListenTo = control->value.toInt();
          setResetButtonToRed();
        },
        1,
        ULONG_MAX,
        1 );
    }

    if( steerConfig.mode == SteerConfig::Mode::AgOpenGps ) {
      addNumber(
        tab,
        "Port to send from*",
        String( steerConfig.aogPortSendFrom ),
        ControlColor::Wetasphalt,
        []( ControlTreeIterator control, int id ) {
          steerConfig.aogPortSendFrom = control->value.toInt();
          setResetButtonToRed();
        },
        1,
        ULONG_MAX,
        1 );

      addNumber(
        tab,
        "Port to send to*",
        String( steerConfig.aogPortSendTo ),
        ControlColor::Wetasphalt,
        []( ControlTreeIterator control, int id ) {
          steerConfig.aogPortSendTo = control->value.toInt();
          setResetButtonToRed();
        },
        1,
        ULONG_MAX,
        1 );
      addNumber(
        tab,
        "Port to listen to*",
        String( steerConfig.aogPortListenTo ),
        ControlColor::Wetasphalt,
        []( ControlTreeIterator control, int id ) {
          steerConfig.aogPortListenTo = control->value.toInt();
          setResetButtonToRed();
        },
        1,
        ULONG_MAX,
        1 );
    }
  }

  // I2C Tab
  {
    ControlTreeIterator tab = ESPUI.addControl( ControlType::Tab, "I2C", "I2C" );

    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "I2C SDA Gpio*",
                          String( ( int )steerConfig.gpioSDA ),
                          ControlColor::Wetasphalt,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.gpioSDA =
                              ( SteerConfig::Gpio )control->value.toInt();
                            setResetButtonToRed();
                          } );
      addGpioDefault( sel );
      addGpioOutput( sel );
    }
    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "I2C SCL Gpio*",
                          String( ( int )steerConfig.gpioSCL ),
                          ControlColor::Wetasphalt,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.gpioSCL =
                              ( SteerConfig::Gpio )control->value.toInt();
                            setResetButtonToRed();
                          } );
      addGpioDefault( sel );
      addGpioOutput( sel );
    }

    addNumber(
      tab,
      "I2C Bus Speed*",
      String( steerConfig.i2cBusSpeed ),
      ControlColor::Wetasphalt,
      []( ControlTreeIterator control, int id ) {
        steerConfig.i2cBusSpeed = control->value.toInt();
        setResetButtonToRed();
      },
      10000,
      5000000,
      1000 );
  }

  // SPI Tab
  {
    ControlTreeIterator tab = ESPUI.addControl( ControlType::Tab, "SPI", "SPI" );

    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "SPI SCK Gpio*",
                          String( ( int )steerConfig.gpioSck ),
                          ControlColor::Wetasphalt,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.gpioSck =
                              ( SteerConfig::Gpio )control->value.toInt();
                            setResetButtonToRed();
                          } );
      addGpioDefault( sel );
      addGpioOutput( sel );
    }
    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "SPI MISO Gpio*",
                          String( ( int )steerConfig.gpioMiso ),
                          ControlColor::Wetasphalt,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.gpioMiso =
                              ( SteerConfig::Gpio )control->value.toInt();
                            setResetButtonToRed();
                          } );
      addGpioDefault( sel );
      addGpioOutput( sel );
    }
    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "SPI MOSI Gpio*",
                          String( ( int )steerConfig.gpioMosi ),
                          ControlColor::Wetasphalt,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.gpioMosi =
                              ( SteerConfig::Gpio )control->value.toInt();
                            setResetButtonToRed();
                          } );
      addGpioDefault( sel );
      addGpioOutput( sel );
    }

    addNumber(
      tab,
      "SPI Bus Speed*",
      String( steerConfig.spiBusSpeed ),
      ControlColor::Wetasphalt,
      []( ControlTreeIterator control, int id ) {
        steerConfig.spiBusSpeed = control->value.toInt();
        setResetButtonToRed();
      },
      10000,
      25000000,
      1000 );
  }

  // CAN Bus
  /* if( false ) */ {
    ControlTreeIterator tab =
      ESPUI.addControl( ControlType::Tab, "CAN Bus/J1939", "CAN Bus/J1939" );

    ESPUI.addControl( tab,
                      ControlType::Switcher,
                      "CAN Bus Enabled*",
                      steerConfig.canBusEnabled ? "1" : "0",
                      ControlColor::Wetasphalt,
                      []( ControlTreeIterator control, int id ) {
                        steerConfig.canBusEnabled = control->value.toInt() == 1;
                        setResetButtonToRed();
                      } );

    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "CAN Bus RX Gpio*",
                          String( ( int )steerConfig.canBusRx ),
                          ControlColor::Wetasphalt,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.canBusRx =
                              ( SteerConfig::Gpio )control->value.toInt();
                            setResetButtonToRed();
                          } );
      addGpioNone( sel );
      addGpioOutput( sel );
    }
    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "CAN Bus TX Gpio*",
                          String( ( int )steerConfig.canBusTx ),
                          ControlColor::Wetasphalt,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.canBusTx =
                              ( SteerConfig::Gpio )control->value.toInt();
                            setResetButtonToRed();
                          } );
      addGpioNone( sel );
      addGpioOutput( sel );
    }

    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "CAN Bus Speed*",
                          String( ( int )steerConfig.canBusSpeed ),
                          ControlColor::Wetasphalt,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.canBusSpeed =
                              ( SteerConfig::CanBusSpeed )control->value.toInt();
                            setResetButtonToRed();
                          } );
      ESPUI.addControl(
        sel, ControlType::Option, "250kB/s", "250", ControlColor::Alizarin );
      ESPUI.addControl(
        sel, ControlType::Option, "500kB/s", "500", ControlColor::Alizarin );
    }
  }

  // CS/DRDY Tab
  {
    ControlTreeIterator tab = ESPUI.addControl( ControlType::Tab, "CS/DRDY", "CS/DRDY" );

    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "ADS131M04 CS Gpio*",
                          String( ( int )steerConfig.gpioAds131m04Cs ),
                          ControlColor::Wetasphalt,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.gpioAds131m04Cs =
                              ( SteerConfig::Gpio )control->value.toInt();
                            setResetButtonToRed();
                          } );
      addGpioInput( sel );
      addGpioOutput( sel );
    }
    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "ADS131M04 Drdy Gpio*",
                          String( ( int )steerConfig.gpioAds131m04Drdy ),
                          ControlColor::Wetasphalt,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.gpioAds131m04Drdy =
                              ( SteerConfig::Gpio )control->value.toInt();
                            setResetButtonToRed();
                          } );
      addGpioInput( sel );
      addGpioOutput( sel );
    }

    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "Acc/Gyro CS Gpio*",
                          String( ( int )steerConfig.gpioAccGyroCs ),
                          ControlColor::Wetasphalt,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.gpioAccGyroCs =
                              ( SteerConfig::Gpio )control->value.toInt();
                            setResetButtonToRed();
                          } );
      addGpioInput( sel );
      addGpioOutput( sel );
    }
    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "Acc/Gyro Drdy Gpio*",
                          String( ( int )steerConfig.gpioAccGyroDrdy ),
                          ControlColor::Wetasphalt,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.gpioAccGyroDrdy =
                              ( SteerConfig::Gpio )control->value.toInt();
                            setResetButtonToRed();
                          } );
      addGpioInput( sel );
      addGpioOutput( sel );
    }

    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "Mag CS Gpio*",
                          String( ( int )steerConfig.gpioMagCs ),
                          ControlColor::Wetasphalt,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.gpioMagCs =
                              ( SteerConfig::Gpio )control->value.toInt();
                            setResetButtonToRed();
                          } );
      addGpioInput( sel );
      addGpioOutput( sel );
    }
    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "Mag Drdy Gpio*",
                          String( ( int )steerConfig.gpioMagDrdy ),
                          ControlColor::Wetasphalt,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.gpioMagDrdy =
                              ( SteerConfig::Gpio )control->value.toInt();
                            setResetButtonToRed();
                          } );
      addGpioInput( sel );
      addGpioOutput( sel );
    }
  }

  // Filter Tab
  {
    ControlTreeIterator tab = ESPUI.addControl( ControlType::Tab, "Filters", "Filters" );

    ESPUI.addControl(
      tab,
      ControlType::Label,
      "Filters:",
      "The ADC Filters are second order butterworth filters with configurable cut-off frequencies.",
      ControlColor::Turquoise );

    {
      ControlTreeIterator sel = ESPUI.addControl(
        tab,
        ControlType::Select,
        "ADS131M04 Sample Rate",
        String( ( uint8_t )steerConfig.ads131m04SampleRate ),
        ControlColor::Peterriver,
        []( ControlTreeIterator control, int id ) {
          steerConfig.ads131m04SampleRate =
            ( SteerConfig::Ads131m04SampleRate )control->value.toInt();
          notifySensorTask( SensorNotifyBits::RefreshFiltersSamplerate );
        } );
      ESPUI.addControl( sel,
                        ControlType::Option,
                        "250Sps",
                        String( ( int )SteerConfig::Ads131m04SampleRate::Rate250Sps ) );
      ESPUI.addControl( sel,
                        ControlType::Option,
                        "500Sps",
                        String( ( int )SteerConfig::Ads131m04SampleRate::Rate500Sps ) );
      ESPUI.addControl( sel,
                        ControlType::Option,
                        "1kSps",
                        String( ( int )SteerConfig::Ads131m04SampleRate::Rate1kSps ) );
      ESPUI.addControl( sel,
                        ControlType::Option,
                        "2kSps",
                        String( ( int )SteerConfig::Ads131m04SampleRate::Rate2kSps ) );
      ESPUI.addControl( sel,
                        ControlType::Option,
                        "4kSps",
                        String( ( int )SteerConfig::Ads131m04SampleRate::Rate4kSps ) );
      ESPUI.addControl( sel,
                        ControlType::Option,
                        "8kSps",
                        String( ( int )SteerConfig::Ads131m04SampleRate::Rate8kSps ) );
      ESPUI.addControl( sel,
                        ControlType::Option,
                        "16kSps",
                        String( ( int )SteerConfig::Ads131m04SampleRate::Rate16kSps ) );
    }

    addNumber(
      tab,
      "ADC Input 1 cut-off frequency",
      String( steerConfig.filterCutOffFrequencies[0] ),
      ControlColor::Peterriver,
      []( ControlTreeIterator control, int id ) {
        steerConfig.filterCutOffFrequencies[0] = control->value.toInt();
        notifySensorTask( SensorNotifyBits::RefreshFiltersSamplerate );
      },
      0.1,
      32000,
      0.1 );

    addNumber(
      tab,
      "ADC Input 2 cut-off frequency",
      String( steerConfig.filterCutOffFrequencies[1] ),
      ControlColor::Peterriver,
      []( ControlTreeIterator control, int id ) {
        steerConfig.filterCutOffFrequencies[1] = control->value.toInt();
        notifySensorTask( SensorNotifyBits::RefreshFiltersSamplerate );
      },
      0.1,
      32000,
      0.1 );

    addNumber(
      tab,
      "ADC Input 3 cut-off frequency",
      String( steerConfig.filterCutOffFrequencies[2] ),
      ControlColor::Peterriver,
      []( ControlTreeIterator control, int id ) {
        steerConfig.filterCutOffFrequencies[2] = control->value.toInt();
        notifySensorTask( SensorNotifyBits::RefreshFiltersSamplerate );
      },
      0.1,
      32000,
      0.1 );

    addNumber(
      tab,
      "ADC Input 4 cut-off frequency",
      String( steerConfig.filterCutOffFrequencies[3] ),
      ControlColor::Peterriver,
      []( ControlTreeIterator control, int id ) {
        steerConfig.filterCutOffFrequencies[3] = control->value.toInt();
        notifySensorTask( SensorNotifyBits::RefreshFiltersSamplerate );
      },
      0.1,
      32000,
      0.1 );
  }

  // Switches/Buttons Tab
  if( false ) {
    ControlTreeIterator tab = ESPUI.addControl(
      ControlType::Tab, "Work- and Steerswitch", "Work- and Steerswitch" );

    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "Workswitch Type",
                          String( ( int )steerConfig.workswitchType ),
                          ControlColor::Peterriver,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.workswitchType =
                              ( SteerConfig::WorkswitchType )control->value.toInt();
                          } );
      ESPUI.addControl( sel, ControlType::Option, "None", "0", ControlColor::Alizarin );
      ESPUI.addControl( sel, ControlType::Option, "Gpio", "1", ControlColor::Alizarin );

      if( steerConfig.canBusEnabled ) {
        ESPUI.addControl( sel,
                          ControlType::Option,
                          "Rear Hitch Position (from Can Bus)",
                          "2",
                          ControlColor::Alizarin );
        ESPUI.addControl( sel,
                          ControlType::Option,
                          "Front Hitch Position (from Can Bus)",
                          "3",
                          ControlColor::Alizarin );
        ESPUI.addControl( sel,
                          ControlType::Option,
                          "Rear Pto Rpm (from Can Bus)",
                          "4",
                          ControlColor::Alizarin );
        ESPUI.addControl( sel,
                          ControlType::Option,
                          "Front Pto Rpm (from Can Bus)",
                          "5",
                          ControlColor::Alizarin );
        ESPUI.addControl( sel,
                          ControlType::Option,
                          "Motor Rpm (from Can Bus)",
                          "6",
                          ControlColor::Alizarin );
      }
    }

    if( steerConfig.canBusEnabled ) {
      addNumber(
        tab,
        "Hitch Threshold",
        String( steerConfig.canBusHitchThreshold ),
        ControlColor::Peterriver,
        []( ControlTreeIterator control, int id ) {
          steerConfig.canBusHitchThreshold = control->value.toInt();
        },
        0,
        100,
        1 );
      {
        addNumber(
          tab,
          "Hitch Threshold Hysteresis",
          String( steerConfig.canBusHitchThresholdHysteresis ),
          ControlColor::Peterriver,
          []( ControlTreeIterator control, int id ) {
            steerConfig.canBusHitchThresholdHysteresis = control->value.toInt();
          },
          0,
          100,
          1 );
      }

      {
        addNumber(
          tab,
          "RPM Threshold",
          String( steerConfig.canBusHitchThreshold ),
          ControlColor::Peterriver,
          []( ControlTreeIterator control, int id ) {
            steerConfig.canBusHitchThreshold = control->value.toInt();
          },
          0,
          3500,
          1 );
      }
      {
        addNumber(
          tab,
          "RPM Threshold Hysteresis",
          String( steerConfig.canBusHitchThresholdHysteresis ),
          ControlColor::Peterriver,
          []( ControlTreeIterator control, int id ) {
            steerConfig.canBusHitchThresholdHysteresis = control->value.toInt();
          },
          0,
          1000,
          1 );
      }
    }

    // {
    // ControlTreeIterator sel = ESPUI.addControl( tab,ControlType::Select,
    // "Workswitch LED*", String( ( int )steerConfig.gpioWorkswitch ),
    // ControlColor::Wetasphalt, tab,
    // []( Control * control, int id ) {
    // steerConfig.gpioWorkswitch = ( SteerConfig::Gpio )control->value.toInt();
    // setResetButtonToRed();
    // } );
    // ESPUI.addControl( ControlType::Option, "None", "0",
    // ControlColor::Alizarin); addGpioOutput( );
    // }

    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "Workswitch Gpio*",
                          String( ( int )steerConfig.gpioWorkswitch ),
                          ControlColor::Wetasphalt,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.gpioWorkswitch =
                              ( SteerConfig::Gpio )control->value.toInt();
                            setResetButtonToRed();
                          } );
      addGpioNone( sel );
      addGpioInput( sel );
      addGpioOutput( sel );
    }
    {
      ESPUI.addControl( tab,
                        ControlType::Switcher,
                        "Workswitch Active Low",
                        steerConfig.workswitchActiveLow ? "1" : "0",
                        ControlColor::Peterriver,
                        []( ControlTreeIterator control, int id ) {
                          steerConfig.workswitchActiveLow = control->value.toInt() == 1;
                        } );
    }

    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "Autosteer switch/button Gpio*",
                          String( ( int )steerConfig.gpioSteerswitch ),
                          ControlColor::Wetasphalt,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.gpioSteerswitch =
                              ( SteerConfig::Gpio )control->value.toInt();
                            setResetButtonToRed();
                          } );
      addGpioNone( sel );
      addGpioInput( sel );
      addGpioOutput( sel );
    }

    {
      addNumber(
        tab,
        "Auto recognise Autosteer GPIO as Switch [ms]",
        String( steerConfig.autoRecogniseSteerGpioAsSwitchOrButton ),
        ControlColor::Peterriver,
        []( ControlTreeIterator control, int id ) {
          steerConfig.autoRecogniseSteerGpioAsSwitchOrButton = control->value.toInt();
        },
        0,
        16000,
        100 );
    }

    {
      ESPUI.addControl( tab,
                        ControlType::Switcher,
                        "Steerswitch Active Low",
                        steerConfig.steerswitchActiveLow ? "1" : "0",
                        ControlColor::Peterriver,
                        []( ControlTreeIterator control, int id ) {
                          steerConfig.steerswitchActiveLow = control->value.toInt() == 1;
                        } );
    }

    // {
    // ControlTreeIterator sel = ESPUI.addControl( tab,ControlType::Select,
    // "Autosteer LED*", String( ( int )steerConfig.gpioSteerswitch ),
    // ControlColor::Wetasphalt, tab,
    // []( Control * control, int id ) {
    // steerConfig.gpioSteerswitch = ( SteerConfig::Gpio
    // )control->value.toInt(); setResetButtonToRed(); } ); ESPUI.addControl(
    // ControlType::Option, "None", "0", ControlColor::Alizarin);
    // addGpioOutput( );
    // }
  }

  // Wheel Angle Sensor Tab
  {
    ControlTreeIterator tab =
      ESPUI.addControl( ControlType::Tab, "Wheel Angle Sensor", "Wheel Angle Sensor" );

    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "Wheel Angle Sensor Input*",
                          String( steerConfig.wheelAngleInput ),
                          ControlColor::Wetasphalt,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.wheelAngleInput = control->value.toInt();
                            setResetButtonToRed();
                          } );
      addAds131m04Input( sel );
    }

    {
      addNumber(
        tab,
        "Wheel Angle Sensor Center",
        String( steerConfig.wheelAnglePositionZero ),
        ControlColor::Peterriver,
        []( ControlTreeIterator control, int id ) {
          steerConfig.wheelAnglePositionZero = control->value.toInt();
        },
        0,
        20000000,
        1 );
    }

    {
      addNumber(
        tab,
        "Wheel Angle Counts per Degree",
        String( steerConfig.wheelAngleCountsPerDegree ),
        ControlColor::Peterriver,
        []( ControlTreeIterator control, int id ) {
          steerConfig.wheelAngleCountsPerDegree = control->value.toFloat();
        },
        0,
        500000,
        1 );
    }

    ESPUI.addControl( tab,
                      ControlType::Switcher,
                      "Invert Wheel Angle Sensor",
                      steerConfig.invertWheelAngleSensor ? "1" : "0",
                      ControlColor::Peterriver,
                      []( ControlTreeIterator control, int id ) {
                        steerConfig.invertWheelAngleSensor = control->value.toInt() == 1;
                      } );

    addSlider(
      tab,
      "Wheel Angle Offset",
      String( steerConfig.wheelAngleOffset ),
      ControlColor::Peterriver,
      []( ControlTreeIterator control, int id ) {
        steerConfig.wheelAngleOffset = control->value.toFloat();
      },
      -40,
      40,
      0.1 );
  }

  // Steering Tab
  {
    ControlTreeIterator tab =
      ESPUI.addControl( ControlType::Tab, "Steering", "Steering" );

    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "Output Type*",
                          String( ( int )steerConfig.outputType ),
                          ControlColor::Wetasphalt,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.outputType =
                              ( SteerConfig::OutputType )control->value.toInt();
                            setResetButtonToRed();
                          } );
      ESPUI.addControl( sel, ControlType::Option, "None", "0", ControlColor::Alizarin );
      ESPUI.addControl(
        sel, ControlType::Option, "Motor: Cytron MD30C", "1", ControlColor::Alizarin );
      ESPUI.addControl(
        sel, ControlType::Option, "Motor: IBT 2", "2", ControlColor::Alizarin );
      ESPUI.addControl( sel,
                        ControlType::Option,
                        "Hydraulic: IBT 2 + PWM 2-Coil Valve",
                        "3",
                        ControlColor::Alizarin );
      ESPUI.addControl( sel,
                        ControlType::Option,
                        "Hydraulic: IBT 2 + Danfoss Valve PVE A/H/M",
                        "4",
                        ControlColor::Alizarin );
    }

    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "Output Pin for PWM (or right coil)*",
                          String( ( int )steerConfig.gpioPwm ),
                          ControlColor::Wetasphalt,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.gpioPwm =
                              ( SteerConfig::Gpio )control->value.toInt();
                            setResetButtonToRed();
                          } );
      addGpioNone( sel );
      addGpioOutput( sel );
    }

    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "Output Pin for Dir (or left coil)*",
                          String( ( int )steerConfig.gpioDir ),
                          ControlColor::Wetasphalt,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.gpioDir =
                              ( SteerConfig::Gpio )control->value.toInt();
                            setResetButtonToRed();
                          } );
      addGpioNone( sel );
      addGpioOutput( sel );
    }

    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "Output Pin for Enable*",
                          String( ( int )steerConfig.gpioEn ),
                          ControlColor::Wetasphalt,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.gpioEn =
                              ( SteerConfig::Gpio )control->value.toInt();
                            setResetButtonToRed();
                          } );
      addGpioNone( sel );
      addGpioOutput( sel );
    }

    ESPUI.addControl( tab,
                      ControlType::Switcher,
                      "Invert Output",
                      steerConfig.invertOutput ? "1" : "0",
                      ControlColor::Peterriver,
                      []( ControlTreeIterator control, int id ) {
                        steerConfig.invertOutput = control->value.toInt() == 1;
                      } );

    // {
    // uint16_t num = ESPUI.addControl( ControlType::Slider, "PWM Frequency",
    // String( steerConfig.pwmFrequency ), ControlColor::Peterriver, tab,
    // []( Control * control, int id ) {
    // steerConfig.pwmFrequency = control->value.toInt();
    //
    // switch ( initialisation.outputType ) {
    // case SteerConfig::OutputType::SteeringMotorIBT2: {
    // ledcWriteTone( 0, steerConfig.pwmFrequency );
    // ledcWriteTone( 1, steerConfig.pwmFrequency );
    // }
    // break;
    // }
    // } );
    // ESPUI.addControl( ControlType::Min, "Min", "1000",
    // ControlColor::Peterriver, num ); ESPUI.addControl( ControlType::Max,
    // "Max", "5000", ControlColor::Peterriver, num ); ESPUI.addControl(
    // ControlType::Step, "Step", "100", ControlColor::Peterriver, num );
    // }
  }

  // Steering PID Tab
  {
    ControlTreeIterator tab =
      ESPUI.addControl( ControlType::Tab, "Steering PID", "Steering PID" );

    {
      addNumber(
        tab,
        "PID Kp",
        String( steerConfig.steeringPidKp, 4 ),
        ControlColor::Peterriver,
        []( ControlTreeIterator control, int id ) {
          steerConfig.steeringPidKp = control->value.toDouble();
          notifySensorTask( SensorNotifyBits::RefreshPidValues );
        },
        0,
        50,
        0.1 );
    }
    {
      addNumber(
        tab,
        "PID Ki",
        String( steerConfig.steeringPidKi, 4 ),
        ControlColor::Peterriver,
        []( ControlTreeIterator control, int id ) {
          steerConfig.steeringPidKi = control->value.toDouble();
          notifySensorTask( SensorNotifyBits::RefreshPidValues );
        },
        0,
        50,
        0.01 );
    }
    {
      addNumber(
        tab,
        "PID Kd",
        String( steerConfig.steeringPidKd, 4 ),
        ControlColor::Peterriver,
        []( ControlTreeIterator control, int id ) {
          steerConfig.steeringPidKd = control->value.toDouble();
          notifySensorTask( SensorNotifyBits::RefreshPidValues );
        },
        0,
        50,
        0.01 );
    }
    {
      addNumber(
        tab,
        "Automatic Bang On Factor (multiple of saturation with Kp, 0 to turn off)",
        String( steerConfig.steeringPidAutoBangOnFactor, 4 ),
        ControlColor::Peterriver,
        []( ControlTreeIterator control, int id ) {
          steerConfig.steeringPidAutoBangOnFactor = control->value.toDouble();
          notifySensorTask( SensorNotifyBits::RefreshPidValues );
        },
        0,
        10,
        0.1 );
    }
    {
      addNumber(
        tab,
        "Turn Output on if error is greater (BangOn)",
        String( steerConfig.steeringPidBangOn, 4 ),
        ControlColor::Peterriver,
        []( ControlTreeIterator control, int id ) {
          steerConfig.steeringPidBangOn = control->value.toDouble();
          notifySensorTask( SensorNotifyBits::RefreshPidValues );
        },
        0,
        50,
        0.01 );
    }
    {
      addNumber(
        tab,
        "Turn Output off if error is smaller (BangOff)",
        String( steerConfig.steeringPidBangOff, 4 ),
        ControlColor::Peterriver,
        []( ControlTreeIterator control, int id ) {
          steerConfig.steeringPidBangOff = control->value.toDouble();
          notifySensorTask( SensorNotifyBits::RefreshPidValues );
        },
        0,
        50,
        0.01 );
    }
    {
      addNumber(
        tab,
        "PWM Threshold",
        String( steerConfig.steeringPidPwmThreshold ),
        ControlColor::Peterriver,
        []( ControlTreeIterator control, int id ) {
          steerConfig.steeringPidPwmThreshold = control->value.toInt();
        },
        0,
        255,
        1 );
    }
    {
      addNumber(
        tab,
        "Minimum PWM",
        String( steerConfig.steeringPidMinPwm ),
        ControlColor::Peterriver,
        []( ControlTreeIterator control, int id ) {
          steerConfig.steeringPidMinPwm = control->value.toInt();
        },
        0,
        255,
        1 );
    }
  }

  // NTRIP/GPS Tab
  {
    ControlTreeIterator tab =
      ESPUI.addControl( ControlType::Tab, "NTRIP/GPS", "NTRIP/GPS" );

    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "RTK Correction*",
                          String( ( int )steerConfig.rtkCorrectionType ),
                          ControlColor::Wetasphalt,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.rtkCorrectionType =
                              ( SteerConfig::RtkCorrectionType )control->value.toInt();
                            setResetButtonToRed();
                          } );
      ESPUI.addControl(
        sel, ControlType::Option, "No Correction", "0", ControlColor::Alizarin );
      ESPUI.addControl( sel, ControlType::Option, "NTRIP", "1", ControlColor::Alizarin );
      // ESPUI.addControl( ControlType::Option, "UDP", "2",
      // ControlColor::Alizarin); ESPUI.addControl( ControlType::Option,
      // "TCP", "3", ControlColor::Alizarin);
    }

    ESPUI.addControl( tab,
                      ControlType::Text,
                      "Server*",
                      String( steerConfig.rtkCorrectionServer ),
                      ControlColor::Wetasphalt,
                      []( ControlTreeIterator control, int id ) {
                        control->value.toCharArray(
                          steerConfig.rtkCorrectionServer,
                          sizeof( steerConfig.rtkCorrectionServer ) );
                        setResetButtonToRed();
                      } );
    ESPUI.addControl( tab,
                      ControlType::Text,
                      "Username*",
                      String( steerConfig.rtkCorrectionUsername ),
                      ControlColor::Wetasphalt,
                      []( ControlTreeIterator control, int id ) {
                        control->value.toCharArray(
                          steerConfig.rtkCorrectionUsername,
                          sizeof( steerConfig.rtkCorrectionUsername ) );
                        setResetButtonToRed();
                      } );
    ESPUI.addControl( tab,
                      ControlType::Text,
                      "Password*",
                      String( steerConfig.rtkCorrectionPassword ),
                      ControlColor::Wetasphalt,
                      []( ControlTreeIterator control, int id ) {
                        control->value.toCharArray(
                          steerConfig.rtkCorrectionPassword,
                          sizeof( steerConfig.rtkCorrectionPassword ) );
                        setResetButtonToRed();
                      } );
    ESPUI.addControl( tab,
                      ControlType::Text,
                      "Mountpoint*",
                      String( steerConfig.rtkCorrectionMountpoint ),
                      ControlColor::Wetasphalt,
                      []( ControlTreeIterator control, int id ) {
                        control->value.toCharArray(
                          steerConfig.rtkCorrectionMountpoint,
                          sizeof( steerConfig.rtkCorrectionMountpoint ) );
                        setResetButtonToRed();
                      } );
    {
      addNumber(
        tab,
        "Port*",
        String( steerConfig.rtkCorrectionPort ),
        ControlColor::Wetasphalt,
        []( ControlTreeIterator control, int id ) {
          steerConfig.rtkCorrectionPort = control->value.toInt();
        },
        1,
        65535,
        1 );
    }

    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "Baudrate GPS",
                          String( steerConfig.rtkCorrectionBaudrate ),
                          ControlColor::Peterriver,
                          []( ControlTreeIterator control, int id ) {
                            uint32_t baudrate                 = control->value.toInt();
                            steerConfig.rtkCorrectionBaudrate = baudrate;
                            Serial2.updateBaudRate( baudrate );
                          } );
      addBaudrateOptions( sel );
    }

    addNumber(
      tab,
      "Intervall to send Position",
      String( steerConfig.ntripPositionSendIntervall ),
      ControlColor::Peterriver,
      []( ControlTreeIterator control, int id ) {
        steerConfig.ntripPositionSendIntervall = control->value.toInt();
      },
      1,
      300,
      1 );

    textNmeaToSend = ESPUI.addControl(
      tab,
      ControlType::Text,
      "NMEA-String to send (leave empty to send live position)",
      String( steerConfig.rtkCorrectionNmeaToSend ),
      ControlColor::Peterriver,
      []( ControlTreeIterator control, int id ) {
        control->value.toCharArray( steerConfig.rtkCorrectionNmeaToSend,
                                    sizeof( steerConfig.rtkCorrectionNmeaToSend ) );
      } );

    {
      ControlTreeIterator sel =
        ESPUI.addControl( tab,
                          ControlType::Select,
                          "Send NMEA-data to",
                          String( ( int )steerConfig.sendNmeaDataTo ),
                          ControlColor::Peterriver,
                          []( ControlTreeIterator control, int id ) {
                            steerConfig.sendNmeaDataTo =
                              ( SteerConfig::SendNmeaDataTo )control->value.toInt();
                          } );
      ESPUI.addControl(
        sel, ControlType::Option, "Nowhere", "0", ControlColor::Alizarin );
      ESPUI.addControl( sel, ControlType::Option, "UDP", "1", ControlColor::Alizarin );
      // ESPUI.addControl( ControlType::Option, "TCP", "2",
      // ControlColor::Alizarin);
      ESPUI.addControl( sel, ControlType::Option, "Serial", "3", ControlColor::Alizarin );
      ESPUI.addControl(
        sel, ControlType::Option, "Serial1", "4", ControlColor::Alizarin );
      ESPUI.addControl(
        sel, ControlType::Option, "Serial2", "5", ControlColor::Alizarin );
      // ESPUI.addControl( ControlType::Option, "Bluetooth", "6",
      // ControlColor::Alizarin);
    }

    addNumber(
      tab,
      "Port to send Data to*",
      String( steerConfig.sendNmeaDataUdpPort ),
      ControlColor::Wetasphalt,
      []( ControlTreeIterator control, int id ) {
        steerConfig.sendNmeaDataUdpPort = control->value.toInt();
        setResetButtonToRed();
      },
      1,
      ULONG_MAX,
      1 );
    addNumber(
      tab,
      "Port to send Data from*",
      String( steerConfig.sendNmeaDataUdpPortFrom ),
      ControlColor::Wetasphalt,
      []( ControlTreeIterator control, int id ) {
        steerConfig.sendNmeaDataUdpPortFrom = control->value.toInt();
        setResetButtonToRed();
      },
      1,
      ULONG_MAX,
      1 );

    {
      addNumber(
        tab,
        "TCP-Socket for a direct connection to the GPS-Receiver (set to 0 to deactivate, can be used for configuration with u-center or with 3rd-party software)*",
        String( steerConfig.sendNmeaDataTcpPort ),
        ControlColor::Wetasphalt,
        []( ControlTreeIterator control, int id ) {
          steerConfig.sendNmeaDataTcpPort = control->value.toInt();
        },
        1,
        ULONG_MAX,
        1 );
    }
  }

  if( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance ) {
    // Channels Tab
    {
      ControlTreeIterator tab =
        ESPUI.addControl( ControlType::Tab, "Channels", "Channels" );

      {
        addNumber(
          tab,
          "Channel ID Autosteer Enable",
          String( steerConfig.qogChannelIdAutosteerEnable ),
          ControlColor::Peterriver,
          []( ControlTreeIterator control, int id ) {
            steerConfig.qogChannelIdAutosteerEnable = control->value.toInt();
          },
          1,
          ULONG_MAX,
          1 );
      }
      {
        addNumber(
          tab,
          "Channel ID Workswitch",
          String( steerConfig.qogChannelIdWorkswitch ),
          ControlColor::Peterriver,
          []( ControlTreeIterator control, int id ) {
            steerConfig.qogChannelIdWorkswitch = control->value.toInt();
          },
          1,
          ULONG_MAX,
          1 );
      }
      {
        addNumber(
          tab,
          "Channel ID Steerswitch",
          String( steerConfig.qogChannelIdSteerswitch ),
          ControlColor::Peterriver,
          []( ControlTreeIterator control, int id ) {
            steerConfig.qogChannelIdWorkswitch = control->value.toInt();
          },
          1,
          ULONG_MAX,
          1 );
      }

      {
        addNumber(
          tab,
          "Channel ID Wheel Angle",
          String( steerConfig.qogChannelIdWheelAngle ),
          ControlColor::Peterriver,
          []( ControlTreeIterator control, int id ) {
            steerConfig.qogChannelIdWheelAngle = control->value.toInt();
          },
          1,
          ULONG_MAX,
          1 );
      }

      {
        addNumber(
          tab,
          "Channel ID Steer Angle Setpoint",
          String( steerConfig.qogChannelIdSetpointSteerAngle ),
          ControlColor::Peterriver,
          []( ControlTreeIterator control, int id ) {
            steerConfig.qogChannelIdSetpointSteerAngle = control->value.toInt();
          },
          1,
          ULONG_MAX,
          1 );
      }

      {
        addNumber(
          tab,
          "Channel ID Orientation",
          String( steerConfig.qogChannelIdOrientation ),
          ControlColor::Peterriver,
          []( ControlTreeIterator control, int id ) {
            steerConfig.qogChannelIdOrientation = control->value.toInt();
          },
          1,
          ULONG_MAX,
          1 );
      }

      {
        addNumber(
          tab,
          "Channel ID GPS Data In",
          String( steerConfig.qogChannelIdGpsDataIn ),
          ControlColor::Peterriver,
          []( ControlTreeIterator control, int id ) {
            steerConfig.qogChannelIdGpsDataIn = control->value.toInt();
          },
          1,
          ULONG_MAX,
          1 );
      }
      {
        addNumber(
          tab,
          "Channel ID GPS Data Out",
          String( steerConfig.qogChannelIdGpsDataOut ),
          ControlColor::Peterriver,
          []( ControlTreeIterator control, int id ) {
            steerConfig.qogChannelIdGpsDataOut = control->value.toInt();
          },
          1,
          ULONG_MAX,
          1 );
      }

      if( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance &&
          steerConfig.canBusEnabled ) {
        {
          addNumber(
            tab,
            "Channel ID CAN: Rear Hitch",
            String( steerConfig.qogChannelIdCanRearHitch ),
            ControlColor::Peterriver,
            []( ControlTreeIterator control, int id ) {
              steerConfig.qogChannelIdCanRearHitch = control->value.toInt();
            },
            1,
            ULONG_MAX,
            1 );
        }
        {
          addNumber(
            tab,
            "Channel ID CAN: Front Hitch",
            String( steerConfig.qogChannelIdCanFrontHitch ),
            ControlColor::Peterriver,
            []( ControlTreeIterator control, int id ) {
              steerConfig.qogChannelIdCanFrontHitch = control->value.toInt();
            },
            1,
            ULONG_MAX,
            1 );
        }
        {
          addNumber(
            tab,
            "Channel ID CAN: Rear RPM",
            String( steerConfig.qogChannelIdCanRearPtoRpm ),
            ControlColor::Peterriver,
            []( ControlTreeIterator control, int id ) {
              steerConfig.qogChannelIdCanRearPtoRpm = control->value.toInt();
            },
            1,
            ULONG_MAX,
            1 );
        }
        {
          addNumber(
            tab,
            "Channel ID CAN: Front RPM",
            String( steerConfig.qogChannelIdCanFrontPtoRpm ),
            ControlColor::Peterriver,
            []( ControlTreeIterator control, int id ) {
              steerConfig.qogChannelIdCanFrontPtoRpm = control->value.toInt();
            },
            1,
            ULONG_MAX,
            1 );
        }
        {
          addNumber(
            tab,
            "Channel ID CAN: Motor RPM",
            String( steerConfig.qogChannelIdCanMotorRpm ),
            ControlColor::Peterriver,
            []( ControlTreeIterator control, int id ) {
              steerConfig.qogChannelIdCanMotorRpm = control->value.toInt();
            },
            1,
            ULONG_MAX,
            1 );
        }
        {
          addNumber(
            tab,
            "Channel ID CAN: Wheel-based Speed",
            String( steerConfig.qogChannelIdCanWheelbasedSpeed ),
            ControlColor::Peterriver,
            []( ControlTreeIterator control, int id ) {
              steerConfig.qogChannelIdCanWheelbasedSpeed = control->value.toInt();
            },
            1,
            ULONG_MAX,
            1 );
        }
      }
    }
  }

  // Default Configurations Tab
  {
    ControlTreeIterator tab =
      ESPUI.addControl( ControlType::Tab, "Configurations", "Configurations" );
    ESPUI.addControl(
      tab,
      ControlType::Label,
      "Attention:",
      "These Buttons here reset the whole config. This affects the WIFI too, if not configured otherwise below. You have to press \"Apply & Reboot\" above to actualy store them.",
      ControlColor::Carrot );

    ESPUI.addControl( tab,
                      ControlType::Label,
                      "OTA Update:",
                      "<a href='/update'>Update</a>",
                      ControlColor::Carrot );

    ESPUI.addControl( tab,
                      ControlType::Label,
                      "Download the config:",
                      "<a href='config.json'>Configuration</a>",
                      ControlColor::Carrot );

    ESPUI.addControl(
      tab,
      ControlType::Label,
      "Upload the config:",
      "<form method='POST' action='/upload-config' enctype='multipart/form-data'><input name='f' type='file'><input type='submit'></form>",
      ControlColor::Carrot );

    ESPUI.addControl( tab,
                      ControlType::Label,
                      "Download the calibration:",
                      "<a href='calibration.json'>Calibration</a>",
                      ControlColor::Carrot );

    ESPUI.addControl(
      tab,
      ControlType::Label,
      "Upload the calibration:",
      "<form method='POST' action='/upload-calibration' enctype='multipart/form-data'><input name='f' type='file'><input type='submit'></form>",
      ControlColor::Carrot );
    // onchange='this.form.submit()'
    {
      ESPUI.addControl( tab,
                        ControlType::Switcher,
                        "Retain WIFI settings",
                        steerConfig.retainWifiSettings ? "1" : "0",
                        ControlColor::Peterriver,
                        []( ControlTreeIterator control, int id ) {
                          steerConfig.retainWifiSettings = control->value.toInt() == 1;
                        } );
    }
    {
      ESPUI.addControl(
        tab,

        ControlType::Button,
        "Set Settings To Default*",
        "Defaults",
        ControlColor::Wetasphalt,
        []( ControlTreeIterator control, int id ) {
          char ssidAp[24], passwordAp[24], ssidSta[24], passwordSta[24], hostname[24];

          if( steerConfig.retainWifiSettings ) {
            memcpy( hostname, steerConfig.hostname, sizeof( hostname ) );

            memcpy( ssidAp, steerConfig.ssidAp, sizeof( ssidAp ) );
            memcpy( passwordAp, steerConfig.passwordAp, sizeof( passwordAp ) );
            memcpy( ssidSta, steerConfig.ssidSta, sizeof( ssidSta ) );
            memcpy( passwordSta, steerConfig.passwordSta, sizeof( passwordSta ) );
          }

          steerConfig = steerConfigDefaults;

          if( steerConfig.retainWifiSettings ) {
            memcpy( steerConfig.hostname, hostname, sizeof( hostname ) );
            memcpy( steerConfig.ssidAp, ssidAp, sizeof( ssidAp ) );
            memcpy( steerConfig.passwordAp, passwordAp, sizeof( passwordAp ) );
            memcpy( steerConfig.ssidSta, ssidSta, sizeof( ssidSta ) );
            memcpy( steerConfig.passwordSta, passwordSta, sizeof( passwordSta ) );
          }

          setResetButtonToRed();
        } );
    }

    tabConfigurations = tab;
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
   * Anyone who is able to sniff traffic will be able to intercept your
   * password since it is transmitted in cleartext. Just add a username and
   * password, for example begin("ESPUI Control", "username", "password")
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

  ESPUI.server->on( "/config.json", HTTP_GET, []( AsyncWebServerRequest* request ) {
    request->send( SPIFFS, "/config.json", "application/json", true );
  } );
  ESPUI.server->on( "/calibration.json", HTTP_GET, []( AsyncWebServerRequest* request ) {
    request->send( SPIFFS, "/calibration.json", "application/json", true );
  } );

  // upload a file to /upload-config
  ESPUI.server->on(
    "/upload-config",
    HTTP_POST,
    []( AsyncWebServerRequest* request ) { request->send( 200 ); },
    [tabConfigurations]( AsyncWebServerRequest* request,
                         String                 filename,
                         size_t                 index,
                         uint8_t*               data,
                         size_t                 len,
                         bool                   final ) {
      if( !index ) {
        request->_tempFile = SPIFFS.open( "/config.json", "w" );
      }

      if( request->_tempFile ) {
        if( len ) {
          request->_tempFile.write( data, len );
        }

        if( final ) {
          request->_tempFile.close();
          setResetButtonToRed();
          String str( "/#tab" );
          str += tabConfigurations->id;
          request->redirect( str );
        }
      }
    } );

  // upload a file to /upload-calibration
  ESPUI.server->on(
    "/upload-calibration",
    HTTP_POST,
    []( AsyncWebServerRequest* request ) { request->send( 200 ); },
    [tabConfigurations]( AsyncWebServerRequest* request,
                         String                 filename,
                         size_t                 index,
                         uint8_t*               data,
                         size_t                 len,
                         bool                   final ) {
      if( !index ) {
        request->_tempFile = SPIFFS.open( "/calibration.json", "w" );
      }

      if( request->_tempFile ) {
        if( len ) {
          request->_tempFile.write( data, len );
        }

        if( final ) {
          request->_tempFile.close();
          setResetButtonToRed();
          String str( "/#tab" );
          str += tabConfigurations->id;
          request->redirect( str );
        }
      }
    } );

  if( steerConfig.enableOTA ) {
    AsyncElegantOTA.begin( ESPUI.server );
  }

  spi.begin();

  pinMode( ( int )steerConfig.gpioAds131m04Drdy, INPUT_PULLUP );
  pinMode( ( int )steerConfig.gpioAccGyroDrdy, INPUT_PULLUP );
  pinMode( ( int )steerConfig.gpioMagDrdy, INPUT_PULLUP );

  pinMode( ( int )steerConfig.gpioAds131m04Cs, OUTPUT );
  pinMode( ( int )steerConfig.gpioAccGyroCs, OUTPUT );
  pinMode( ( int )steerConfig.gpioMagCs, OUTPUT );

  digitalWrite( ( int )steerConfig.gpioAds131m04Cs, HIGH );
  digitalWrite( ( int )steerConfig.gpioAccGyroCs, HIGH );
  digitalWrite( ( int )steerConfig.gpioMagCs, HIGH );

  pinMode( 15, OUTPUT );
  pinMode( 32, OUTPUT );

  initIdleStats();

  initSensors();

  initRtkCorrection();

  initCan();

  initAutosteer();

  notifySensorTask( SensorNotifyBits::RefreshFiltersSamplerate );
}

void
loop( void ) {
  dnsServer.processNextRequest();
  AsyncElegantOTA.loop();
  ArduinoOTA.handle();
  ESPUI.handleClients();

  //   showDisplay();

  vTaskDelay( 100 );

  //   digitalWrite( 33, LOW );
  //   {
  //     std::array< uint8_t, 12 > tmpWriteBuffer;
  //     std::array< uint8_t, 70 > tmpReadBuffer;
  //
  //     spi.read( tmpWriteBuffer.data(), tmpWriteBuffer.size() );
  //   }
  //   digitalWrite( 33, HIGH );

  //     digitalWrite( 15, LOW );
  /*if( false )*/ {
    std::array< uint8_t, 70 > tmpWriteBuffer;
    std::array< uint8_t, 70 > tmpReadBuffer;

    //         digitalWrite( 15, LOW );
    //         spi.read( tmpWriteBuffer.data(), tmpWriteBuffer.size(), 0xAA );
    //         digitalWrite( 15, HIGH );
    //
    //         delay(5);
    //
    //     digitalWrite( 15, LOW );
    //     spi.write( tmpReadBuffer.data(),
    //                tmpReadBuffer.size(),
    //                tmpWriteBuffer.data(),
    //                tmpWriteBuffer.size() );
    //     digitalWrite( 15, HIGH );
    //
    //             delay(5);
    //
    //     digitalWrite( 15, LOW );
    //     spi.write_then_read( tmpWriteBuffer.data(),
    //                          tmpWriteBuffer.size(),
    //                          tmpReadBuffer.data(),
    //                          tmpReadBuffer.size(),
    //                          0xAA );
    //     digitalWrite( 15, HIGH );
    //
    //         delay(5);
    //
    //         digitalWrite( 15, LOW );
    //         spi.write_and_read( tmpWriteBuffer.data(), tmpWriteBuffer.size() );
    //         digitalWrite( 15, HIGH );
  }
}
