#include <Preferences.h>
#include <ESPUI.h>
#include "hwSetup.hpp"
#include "webUi.hpp"
#include "main.hpp"
#include "gpsRtcm.hpp"
#include "gpsNmea.hpp"
#include "ioAccess.hpp"
#include "network.hpp"



void hwSetupInitial() {
  // WiFi
  hwSetupNetworkAp();

  // nothing in the webUi
}

void hwSetupNodeMcuCytronNmea() {
  bool hwInitErrors = false;

  // for the status LED
  ioAccessInitAsDigitalOutput(2);
  status.statusPort = 2;

  // I2C
  Wire.begin(32, 33, 400000 );

  // ADS1115
  if (ioAccess_ads1115_init(0x48) == false) {
    hwInitErrors = true;
    usb.println("ERROR: Failed to initialize the ADS1115");
  }
  //internal ADC
  analogReadResolution(10);
  
  if (hwInitErrors) {
    status.hardwareStatus = Status::Hardware::error;
  } else {
    status.hardwareStatus = Status::Hardware::ok;
  }

  hwSetupWebNetwork();

  // wait 3s so the user can press the button
  delay(3000);
  // if pressed AP, else configured network
  if (ioAccessGetDigitalInput(0) == false) {
    hwSetupNetworkAp();
  } else {
    // normal networking
    uint8_t networkSetup = preferences.getUChar("networkSetup", 0);
    switch (networkSetup) {
      case 1:
         hwSetupNetworkClient();
        break;
      default:
         hwSetupNetworkAp(false);
        break;
    }
  }

  // gps
  gpsCommonInit();
  gpsRtcmSetup(GpsRtcmData::RtcmDestination::gps1);
  gpsNmeasingleReader();
}

void hwSetupF9PIoBoardNmea() {
  bool hwInitErrors = false;
  // some Variables
  hwSetupHasEthernet = true;

  // I2C
  Wire.begin(32, 33, 400000 );
 //IO init
  if (ioAccess_FXL6408_init(0x43) == false) {
    hwInitErrors = true;
    usb.println("ERROR: Failed to initialize the FXL6408");
  }
  // led
  ioAccessInitAsDigitalOutput(75);
  status.statusPort = 75;
  // disable Ethernet
  ioAccessInitAsDigitalOutput(74);
  ioAccessSetDigitalOutput(74, false);
  // set up "setup" Switch
  ioAccessInitAsDigitalInput(73, true, true);

  // ADS1115
  if (ioAccess_ads1115_init(0x48) == false) {
    hwInitErrors = true;
    usb.println("ERROR: Failed to initialize the ADS1115");
  }

  //internal ADC
  analogReadResolution(10);

  // serial
  gpio_pad_select_gpio(GPIO_NUM_14);
  gpio_set_direction(GPIO_NUM_14, GPIO_MODE_INPUT);
  gpio_set_pull_mode(GPIO_NUM_14, GPIO_FLOATING);
  gpio_pad_select_gpio(GPIO_NUM_13);
  gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(GPIO_NUM_13, GPIO_FLOATING);
  gps1.begin(115200, SERIAL_8N1, 14, 13);
  rs232.begin(57600, SERIAL_8N1, 16, 15);

  // motor
  ioAccessInitAsDigitalOutput(79);
  ioAccessInitAsDigitalOutput(80);
  ioAccessInitPwmChannel(0, 5000);
  ioAccessInitAttachToPwmChannel(4, 0);
   ioAccessMotor1 = &hwSetupF9PIoBoardMotor1;

  // digital Outputs
  ioAccessInitAsDigitalOutput(76);
  ioAccessInitAsDigitalOutput(77);
  ioAccessInitAsDigitalOutput(78);
  ioAccessInitAsDigitalOutput(12);
  ioAccessSetDigitalOutput(12, true); // PWM pin von M2 auf high => directes schalten von M2A/M2B


  // analog inputs
  ioAccessWebListAnalogIn = &hwSetupF9PIoBoardWebAnalogIn;
  ioAccessWebListDigitalOut = &hwSetupF9PIoBoardWebDigitalOut;

  if (hwInitErrors) {
    status.hardwareStatus = Status::Hardware::error;
  } else {
    status.hardwareStatus = Status::Hardware::ok;
  }

  // network
  // web
  hwSetupWebNetwork();

  // check if button for AP is pressed
  usb.println("INFO: Set up network");
  if (ioAccessGetDigitalInput(73) == false) {
    usb.println("INFO: Setup button has been pressed, use hotspot mode.");
    hwSetupNetworkAp();
  } else {
    // normal networking
    uint8_t networkSetup = preferences.getUChar("networkSetup", 0);
    usb.print("INFO: network configuration: ");
    usb.println((int)networkSetup);
    switch (networkSetup) {
      case 1:
        usb.println("INFO: Network mode: WiFi client");
         hwSetupNetworkClient();
        break;
      case 2:
         usb.println("INFO: Network mode: Ethernet (wired)");
         ioAccessSetDigitalOutput(74, true);
         hwSetupNetworkLan8720(0, -1, 23, 18);
        break;
      default:
         usb.println("INFO: Network mode: WiFi Access Point");
         hwSetupNetworkAp(false);
        break;
    }
  }
  // gps
  usb.println("INFO: Set up a single nmea GPS-Interface on gps1");
  gpsCommonInit();
  gpsRtcmSetup(GpsRtcmData::RtcmDestination::gps1);
  gpsNmeasingleReader();

}

void hwSetupF9PIoBoardWebAnalogIn(int parent) {
  ESPUI.addControl( ControlType::Option, "I1", "36", ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "I2", "39", ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "I3", "34", ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "A0 (Ref GND)", "41", ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "A0 (Ref 5V)",  "46", ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "A0 (Ref A1)",  "45", ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "A1 (Ref GND)", "42", ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "A1 (Ref 5V)",  "47", ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "A2 (Ref GND)", "43", ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "A2 (Ref 5V)",  "48", ControlColor::Alizarin, parent );
}

void hwSetupF9PIoBoardWebDigitalOut(int parent) {
  ESPUI.addControl( ControlType::Option, "M2A", "77", ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "M2B", "78", ControlColor::Alizarin, parent );
  ESPUI.addControl( ControlType::Option, "Relay", "76", ControlColor::Alizarin, parent );
}

void hwSetupF9PIoBoardMotor1(int pwm) {
  bool direction = pwm > 1;
  pwm = abs(pwm);

  if (pwm == 0) { // zero => both off
    ioAccessSetDigitalOutput(79, false);
    ioAccessSetDigitalOutput(80, false);
  } else {
    if (direction) {
      ioAccessSetDigitalOutput(79, true);
      ioAccessSetDigitalOutput(80, false);
    } else {
      ioAccessSetDigitalOutput(79, false);
      ioAccessSetDigitalOutput(80, true);
    }
  }
  ioAccessSetPwmUtil(0, pwm); // set duty cycle

}


void hwSetupWebSetup() {
  uint8_t hw = preferences.getUChar("hwSetup");

  // HW-"Plattform"
  if ( hw  == 0 ) {
    uint16_t sel = ESPUI.addControl( ControlType::Select, "Hardware", "0", ControlColor::Wetasphalt, webTabHardware,
      []( Control * control, int id ) {
        preferences.putUChar("hwSetup", control->value.toInt());
        control->color = ControlColor::Carrot;
        webChangeNeedsReboot();
      } );
    ESPUI.addControl( ControlType::Option, hwSetupHardwareIdToName(0), "0", ControlColor::Alizarin, sel );
    ESPUI.addControl( ControlType::Option, hwSetupHardwareIdToName(1), "1", ControlColor::Alizarin, sel );
    ESPUI.addControl( ControlType::Option, hwSetupHardwareIdToName(2), "2", ControlColor::Alizarin, sel );
  } else {
    uint16_t sel = ESPUI.addControl( ControlType::Select, "Hardware", String(hw), ControlColor::Wetasphalt, webTabHardware,
      []( Control * control, int id ) {} );
    ESPUI.addControl( ControlType::Option, hwSetupHardwareIdToName(hw), String(hw), ControlColor::Alizarin, sel );
  }
  // Reset
  ESPUI.addControl( ControlType::Button, "Reset to Default", "Reset", ControlColor::Wetasphalt, webTabHardware,
    []( Control * control, int id ) {
      if ( id == B_UP && control->value.equals("Reset")) {
        control->value = "Really?";
        control->color = ControlColor::Alizarin;
        ESPUI.updateControl( control );
      } else if ( id == B_UP && control->value.equals("Really?")) {
        preferences.clear();
        preferences.end();
        webChangeNeedsReboot();
      }
    } );
}

char* hwSetupHardwareIdToName(uint8_t setup) {
  switch (setup) {
    case 1:
      return (char*)"NodeMCU Cytron Nmea";
      break;
    case 2:
      return (char*)"F9P-IO-Board Nmea";
      break;
    default:
      return (char*)"Initial Setup";
      break;
  }
}
