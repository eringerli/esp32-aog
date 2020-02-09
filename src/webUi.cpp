#include <ESPUI.h>
#include "webUi.hpp"

uint16_t webLabelLoad;
uint16_t webLabelPgnStatus;
uint16_t webButtonReboot;

uint16_t webTabHardware;
uint16_t webTabGPS;
uint16_t webTabIMU;
uint16_t webTabSteeringAngle;
int16_t webTabSteeringActuator;
uint16_t webTabUturn;
uint16_t webTabWorkSteerSwitch;

void webInitCore() {
  webLabelLoad = ESPUI.addControl( ControlType::Label, "Load:", "", ControlColor::Turquoise );
  webLabelPgnStatus = ESPUI.addControl( ControlType::Label, "PGNs:", "", ControlColor::Turquoise );
  webButtonReboot = ESPUI.addControl( ControlType::Button, "If this turn red, you have to", "Reboot", ControlColor::Emerald, Control::noParent,
    []( Control * control, int id ) {
      if ( id == B_UP ) {
        ESP.restart();
      }
    } );
  webTabGPS = ESPUI.addControl( ControlType::Tab, "GPS", "GPS" );
  webTabIMU = ESPUI.addControl( ControlType::Tab, "IMU", "IMU" );
  webTabSteeringAngle = ESPUI.addControl( ControlType::Tab, "Steering angle", "Steering angle" );
  webTabSteeringActuator = ESPUI.addControl( ControlType::Tab, "Steering actuator", "Steering actuator" );
  webTabUturn = ESPUI.addControl( ControlType::Tab, "U-Turn", "U-Turn" );
  webTabWorkSteerSwitch = ESPUI.addControl( ControlType::Tab, "Work-/Steer switch", "Work-/Steer switch" );
  webTabHardware = ESPUI.addControl( ControlType::Tab, "Hardware", "Hardware" );
}

void webStart() {
  // Use SPIFFS if the ressources get a bit tough in the end, change readme in that case
  //ESPUI.beginSPIFFS("AgOpenGPS ESP32 controler");
  ESPUI.jsonUpdateDocumentSize = 2000;
  ESPUI.jsonInitialDocumentSize = 32000;
  ESPUI.begin("AgOpenGPS ESP32 controler");
}
void webChangeNeedsReboot(){
  ESPUI.getControl( webButtonReboot )->color = ControlColor::Alizarin;
  ESPUI.updateControl( webButtonReboot );
};
