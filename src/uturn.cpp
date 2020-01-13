#include "uturn.hpp"
#include "webUi.hpp"
#include "ioAccess.hpp"
#include "main.hpp"
#include "udpHandler.hpp"
#include <ESPUI.h>

// init webinterface and start task for the uturn relais
void uturnInit() {
  // Webinterface
  uint16_t sel = ESPUI.addControl( ControlType::Select, "Relay 1", (String)preferences.getUChar("uturnR1", 255), ControlColor::Wetasphalt, webTabUturn,
    []( Control * control, int id ) {
      preferences.putUChar("uturnR1", control->value.toInt());
      control->color = ControlColor::Carrot;
      ESPUI.updateControl( control );
      webChangeNeedsReboot();
    } );
  ESPUI.addControl( ControlType::Option, "None", "255", ControlColor::Alizarin, sel );
  ioAccessWebListDigitalOut(sel);

  sel = ESPUI.addControl( ControlType::Select, "Relay 2", (String)preferences.getUChar("uturnR2", 255), ControlColor::Wetasphalt, webTabUturn,
    []( Control * control, int id ) {
      preferences.putUChar("uturnR2", control->value.toInt());
      control->color = ControlColor::Carrot;
      ESPUI.updateControl( control );
      webChangeNeedsReboot();
    } );
  ESPUI.addControl( ControlType::Option, "None", "255", ControlColor::Alizarin, sel );
  ioAccessWebListDigitalOut(sel);

  sel = ESPUI.addControl( ControlType::Select, "Relay 3", (String)preferences.getUChar("uturnR3", 255), ControlColor::Wetasphalt, webTabUturn,
    []( Control * control, int id ) {
      preferences.putUChar("uturnR3", control->value.toInt());
      control->color = ControlColor::Carrot;
      ESPUI.updateControl( control );
      webChangeNeedsReboot();
    } );
  ESPUI.addControl( ControlType::Option, "None", "255", ControlColor::Alizarin, sel );
  ioAccessWebListDigitalOut(sel);

  sel = ESPUI.addControl( ControlType::Select, "Relay 4", (String)preferences.getUChar("uturnR4", 255), ControlColor::Wetasphalt, webTabUturn,
    []( Control * control, int id ) {
      preferences.putUChar("uturnR4", control->value.toInt());
      control->color = ControlColor::Carrot;
      ESPUI.updateControl( control );
      webChangeNeedsReboot();
    } );
  ESPUI.addControl( ControlType::Option, "None", "255", ControlColor::Alizarin, sel );
  ioAccessWebListDigitalOut(sel);

  sel = ESPUI.addControl( ControlType::Select, "Relay 5", (String)preferences.getUChar("uturnR5", 255), ControlColor::Wetasphalt, webTabUturn,
    []( Control * control, int id ) {
      preferences.putUChar("uturnR5", control->value.toInt());
      control->color = ControlColor::Carrot;
      ESPUI.updateControl( control );
      webChangeNeedsReboot();
    } );
  ESPUI.addControl( ControlType::Option, "None", "255", ControlColor::Alizarin, sel );
  ioAccessWebListDigitalOut(sel);

  sel = ESPUI.addControl( ControlType::Select, "Relay 6", (String)preferences.getUChar("uturnR6", 255), ControlColor::Wetasphalt, webTabUturn,
    []( Control * control, int id ) {
      preferences.putUChar("uturnR6", control->value.toInt());
      control->color = ControlColor::Carrot;
      ESPUI.updateControl( control );
      webChangeNeedsReboot();
    } );
  ESPUI.addControl( ControlType::Option, "None", "255", ControlColor::Alizarin, sel );
  ioAccessWebListDigitalOut(sel);


    // start task
    xTaskCreate( uturnTask, "uturn", 4096, NULL, 4, NULL );
}

// against gliches/noise: two values must show same value for a change + a hysteresis
void uturnTask(void *z) {
  uint lastUpdate = 0;
  uint8_t ports [6];
  ports[0] = preferences.getUChar("uturnR1", 255);
  ports[1] = preferences.getUChar("uturnR2", 255);
  ports[2] = preferences.getUChar("uturnR3", 255);
  ports[3] = preferences.getUChar("uturnR4", 255);
  ports[4] = preferences.getUChar("uturnR5", 255);
  ports[5] = preferences.getUChar("uturnR6", 255);

  while (1) {
    if (udpAogData.lastReceived7FFE > lastUpdate) {
      lastUpdate = udpAogData.lastReceived7FFE;
      // set Outputs
      byte data = udpAogData.uTurnRelais;
      // get value for first bit
      bool value = data & 1;
      ioAccessSetDigitalOutput(ports[0], value);
      // relay 2
      // shift one bit for the next
      data = data >>1;
      value = data & 1;
      ioAccessSetDigitalOutput(ports[1], value);
      // relay 3
      // shift one bit for the next
      data = data >>1;
      value = data & 1;
      ioAccessSetDigitalOutput(ports[2], value);
      // relay 4
      // shift one bit for the next
      data = data >>1;
      value = data & 1;
      ioAccessSetDigitalOutput(ports[3], value);
      // relay 5
      // shift one bit for the next
      data = data >>1;
      value = data & 1;
      ioAccessSetDigitalOutput(ports[4], value);
      // relay 6
      // shift one bit for the next
      data = data >>1;
      value = data & 1;
      ioAccessSetDigitalOutput(ports[5], value);
    }
    vTaskDelay( 8 );
  }
}
