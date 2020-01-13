// should handle the normal messages from AOG on the PGN broadcast port
// not special like NTRIP on other ports

#include "main.hpp"

struct UdpFromAogData {
  uint lastReceived7FFE;
  int16_t distanceFromGuidanceLine = 32020;
  float requiredSteerAngle;
  byte uTurnRelais;
};
extern UdpFromAogData udpAogData;

struct UdpActualData {
  uint lastSent;

  float steerAngleActual;

  bool workSwitch;
  bool steerSwitch = false; // safe init
  bool remoteSwitch; // unused

  int16_t pwm;

  float roll = 9999/16.0;
  float heading = 9999/16.0;

};
extern UdpActualData udpActualData;

void udpHandlerInit();
void udpHandlerCreateReceiveHandler();
String udpHandlerTimeGenerator(uint last);
void udpHandlerSendData( void* z ) ;
void udpHandlerWebUpdate();
