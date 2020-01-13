#include <stdint.h>


#ifndef WEBUI_HPP
#define WEBUI_HPP

extern uint16_t webLabelLoad;
extern uint16_t webLabelPgnStatus;
extern uint16_t webButtonReboot;

extern uint16_t webTabHardware;
extern uint16_t webTabGPS;
extern uint16_t webTabIMU;
extern uint16_t webTabSteeringAngle;
extern int16_t webTabSteeringActuator;
extern uint16_t webTabUturn;
extern uint16_t webTabWorkSteerSwitch;




void webInitCore();
void webStart();
void webChangeNeedsReboot();

#endif
