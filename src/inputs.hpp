#ifndef inputs_HPP
#define inputs_HPP
#include <stdint.h>


constexpr int inputsHysteresis = 2;

struct InputsWasData {
  bool invertSensor;
  float center;
  float degreMultiplier;
  float statusRaw;
  float statusDegrees;
  int wheelbase;
  int trackWidth;
  uint8_t correction;
  uint8_t inputPort;
};
extern InputsWasData inputsWasSetup;

struct InputsSwitchesConfig {
  bool steerSwitchIsButton;
  bool steerSwitchInvert;
  uint8_t steerSwitchThreshold;
  uint8_t workSwitchThreshold;
  uint8_t steerSwitchPort;
  uint8_t workSwitchPort;
};
extern InputsSwitchesConfig inputsSwitchesSetup;


extern int inputsWasWebStatus;

void inputsSwitchesInit();
void inputsSwitchesTask(void *z);

void inputsWheelAngleInit();
void inputsWheelAngleTask(void *z);
void inputsWheelAngleStatusUpdate();

#endif
