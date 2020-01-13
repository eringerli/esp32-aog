#ifndef steering_HPP
#define steering_HPP
#include <stdint.h>

struct SteeringCurrentSettings {
  // ports
  uint8_t isActivePort;

  enum class SteeringType : uint8_t {
    none = 0,
    twoChannelLeftRight = 1,
    oneChannelCenterNeutral = 2
  } type = SteeringType::none;

  // settings for calculation the PWM
  float Kp = 20;
  float Ki = 0.5;
  float Kd = 1;
  float BangOn = 15;
  float BangOff = 0.25;
  // for PWM
  uint8_t minPWM = 5;
  bool testMinPwm = false;
  bool invertOutput = false;
};
extern SteeringCurrentSettings steeringSettings;

void steeringTask( void* z );
void steeringInit();

#endif
