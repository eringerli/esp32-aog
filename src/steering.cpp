#include <AutoPID.h>
#include <ESPUI.h>
#include "main.hpp"
#include "steering.hpp"
#include "udpHandler.hpp"
#include "ioAccess.hpp"
#include "webUi.hpp"

SteeringCurrentSettings steeringSettings;

void steeringInit() {
  // Webinterface
  steeringSettings.isActivePort = preferences.getUChar("steeringPortEn", 255);
  uint16_t sel = ESPUI.addControl( ControlType::Select, "Enables Output", (String)steeringSettings.isActivePort , ControlColor::Wetasphalt, webTabSteeringActuator,
    []( Control * control, int id ) {
      preferences.putUChar("steeringPortEn", control->value.toInt());
      control->color = ControlColor::Carrot;
      ESPUI.updateControl( control );
      webChangeNeedsReboot();
    } );
  ESPUI.addControl( ControlType::Option, "None", "255", ControlColor::Alizarin, sel );
  ioAccessWebListDigitalOut(sel);
  steeringSettings.invertOutput = preferences.getBool("steeringInvert");
  ESPUI.addControl( ControlType::Switcher, "Invert Output", String( (int)steeringSettings.invertOutput ) , ControlColor::Wetasphalt, webTabSteeringActuator,
    []( Control * control, int id ) {
      steeringSettings.invertOutput= (boolean)control->value.toInt();
      preferences.putBool("steeringInvert", steeringSettings.invertOutput);
      control->color = ControlColor::Carrot;
      ESPUI.updateControl( control );
    } );
  // Type
  steeringSettings.type = (SteeringCurrentSettings::SteeringType)preferences.getUChar("steeringType", 0);
  sel = ESPUI.addControl( ControlType::Select, "Output Type", String((int)steeringSettings.type) , ControlColor::Wetasphalt, webTabSteeringActuator,
    []( Control * control, int id ) {
      preferences.putUChar("steeringType", control->value.toInt());
      control->color = ControlColor::Carrot;
      ESPUI.updateControl( control );
      webChangeNeedsReboot();
    } );
  ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
  ESPUI.addControl( ControlType::Option, "Polarity & PWM", "1", ControlColor::Alizarin, sel );
  ESPUI.addControl( ControlType::Option, "PWM (50% = straight)", "2", ControlColor::Alizarin, sel );
  // Parameters
  steeringSettings.Kp = preferences.getFloat("steeringKp", steeringSettings.Kp);
  ESPUI.addControl( ControlType::Number, "Proportional Gain (Kp)", (String)steeringSettings.Kp, ControlColor::Wetasphalt, webTabSteeringActuator,
    []( Control * control, int id ) {
      steeringSettings.Kp = control->value.toFloat();
      preferences.putFloat("steeringKp", steeringSettings.Kp);
      control->color = ControlColor::Carrot;
      ESPUI.updateControl( control );
    } );
  steeringSettings.Ki = preferences.getFloat("steeringKi", steeringSettings.Ki);
  ESPUI.addControl( ControlType::Number, "Integral Gain (Ki)", (String)steeringSettings.Ki, ControlColor::Wetasphalt, webTabSteeringActuator,
    []( Control * control, int id ) {
      steeringSettings.Ki = control->value.toFloat();
      preferences.putFloat("steeringKi", steeringSettings.Ki);
      control->color = ControlColor::Carrot;
      ESPUI.updateControl( control );
    } );
  steeringSettings.Kd = preferences.getFloat("steeringKd", steeringSettings.Kd);
  ESPUI.addControl( ControlType::Number, "Differential Gain (Kd)", (String)steeringSettings.Kd, ControlColor::Wetasphalt, webTabSteeringActuator,
    []( Control * control, int id ) {
      steeringSettings.Kd = control->value.toFloat();
      preferences.putFloat("steeringKd", steeringSettings.Kd);
      control->color = ControlColor::Carrot;
      ESPUI.updateControl( control );
    } );
  steeringSettings.BangOn = preferences.getFloat("steeringBangOn", steeringSettings.BangOn);
  ESPUI.addControl( ControlType::Number, "Degrees differece for full on (BangOn)", (String)steeringSettings.BangOn, ControlColor::Wetasphalt, webTabSteeringActuator,
    []( Control * control, int id ) {
      steeringSettings.BangOn = control->value.toFloat();
      preferences.putFloat("steeringBangOn", steeringSettings.BangOn);
      control->color = ControlColor::Carrot;
      ESPUI.updateControl( control );
    } );
  steeringSettings.BangOff = preferences.getFloat("steeringBangOff", steeringSettings.BangOff);
  ESPUI.addControl( ControlType::Number, "Degrees differece for off (BangOff)", (String)steeringSettings.BangOff, ControlColor::Wetasphalt, webTabSteeringActuator,
    []( Control * control, int id ) {
      steeringSettings.BangOff = control->value.toFloat();
      preferences.putFloat("steeringBangOff", steeringSettings.BangOff);
      control->color = ControlColor::Carrot;
      ESPUI.updateControl( control );
    } );
  steeringSettings.minPWM = preferences.getFloat("steeringMinPwm", steeringSettings.minPWM);
  ESPUI.addControl( ControlType::Number, "Minimal PWM Value (0-255)", (String)steeringSettings.minPWM, ControlColor::Wetasphalt, webTabSteeringActuator,
    []( Control * control, int id ) {
      steeringSettings.minPWM = control->value.toFloat();
      preferences.putFloat("steeringMinPwm", steeringSettings.minPWM);
      control->color = ControlColor::Carrot;
      ESPUI.updateControl( control );
    } );
  ESPUI.addControl( ControlType::Switcher, "Test minimal PWM", "0" , ControlColor::Wetasphalt, webTabSteeringActuator,
    []( Control * control, int id ) {
      steeringSettings.testMinPwm = (boolean)control->value.toInt();
      if (steeringSettings.testMinPwm) {
        control->color = ControlColor::Alizarin;
      } else {
        control->color = ControlColor::Wetasphalt;
      }
      ESPUI.updateControl( control );
    } );
  // start Task
  xTaskCreate( steeringTask, "Steering", 4096, NULL, 8, NULL );
}

void steeringTask( void* z ) {
  constexpr TickType_t xFrequency = 20;
  constexpr time_t udpTimeout = 1000;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  float pidOutput = 0;
  AutoPID pid(
    &( udpActualData.steerAngleActual ),
    &( udpAogData.requiredSteerAngle ),
    &( pidOutput ),
    -255, 255,
    steeringSettings.Kp, steeringSettings.Ki, steeringSettings.Kd );
  pid.setTimeStep( xFrequency );
  bool isActive = false;

  while(1) {
    // check all conditions when there should be no output
    if (!udpActualData.steerSwitch ||                    // steering disabled == true
        steeringSettings.testMinPwm ||                           // just minPWM test
        udpAogData.lastReceived7FFE < (millis() - udpTimeout) || // timeout
        udpAogData.distanceFromGuidanceLine == 32020 ) {         // AOG disabled
      // no steering active
      if (udpAogData.lastReceived7FFE < (millis() - udpTimeout)) {
        udpActualData.steerSwitch = false;  // if reason is timeout, disable steer switch (if button, then it avoids nasty surprises later when there is communication again)
                                            // if switch, then it will not help
      }
      // disable pid
      pid.stop();
      pidOutput = 0;
      isActive = false;
    } else {
      // Steering enabled
      isActive = true;
      // update parameters if changed on the web interface
      pid.setGains(steeringSettings.Kp, steeringSettings.Ki, steeringSettings.Kd);
      pid.setBangBang(steeringSettings.BangOn, steeringSettings.BangOff);

      // here comes the magic: executing the PID loop
      // the values are given by pointers, so the AutoPID gets them automaticaly
      pid.run();
    }

    // handle the "testPWM" case
    if (steeringSettings.testMinPwm) {
      isActive = true;
      if ( (millis() % 2048 ) < 1024 ) {
        pidOutput = 1; // set to a minimal value, for a second each direction
      } else {
        pidOutput = -1;
      }
    }

    // handle output
    ioAccessSetDigitalOutput(steeringSettings.isActivePort, isActive);

    if (steeringSettings.invertOutput) {
      pidOutput = -1 * pidOutput;
    }

    // scale the 9 bit PWM to a +- 10 bit motor, including minPWM and the type of motor
    int motorPWM = 0;
    switch(steeringSettings.type) {
      case SteeringCurrentSettings::SteeringType::twoChannelLeftRight:
        {
          if (round(pidOutput) == 0) {
            motorPWM = 0;
          } else if (pidOutput > 0) {
            motorPWM = steeringSettings.minPWM * 4 + round(pidOutput / 255 * (1023 - steeringSettings.minPWM * 4)); // minPWM times four to be compatible with the 8 bit world outside
                                                                                                              // scale the PWM output to the 10 Bit
          } else {
            motorPWM = round(pidOutput / 255 * (1023 - steeringSettings.minPWM * 4)) - steeringSettings.minPWM * 4;
          }
        }
        break;
      case SteeringCurrentSettings::SteeringType::oneChannelCenterNeutral:
        {
          if (round(pidOutput) == 0) {
            motorPWM = 511;
          } else if (pidOutput > 0) {
            motorPWM = 511 + steeringSettings.minPWM * 2 + round(pidOutput / 255 * (512 - steeringSettings.minPWM * 2));
          } else {
            motorPWM = 511 - steeringSettings.minPWM * 2 - round(pidOutput / 255 * (511 - steeringSettings.minPWM * 2));
          }
        }
        break;
      default:
        motorPWM = 0; // just be sure, do nothing if not configured what to do
    }

    ioAccessMotor1(motorPWM);

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}
