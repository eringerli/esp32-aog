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

#include <AutoPID.h>

#include "main.hpp"

SteerSettings steerSettings;
SteerSetpoints steerSetpoints;
SteerMachineControl steerMachineControl;

AsyncUDP udpSendFrom;
AsyncUDP udpLocalPort;
AsyncUDP udpRemotePort;

double pidOutput = 0;
AutoPID pid(
        &( steerSetpoints.actualSteerAngle ),
        &( steerSetpoints.requestedSteerAngle ),
        &( pidOutput ),
        -255, 255,
        steerConfig.steeringPidKp, steerConfig.steeringPidKi, steerConfig.steeringPidKd );

constexpr time_t Timeout = 2000;

void autosteerWorker100Hz( void* z ) {
  constexpr TickType_t xFrequency = 10;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  pid.setTimeStep( xFrequency );

  for( ;; ) {
    time_t timeoutPoint = millis() - Timeout;

    // check for timeout and data from AgOpenGPS
    if( steerSetpoints.lastPacketReceived < timeoutPoint ||
        steerSetpoints.distanceFromLine == 32020/* ||
         steerSetpoints.speed < 1*/
      ) {
      steerSetpoints.enabled = false;
//       Serial.print( "Autosteer disabled: " );
//       Serial.print( timeoutPoint );
//       Serial.print( ", " );
//       Serial.print( steerSetpoints.lastPacketReceived );
//       Serial.print( ", " );
//       Serial.println( steerSetpoints.distanceFromLine );

      switch( initialisation.outputType ) {
        case SteerConfig::OutputType::HydraulicDanfoss: {
          ledcWrite( 0, 128 );
          ledcWrite( 1, 0 );
        }
        break;

        default: {
          ledcWrite( 0, 0 );
          ledcWrite( 1, 0 );
        }
        break;
      }

      if( steerConfig.gpioEn != SteerConfig::Gpio::None ) {
        digitalWrite( ( uint8_t )steerConfig.gpioEn, HIGH );
      }
    } else {
      steerSetpoints.enabled = true;
//       Serial.println( "Autosteer enabled" );

      pid.setGains( steerConfig.steeringPidKp, steerConfig.steeringPidKi, steerConfig.steeringPidKd );

      if( steerConfig.steeringPidAutoBangOnFactor ) {
        pid.setBangBang( ( ( double )0xFF / steerSettings.Kp ) * steerConfig.steeringPidAutoBangOnFactor, steerConfig.steeringPidBangOff );
      } else {
        pid.setBangBang( steerConfig.steeringPidBangOn, steerConfig.steeringPidBangOff );
      }

      // here comes the magic: executing the PID loop
      // the values are given by pointers, so the AutoPID gets them automaticaly
      pid.run();

//         Serial.print( "actualSteerAngle: " );
//         Serial.print( steerSetpoints.actualSteerAngle );
//         Serial.print( ", requestedSteerAngle: " );
//         Serial.print( steerSetpoints.requestedSteerAngle );
//         Serial.print( "pidOutput: " );
//         Serial.println( pidOutput );

      if( pidOutput ) {

        double pidOutputTmp = steerConfig.invertOutput ? pidOutput : -pidOutput;

        if( pidOutputTmp < 0 && pidOutputTmp > -steerConfig.steeringPidMinPwm ) {
          pidOutputTmp = -steerConfig.steeringPidMinPwm;
        }

        if( pidOutputTmp > 0 && pidOutputTmp < steerConfig.steeringPidMinPwm ) {
          pidOutputTmp = steerConfig.steeringPidMinPwm;
        }

        switch( initialisation.outputType ) {
          case SteerConfig::OutputType::SteeringMotorIBT2:
          case SteerConfig::OutputType::HydraulicPwm2Coil: {
            if( pidOutputTmp >= 0 ) {
              ledcWrite( 0, pidOutputTmp );
              ledcWrite( 1, 0 );
            }

            if( pidOutputTmp < 0 ) {
              ledcWrite( 0, 0 );
              ledcWrite( 1, -pidOutputTmp );
            }
          }
          break;

          case SteerConfig::OutputType::SteeringMotorCytron: {
            if( pidOutputTmp >= 0 ) {
              ledcWrite( 1, 255 );
            } else {
              ledcWrite( 0, 255 );
              pidOutputTmp = -pidOutputTmp;
            }

            ledcWrite( 0, pidOutputTmp );

            if( steerConfig.gpioEn != SteerConfig::Gpio::None ) {
              digitalWrite( ( uint8_t )steerConfig.gpioEn, HIGH );
            }
          }
          break;

          case SteerConfig::OutputType::HydraulicDanfoss: {

            // go from 25% on: max left, 50% on: center, 75% on: right max
            if( pidOutputTmp >  250 ) {
              pidOutputTmp =  250;
            }

            if( pidOutputTmp < -250 ) {
              pidOutputTmp = -250;
            }

            pidOutputTmp /= 4;
            pidOutputTmp += 128;
            ledcWrite( 0, pidOutputTmp );
          }
          break;

          default:
            break;
        }

        if( steerConfig.gpioEn != SteerConfig::Gpio::None ) {
          digitalWrite( ( uint8_t )steerConfig.gpioEn, HIGH );
        }
      } else {
        ledcWrite( 0, 0 );
        ledcWrite( 1, 0 );
      }
    }

    {
      static uint8_t loopCounter = 0;

      if( ++loopCounter >= 10 ) {
        loopCounter = 0;

        if( initialisation.outputType != SteerConfig::OutputType::None ) {
          uint8_t data[10] = {0};
          data[0] = 0x7F;
          data[1] = 0xFD;

          {
            int16_t steerAngle = steerSetpoints.actualSteerAngle * 100;
            data[2] = ( uint16_t )steerAngle >> 8;
            data[3] = ( uint16_t )steerAngle;
          }

          {
            uint16_t heading;

            if( initialisation.imuType != SteerConfig::ImuType::None ) {
              heading = ( float )steerImuInclinometerData.heading * 16;
            } else {
              heading = 9999;
            }

            data[4] = heading >> 8;
            data[5] = heading;
          }

          {
            int16_t roll;

            if( initialisation.inclinoType != SteerConfig::InclinoType::None ) {
              roll = steerImuInclinometerData.roll * 16;

              if( steerConfig.invertRoll ) {
                roll = -roll;
              }
            } else {
              roll = 9999;
            }

            data[6] = ( uint16_t )roll >> 8;
            data[7] = ( uint16_t )roll;
          }

          // read inputs
          {
            if( steerConfig.workswitchType != SteerConfig::WorkswitchType::None ) {
              uint16_t value = 0;
              uint16_t threshold = 0;
              uint16_t hysteresis = 0;

              switch( steerConfig.workswitchType ) {
                case SteerConfig::WorkswitchType::Gpio:
                  value =  digitalRead( ( uint8_t )steerConfig.gpioWorkswitch ) ? 1 : 0;
                  threshold = 1;
                  hysteresis = 0;
                  break;

                case SteerConfig::WorkswitchType::RearHitchPosition:
                  value = steerCanData.rearHitchPosition;
                  threshold = steerConfig.canBusHitchThreshold;
                  hysteresis = steerConfig.canBusHitchThresholdHysteresis;
                  break;

                case SteerConfig::WorkswitchType::FrontHitchPosition:
                  value = steerCanData.frontHitchPosition;
                  threshold = steerConfig.canBusHitchThreshold;
                  hysteresis = steerConfig.canBusHitchThresholdHysteresis;
                  break;

                case SteerConfig::WorkswitchType::RearPtoRpm:
                  value = steerCanData.rearPtoRpm;
                  threshold = steerConfig.canBusRpmThreshold;
                  hysteresis = steerConfig.canBusRpmThresholdHysteresis;
                  break;

                case SteerConfig::WorkswitchType::FrontPtoRpm:
                  value = steerCanData.frontPtoRpm;
                  threshold = steerConfig.canBusRpmThreshold;
                  hysteresis = steerConfig.canBusRpmThresholdHysteresis;
                  break;

                case SteerConfig::WorkswitchType::MotorRpm:
                  value = steerCanData.motorRpm;
                  threshold = steerConfig.canBusRpmThreshold;
                  hysteresis = steerConfig.canBusRpmThresholdHysteresis;
                  break;

                default:
                  break;
              }

              static bool workswitchState = false;

              if( value >= threshold ) {
                workswitchState = true;
              }

              if( value < ( threshold - hysteresis ) ) {
                workswitchState = false;
              }

              if( steerConfig.workswitchActiveLow ) {
                workswitchState = ! workswitchState;
              }

              data[8] |= workswitchState ? 1 : 0;
            }

            if( steerConfig.gpioSteerswitch != SteerConfig::Gpio::None ) {
              static time_t lastRisingEdge = 0;
              static bool lastInputState = false;

              static bool steerswitchState = false;

              bool currentState = digitalRead( ( uint8_t )steerConfig.gpioSteerswitch );

              if( currentState != lastInputState ) {
                // rising edge
                if( currentState == true ) {
                  steerswitchState = !steerswitchState;
                  lastRisingEdge = millis();
                }

                // falling edge
                if( currentState == false ) {
                  if( lastRisingEdge + steerConfig.autoRecogniseSteerGpioAsSwitchOrButton < millis() ) {
                    steerswitchState = false;
                  }
                }
              }

              if( steerConfig.steerswitchActiveLow ) {
                steerswitchState = ! steerswitchState;
              }

              data[8] |= steerswitchState ? 2 : 0;
            }
          }

          udpSendFrom.broadcastTo( data, sizeof( data ), initialisation.portSendTo );

        } else {
          if( initialisation.inclinoType !=  SteerConfig::InclinoType::None || initialisation.imuType != SteerConfig::ImuType::None ) {
            uint8_t data[10] = {0};
            data[0] = 0x7F;
            data[1] = 0xEE;

            {
              uint16_t heading;

              if( initialisation.imuType != SteerConfig::ImuType::None ) {
                heading = ( float )steerImuInclinometerData.heading * 16;
              } else {
                heading = 9999;
              }

              data[4] = heading >> 8;
              data[5] = heading;
            }

            {
              uint16_t roll;

              if( initialisation.inclinoType != SteerConfig::InclinoType::None ) {
                roll = steerImuInclinometerData.roll * 16;
              } else {
                roll = 9999;
              }

              data[6] = roll >> 8;
              data[7] = roll;
            }

            udpSendFrom.broadcastTo( data, sizeof( data ), initialisation.portSendTo );
          }
        }
      }
    }

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void initAutosteer() {
  if( steerConfig.aogPortSendFrom != 0 ) {
    initialisation.portSendFrom = steerConfig.aogPortSendFrom;
  }

  if( steerConfig.aogPortListenTo != 0 ) {
    initialisation.portListenTo = steerConfig.aogPortListenTo;
  }

  if( steerConfig.aogPortSendTo != 0 ) {
    initialisation.portSendTo = steerConfig.aogPortSendTo;
  }

  udpSendFrom.listen( initialisation.portSendFrom );

  if( udpLocalPort.listen( initialisation.portListenTo ) ) {
    udpLocalPort.onPacket( []( AsyncUDPPacket packet ) {
      uint8_t* data = packet.data();
      uint16_t pgn = data[1] + ( data[0] << 8 );

      // see pgn.xlsx in https://github.com/farmerbriantee/AgOpenGPS/tree/master/AgOpenGPS_Dev
      switch( pgn ) {
        case 0x7FFE: {
          steerSetpoints.relais = data[2];
          steerSetpoints.speed = ( float )data[3] / 4;
          steerSetpoints.distanceFromLine = data[5] + ( data[4] << 8 );
          steerSetpoints.requestedSteerAngle = ( int16_t )( data[7] + ( data[6] << 8 ) ) / 100;

          steerSetpoints.lastPacketReceived = millis();
        }
        break;

        case 0x7FFC: {
          steerSettings.Kp = ( float )data[2] * 1.0; // read Kp from AgOpenGPS
          steerSettings.Ki = ( float )data[3] * 0.001; // read Ki from AgOpenGPS
          steerSettings.Kd = ( float )data[4] * 1.0; // read Kd from AgOpenGPS
          steerSettings.Ko = ( float )data[5] * 0.1; // read Ko from AgOpenGPS
          steerSettings.wheelAnglePositionZero = ( int8_t )data[6]; //read steering zero offset
          steerSettings.minPWMValue = data[7]; //read the minimum amount of PWM for instant on
          steerSettings.maxIntegralValue = data[8] * 0.1; //
          steerSettings.wheelAngleCountsPerDegree = data[9]; //sent as 10 times the setting displayed in AOG

          steerSettings.lastPacketReceived = millis();
        }
        break;

        case 0x7FF6: {
          steerMachineControl.pedalControl = data[2];
          steerMachineControl.speed = ( float )data[3] / 4;
          steerMachineControl.relais = data[4];
          steerMachineControl.youTurn = data[5];

          steerMachineControl.lastPacketReceived = millis();
        }
        break;

        default:
          break;
      }
    } );
  }

  // if no inclinometer is configured, try to receive the value from the net
  if( initialisation.inclinoType == SteerConfig::InclinoType::None ) {
    if( udpRemotePort.listen( initialisation.portSendTo ) ) {
      udpRemotePort.onPacket( []( AsyncUDPPacket packet ) {
        uint8_t* data = packet.data();
        uint16_t pgn = data[1] + ( data[0] << 8 );

        // see pgn.xlsx in https://github.com/farmerbriantee/AgOpenGPS/tree/master/AgOpenGPS_Dev
        switch( pgn ) {
          case 0x7FFD: {
            uint16_t rollInteger = data[7] + ( data[6 << 8] );

            if( rollInteger != 9999 ) {
              steerSetpoints.receivedRoll = ( float )rollInteger / 16;
            }
          }
          break;

          case 0x7FEE: {
            uint16_t rollInteger = data[7] + ( data[6 << 8] );

            if( rollInteger != 9999 ) {
              steerSetpoints.receivedRoll = ( float )rollInteger / 16;
            }
          }
          break;

          default:
            break;
        }
      } );
    }
  }

  // init output
  {
    if( steerConfig.gpioPwm != SteerConfig::Gpio::None ) {
      pinMode( ( uint8_t )steerConfig.gpioPwm, OUTPUT );
      ledcSetup( 0, 1000/*steerConfig.pwmFrequency*/, 8 );
      ledcAttachPin( ( uint8_t )steerConfig.gpioPwm, 0 );
      ledcWrite( 0, 0 );
    }

    if( steerConfig.gpioDir != SteerConfig::Gpio::None ) {
      pinMode( ( uint8_t )steerConfig.gpioDir, OUTPUT );
      ledcSetup( 0, 1000/*steerConfig.pwmFrequency*/, 8 );
      ledcAttachPin( ( uint8_t )steerConfig.gpioDir, 0 );
      ledcWrite( 0, 0 );
    }

    if( steerConfig.gpioEn != SteerConfig::Gpio::None ) {
      pinMode( ( uint8_t )steerConfig.gpioEn, OUTPUT );
      digitalWrite( ( uint8_t )steerConfig.gpioEn, LOW );
    }

    switch( steerConfig.outputType ) {
      case SteerConfig::OutputType::SteeringMotorIBT2: {
        Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );

        if( steerConfig.gpioPwm != SteerConfig::Gpio::None &&
            steerConfig.gpioDir != SteerConfig::Gpio::None &&
            steerConfig.gpioEn  != SteerConfig::Gpio::None ) {
          labelStatusOutputHandle->value = "Output configured";
          labelStatusOutputHandle->color = ControlColor::Emerald;
          ESPUI.updateControlAsync( labelStatusOutputHandle );

          initialisation.outputType = SteerConfig::OutputType::SteeringMotorIBT2;
        } else {
          {
            labelStatusOutputHandle->value = "GPIOs not correctly defined";
            labelStatusOutputHandle->color = ControlColor::Carrot;
            ESPUI.updateControlAsync( labelStatusOutputHandle );
          }
        }
      }
      break;

      case SteerConfig::OutputType::SteeringMotorCytron: {
        Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );

        if( steerConfig.gpioPwm != SteerConfig::Gpio::None &&
            steerConfig.gpioDir != SteerConfig::Gpio::None ) {
          labelStatusOutputHandle->value = "Output configured";
          labelStatusOutputHandle->color = ControlColor::Emerald;
          ESPUI.updateControlAsync( labelStatusOutputHandle );

          initialisation.outputType = SteerConfig::OutputType::SteeringMotorCytron;
        } else {
          {
            labelStatusOutputHandle->value = "GPIOs not correctly defined";
            labelStatusOutputHandle->color = ControlColor::Carrot;
            ESPUI.updateControlAsync( labelStatusOutputHandle );
          }
        }
      }
      break;

      case SteerConfig::OutputType::HydraulicPwm2Coil: {
        Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );

        if( steerConfig.gpioPwm != SteerConfig::Gpio::None &&
            steerConfig.gpioDir != SteerConfig::Gpio::None ) {
          labelStatusOutputHandle->value = "Output configured";
          labelStatusOutputHandle->color = ControlColor::Emerald;
          ESPUI.updateControlAsync( labelStatusOutputHandle );

          initialisation.outputType = SteerConfig::OutputType::HydraulicPwm2Coil;
        } else {
          {
            labelStatusOutputHandle->value = "GPIOs not correctly defined";
            labelStatusOutputHandle->color = ControlColor::Carrot;
            ESPUI.updateControlAsync( labelStatusOutputHandle );
          }
        }
      }
      break;

      case SteerConfig::OutputType::HydraulicDanfoss: {
        Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );

        if( steerConfig.gpioPwm != SteerConfig::Gpio::None &&
            steerConfig.gpioDir != SteerConfig::Gpio::None ) {
          labelStatusOutputHandle->value = "Output configured";
          labelStatusOutputHandle->color = ControlColor::Emerald;
          ESPUI.updateControlAsync( labelStatusOutputHandle );

          initialisation.outputType = SteerConfig::OutputType::HydraulicDanfoss;
        } else {
          {
            labelStatusOutputHandle->value = "GPIOs not correctly defined";
            labelStatusOutputHandle->color = ControlColor::Carrot;
            ESPUI.updateControlAsync( labelStatusOutputHandle );
          }
        }
      }
      break;

      default:
        break;

    }

  }

  if( steerConfig.gpioWorkswitch != SteerConfig::Gpio::None ) {
    pinMode( ( uint8_t )steerConfig.gpioWorkswitch, INPUT_PULLUP );
  }

  if( steerConfig.gpioSteerswitch != SteerConfig::Gpio::None ) {
    pinMode( ( uint8_t )steerConfig.gpioSteerswitch, INPUT_PULLUP );
  }

  xTaskCreate( autosteerWorker100Hz, "autosteerWorker", 2048, NULL, 3, NULL );
}



