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

  for ( ;; ) {
    time_t timeoutPoint = millis() - Timeout;

    // check for timeout and data from AgOpenGPS
    if ( steerSetpoints.lastPacketReceived < timeoutPoint ||
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

      switch ( initialisation.outputType ) {
        case SteerConfig::OutputType::SteeringMotorIBT2: {
          digitalWrite( ( uint8_t )steerConfig.gpioEn, LOW );
          ledcWrite( 0, 0 );
          ledcWrite( 1, 0 );
        }
        break;
      }

    } else {
      steerSetpoints.enabled = true;
//       Serial.println( "Autosteer enabled" );

      // use AOG-values
      if ( steerConfig.allowPidOverwrite == true ) {

        static uint8_t loopCounter = 0;

        if ( ++loopCounter >= 10 ) {
          loopCounter = 0;

          //close enough to center, 4 cm, remove any correction
          if ( steerSetpoints.distanceFromLine <= 40 && steerSetpoints.distanceFromLine >= -40 ) {
            steerSetpoints.correction = 0;
          } else {
            //use the integal value to adjust how much per cycle it increases
            steerSetpoints.correction += steerSettings.Ki;

            //provide a limit - the old max integral value
            if ( steerSetpoints.correction > steerSettings.maxIntegralValue ) {
              steerSetpoints.correction = steerSettings.maxIntegralValue;
            }

            //now add the correction to fool steering position
            if ( steerSetpoints.distanceFromLine > 40 ) {
              steerSetpoints.requestedSteerAngle -= steerSetpoints.correction;
            } else {
              steerSetpoints.requestedSteerAngle += steerSetpoints.correction;
            }
          }


          float steerAngleError = steerSetpoints.actualSteerAngle - steerSetpoints.requestedSteerAngle;
          float driveValue = steerSettings.Kp * steerAngleError * steerSettings.Ko * ( steerConfig.invertOutput ? -1 : 1 );

//           Serial.print( "Kp,Ko,Ki,maxIntegralValue: " );
//           Serial.print( steerSettings.Kp );
//           Serial.print( "," );
//           Serial.print( steerSettings.Ko );
//           Serial.print( "," );
//           Serial.print( steerSettings.Ki );
//           Serial.print( "," );
//           Serial.print( steerSettings.maxIntegralValue );
//           Serial.print( "," );
// 
//           Serial.print( "steerAngleError: " );
//           Serial.print( steerAngleError );
//           Serial.print( ", driveValue: " );
//           Serial.print( driveValue );

          if ( driveValue > 0xFF ) {
            driveValue = 0xFF;
          }

          if ( driveValue < -0xFF ) {
            driveValue = -0xFF;
          }

//           Serial.print( ", " );
//           Serial.print( driveValue );

          if ( driveValue < 0 && driveValue > -steerSettings.minPWMValue ) {
            driveValue = -steerSettings.minPWMValue;
          }

          if ( driveValue > 0 && driveValue < steerSettings.minPWMValue ) {
            driveValue = steerSettings.minPWMValue;
          }


//           Serial.print( ", " );
//           Serial.println( driveValue );


//           switch ( initialisation.outputType ) {
//             case SteerConfig::OutputType::SteeringMotorIBT2: {
//               if ( driveValue > 0 ) {
//                 Serial.println( "driveValue > 0" );
//
//                 ledcWrite( 0, driveValue );
//                 ledcWrite( 1, 0 );
//               }
//
//               if ( driveValue == 0 ) {
//                 Serial.println( "driveValue == 0" );
//                 ledcWrite( 0, 0 );
//                 ledcWrite( 1, 0 );
//               }
//
//               if ( driveValue < 0 ) {
//                 Serial.println( "driveValue < 0" );
//                 ledcWrite( 0, 0 );
//                 ledcWrite( 1, -driveValue );
//               }
//
//               digitalWrite( ( uint8_t )steerConfig.gpioEn, HIGH );
//             }
//             break;
//           }
        }

        // our own PID schema
      } else {
        pid.setGains( steerConfig.steeringPidKp, steerConfig.steeringPidKi, steerConfig.steeringPidKd );

        if ( steerConfig.steeringPidAutoBangOnFactor ) {
          pid.setBangBang( ( ( double )0xFF / steerSettings.Kp ) * steerConfig.steeringPidAutoBangOnFactor, steerConfig.steeringPidBangOff );
        } else {
          pid.setBangBang( steerConfig.steeringPidBangOn, steerConfig.steeringPidBangOff );
        }

        pid.run();

//         Serial.print( "actualSteerAngle: " );
//         Serial.print( steerSetpoints.actualSteerAngle );
//         Serial.print( ", requestedSteerAngle: " );
//         Serial.print( steerSetpoints.requestedSteerAngle );
//         Serial.print( "pidOutput: " );
//         Serial.println( pidOutput );

        if ( pidOutput ) {

          double pidOutputTmp = steerConfig.invertOutput ? pidOutput : -pidOutput;

          if ( pidOutputTmp < 0 && pidOutputTmp > -steerConfig.steeringPidMinPwm ) {
            pidOutputTmp = -steerConfig.steeringPidMinPwm;
          }

          if ( pidOutputTmp > 0 && pidOutputTmp < steerConfig.steeringPidMinPwm ) {
            pidOutputTmp = steerConfig.steeringPidMinPwm;
          }

          switch ( initialisation.outputType ) {
            case SteerConfig::OutputType::SteeringMotorIBT2: {
              if ( pidOutputTmp >= 0 ) {
//                 Serial.println( "driveValue > 0" );

                ledcWrite( 0, pidOutputTmp );
                ledcWrite( 1, 0 );
              }

              if ( pidOutputTmp < 0 ) {
//                 Serial.println( "driveValue < 0" );
                ledcWrite( 0, 0 );
                ledcWrite( 1, -pidOutputTmp );
              }

              digitalWrite( ( uint8_t )steerConfig.gpioEn, HIGH );
            }
            break;

            default:
              break;
          }
        } else {
          ledcWrite( 0, 0 );
          ledcWrite( 1, 0 );
        }
      }
    }

    {
      static uint8_t loopCounter = 0;

      if ( ++loopCounter >= 10 ) {
        loopCounter = 0;

        if ( initialisation.outputType != SteerConfig::OutputType::None ) {
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

            if ( initialisation.imuType != SteerConfig::ImuType::None ) {
              heading = ( float )steerImuInclinometerData.bnoAverageHeading * 16;
            } else {
              heading = 9999;
            }

            data[4] = heading >> 8;
            data[5] = heading;
          }

          {
            int16_t roll;

            if ( initialisation.inclinoType != SteerConfig::InclinoType::None ) {
              roll = steerImuInclinometerData.roll * 16;
            } else {
              roll = 9999;
            }

            data[6] = ( uint16_t )roll >> 8;
            data[7] = ( uint16_t )roll;
          }

          // TODO read inputs
          {
            if ( steerConfig.gpioWorkswitch != SteerConfig::Gpio::None ) {
              data[8] |= digitalRead( ( uint8_t )steerConfig.gpioWorkswitch ) ? 1 : 0;
            }

            if ( steerConfig.gpioSteerswitch != SteerConfig::Gpio::None ) {
              data[8] |= digitalRead( ( uint8_t )steerConfig.gpioSteerswitch ) ? 2 : 0;
            }
          }

          udpSendFrom.broadcastTo( data, sizeof( data ), initialisation.portSendTo );

        } else {
          if ( initialisation.inclinoType !=  SteerConfig::InclinoType::None || initialisation.imuType != SteerConfig::ImuType::None ) {
            uint8_t data[10] = {0};
            data[0] = 0x7F;
            data[1] = 0xEE;

            {
              uint16_t heading;

              if ( initialisation.imuType != SteerConfig::ImuType::None ) {
                heading = ( float )steerImuInclinometerData.bnoAverageHeading * 16;
              } else {
                heading = 9999;
              }

              data[4] = heading >> 8;
              data[5] = heading;
            }

            {
              uint16_t roll;

              if ( initialisation.inclinoType != SteerConfig::InclinoType::None ) {
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
  if ( steerConfig.portSendFrom != 0 ) {
    initialisation.portSendFrom = steerConfig.portSendFrom;
  }

  if ( steerConfig.portListenTo != 0 ) {
    initialisation.portListenTo = steerConfig.portListenTo;
  }

  if ( steerConfig.portSendTo != 0 ) {
    initialisation.portSendTo = steerConfig.portSendTo;
  }

  udpSendFrom.listen( initialisation.portSendFrom );

  if ( udpLocalPort.listen( initialisation.portListenTo ) ) {
    udpLocalPort.onPacket( []( AsyncUDPPacket packet ) {
      uint8_t* data = packet.data();
      uint16_t pgn = data[1] + ( data[0] << 8 );

      // see pgn.xlsx in https://github.com/farmerbriantee/AgOpenGPS/tree/master/AgOpenGPS_Dev
      switch ( pgn ) {
        case 0x7FFE: {
//           Serial.println( "Autosteer 0x7FFE" );
          steerSetpoints.relais = data[2];
          steerSetpoints.speed = ( float )data[3] / 4;
          steerSetpoints.distanceFromLine = data[5] + ( data[4] << 8 );
          steerSetpoints.requestedSteerAngle = ( int16_t )( data[7] + ( data[6] << 8 ) ) / 100;

          steerSetpoints.lastPacketReceived = millis();
        }
        break;

        case 0x7FFC: {
//           Serial.println( "Autosteer 0x7FFC" );
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
//           Serial.println( "Autosteer 0x7FF6" );
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
  if ( initialisation.inclinoType == SteerConfig::InclinoType::None ) {
    if ( udpRemotePort.listen( initialisation.portSendTo ) ) {
      udpRemotePort.onPacket( []( AsyncUDPPacket packet ) {
        uint8_t* data = packet.data();
        uint16_t pgn = data[1] + ( data[0] << 8 );

        // see pgn.xlsx in https://github.com/farmerbriantee/AgOpenGPS/tree/master/AgOpenGPS_Dev
        switch ( pgn ) {
          case 0x7FFD: {
            uint16_t rollInteger = data[7] + ( data[6 << 8] );

            if ( rollInteger != 9999 ) {
              steerSetpoints.receivedRoll = ( float )rollInteger / 16;
            }
          }
          break;

          case 0x7FEE: {
            uint16_t rollInteger = data[7] + ( data[6 << 8] );

            if ( rollInteger != 9999 ) {
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
    switch ( steerConfig.outputType ) {
      case SteerConfig::OutputType::SteeringMotorIBT2: {
        Control* labelStatusOutputHandle = ESPUI.getControl( labelStatusOutput );

        if ( steerConfig.gpioPwm != SteerConfig::Gpio::None &&
             steerConfig.gpioDir != SteerConfig::Gpio::None &&
             steerConfig.gpioEn  != SteerConfig::Gpio::None ) {
          pinMode( ( uint8_t )steerConfig.gpioPwm, OUTPUT );
          pinMode( ( uint8_t )steerConfig.gpioDir, OUTPUT );
          pinMode( ( uint8_t )steerConfig.gpioEn, OUTPUT );
          digitalWrite( ( uint8_t )steerConfig.gpioEn, LOW );

          ledcSetup( 0, 1000/*steerConfig.pwmFrequency*/, 8 ); // PWM Output with channel 0, 1kHz, 8-bit resolution (0-255)
          ledcSetup( 1, 1000/*steerConfig.pwmFrequency*/, 8 ); // PWM Output with channel 1, 1kHz, 8-bit resolution (0-255)
          ledcAttachPin( ( uint8_t )steerConfig.gpioPwm, 0 ); // attach PWM PIN to Channel 0
          ledcAttachPin( ( uint8_t )steerConfig.gpioDir, 1 ); // attach PWM PIN to Channel 1
          ledcWrite( 0, 0 );
          ledcWrite( 1, 0 );

          {
            labelStatusOutputHandle->value = String( "Output configured" );
            labelStatusOutputHandle->color = ControlColor::Emerald;
            ESPUI.updateControl( labelStatusOutputHandle );
          }

          initialisation.outputType = SteerConfig::OutputType::SteeringMotorIBT2;
        } else {
          {
            labelStatusOutputHandle->value = String( "GPIOs not correctly defined" );
            labelStatusOutputHandle->color = ControlColor::Carrot;
            ESPUI.updateControl( labelStatusOutputHandle );
          }
        }


      }
      break;

    }

//     initialisation.outputType = steerConfig.outputType;
  }

  if ( steerConfig.gpioWorkswitch != SteerConfig::Gpio::None ) {
    pinMode( ( uint8_t )steerConfig.gpioWorkswitch, INPUT_PULLUP );
  }

  if ( steerConfig.gpioSteerswitch != SteerConfig::Gpio::None ) {
    pinMode( ( uint8_t )steerConfig.gpioSteerswitch, INPUT_PULLUP );
  }

  if ( steerConfig.gpioWheelencoderA != SteerConfig::Gpio::None &&
       steerConfig.gpioWheelencoderB != SteerConfig::Gpio::None ) {

  }

  // 10ms -> 100Hz
  pid.setTimeStep( 10 );
//   if ( initialisation.outputType != SteerConfig::OutputType::None ) {
  xTaskCreate( autosteerWorker100Hz, "autosteerWorker", 4096, NULL, 3, NULL );
//   }
}



