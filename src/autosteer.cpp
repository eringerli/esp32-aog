// MIT License
//
// Copyright (c) 2020 Christian Riggenbach
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

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <stdio.h>
#include <string.h>

#include <AutoPID.h>

#include "CborPackage.hpp"
#include "main.hpp"

#include <sstream> // std::stringstream
#include <string>  // std::string

SteerSettings  steerSettings;
SteerSetpoints steerSetpoints;

AsyncUDP udpSendFrom;
AsyncUDP udpLocalPort;
AsyncUDP udpRemotePort;

double  pidOutput = 0;
AutoPID pid( &( steerSetpoints.actualSteerAngle ),
             &( steerSetpoints.requestedSteerAngle ),
             &( pidOutput ),
             -255,
             255,
             steerConfig.steeringPidKp,
             steerConfig.steeringPidKi,
             steerConfig.steeringPidKd );

CborQueueSelector cborQueueSelector;

constexpr time_t Timeout = 1000;

void
autosteerWorker100Hz( void* z ) {
  constexpr TickType_t xFrequency    = 10;
  TickType_t           xLastWakeTime = xTaskGetTickCount();

  pid.setTimeStep( xFrequency );

  auto queue = xQueueCreate( 16, sizeof( CborPackageBase* ) );
  cborQueueSelector.addQueue( steerConfig.qogChannelIdAutosteerEnable, queue );
  cborQueueSelector.addQueue( steerConfig.qogChannelIdSetpointSteerAngle, queue );

  for( ;; ) {
    time_t timeoutPoint = millis() - Timeout;

    if( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance ) {
      while( uxQueueMessagesWaiting( queue ) ) {
        CborPackageBase* cborPackage = nullptr;

        if( xQueueReceive( queue, &cborPackage, 0 ) == pdTRUE ) {
          switch( cborPackage->type ) {
            case CborPackageBase::Type::Number: {
              if( cborPackage->channelId == steerConfig.qogChannelIdSetpointSteerAngle ) {
                auto numberPackage = static_cast< CborPackageNumber* >( cborPackage );

                steerSetpoints.requestedSteerAngle = numberPackage->number;
                steerSetpoints.lastPacketReceived  = millis();
              }
            } break;
            case CborPackageBase::Type::State: {
              if( cborPackage->channelId == steerConfig.qogChannelIdAutosteerEnable ) {
                auto statePackage = static_cast< CborPackageState* >( cborPackage );

                steerSetpoints.enabled            = statePackage->state;
                steerSetpoints.lastPacketReceived = millis();
              }
            } break;
          }

          delete cborPackage;
        }
      }
    }

    // check for timeout and data from AgOpenGPS
    if( steerSetpoints.lastPacketReceived < timeoutPoint ||
        ( steerConfig.mode == SteerConfig::Mode::AgOpenGps &&
          steerSetpoints.distanceFromLine == 32020 ) ||
        ( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance &&
          steerSetpoints.enabled == false ) /* ||
steerSetpoints.speed < 1*/
    ) {
      if( steerConfig.mode == SteerConfig::Mode::AgOpenGps ) {
        steerSetpoints.enabled = false;
      }

      switch( initialisation.outputType ) {
        case SteerConfig::OutputType::HydraulicDanfoss: {
          ledcWrite( 0, 128 );
          ledcWrite( 1, 0 );
        } break;

        default: {
          ledcWrite( 0, 0 );
          ledcWrite( 1, 0 );
        } break;
      }

      if( steerConfig.gpioEn != SteerConfig::Gpio::None ) {
        digitalWrite( ( uint8_t )steerConfig.gpioEn, HIGH );
      }
    } else {
      if( steerConfig.mode == SteerConfig::Mode::AgOpenGps ) {
        steerSetpoints.enabled = true;
      }

      pid.setGains(
        steerConfig.steeringPidKp, steerConfig.steeringPidKi, steerConfig.steeringPidKd );

      if( steerConfig.steeringPidAutoBangOnFactor ) {
        pid.setBangBang( ( ( double )0xFF / steerSettings.Kp ) *
                           steerConfig.steeringPidAutoBangOnFactor,
                         steerConfig.steeringPidBangOff );
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
          } break;

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
          } break;

          case SteerConfig::OutputType::HydraulicDanfoss: {
            // go from 25% on: max left, 50% on: center, 75% on: right max
            if( pidOutputTmp > 250 ) {
              pidOutputTmp = 250;
            }

            if( pidOutputTmp < -250 ) {
              pidOutputTmp = -250;
            }

            pidOutputTmp /= 4;
            pidOutputTmp += 128;
            ledcWrite( 0, pidOutputTmp );
          } break;

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

    if( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance ) {
    }

    static uint8_t loopCounter = 0;

    if( ++loopCounter >= 10 ) {
      loopCounter = 0;

      if( initialisation.outputType != SteerConfig::OutputType::None ) {
        uint8_t data[10] = { 0 };

        // read inputs
        {
          if( steerConfig.workswitchType != SteerConfig::WorkswitchType::None ) {
            uint16_t value      = 0;
            uint16_t threshold  = 0;
            uint16_t hysteresis = 0;

            switch( steerConfig.workswitchType ) {
              case SteerConfig::WorkswitchType::Gpio:
                value      = digitalRead( ( uint8_t )steerConfig.gpioWorkswitch ) ? 1 : 0;
                threshold  = 1;
                hysteresis = 0;
                break;

              case SteerConfig::WorkswitchType::RearHitchPosition:
                value      = steerCanData.rearHitchPosition;
                threshold  = steerConfig.canBusHitchThreshold;
                hysteresis = steerConfig.canBusHitchThresholdHysteresis;
                break;

              case SteerConfig::WorkswitchType::FrontHitchPosition:
                value      = steerCanData.frontHitchPosition;
                threshold  = steerConfig.canBusHitchThreshold;
                hysteresis = steerConfig.canBusHitchThresholdHysteresis;
                break;

              case SteerConfig::WorkswitchType::RearPtoRpm:
                value      = steerCanData.rearPtoRpm;
                threshold  = steerConfig.canBusRpmThreshold;
                hysteresis = steerConfig.canBusRpmThresholdHysteresis;
                break;

              case SteerConfig::WorkswitchType::FrontPtoRpm:
                value      = steerCanData.frontPtoRpm;
                threshold  = steerConfig.canBusRpmThreshold;
                hysteresis = steerConfig.canBusRpmThresholdHysteresis;
                break;

              case SteerConfig::WorkswitchType::MotorRpm:
                value      = steerCanData.motorRpm;
                threshold  = steerConfig.canBusRpmThreshold;
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
              workswitchState = !workswitchState;
            }

            if( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance ) {
              CborPackageState( steerConfig.qogChannelIdWorkswitch, workswitchState )
                .sendPackage();
            }

            if( steerConfig.mode == SteerConfig::Mode::AgOpenGps ) {
              data[8] |= workswitchState ? 1 : 0;
            }
          }

          if( steerConfig.gpioSteerswitch != SteerConfig::Gpio::None ) {
            static time_t lastRisingEdge = 0;
            static bool   lastInputState = false;

            static bool steerswitchState = false;

            bool currentState = digitalRead( ( uint8_t )steerConfig.gpioSteerswitch );

            if( currentState != lastInputState ) {
              // rising edge
              if( currentState == true ) {
                steerswitchState = !steerswitchState;
                lastRisingEdge   = millis();
              }

              // falling edge
              if( currentState == false ) {
                if( lastRisingEdge + steerConfig.autoRecogniseSteerGpioAsSwitchOrButton <
                    millis() ) {
                  steerswitchState = false;
                }
              }
            }

            if( steerConfig.steerswitchActiveLow ) {
              steerswitchState = !steerswitchState;
            }

            if( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance ) {
              CborPackageState( steerConfig.qogChannelIdSteerswitch, steerswitchState )
                .sendPackage();
            }

            if( steerConfig.mode == SteerConfig::Mode::AgOpenGps ) {
              data[8] |= steerswitchState ? 2 : 0;
            }
          }
        }

        if( steerConfig.mode == SteerConfig::Mode::AgOpenGps ) {
          udpSendFrom.broadcastTo( data, sizeof( data ), initialisation.portSendTo );
        }
      }

      {
        switch( steerConfig.outputType ) {
          case SteerConfig::OutputType::SteeringMotorIBT2: {
            String str;
            str.reserve( 30 );
            str = "IBT2 Motor, SetPoint: ";
            str += ( float )steerSetpoints.requestedSteerAngle;
            str += "°, timeout: ";
            str += ( bool )( steerSetpoints.lastPacketReceived < timeoutPoint );
            str += ", enabled: ";
            str += ( bool )steerSetpoints.enabled;
            labelStatusOutput->value = str;
            labelStatusOutput->color = ControlColor::Emerald;
            ESPUI.updateControlAsync( labelStatusOutput );
          } break;

          case SteerConfig::OutputType::SteeringMotorCytron: {
            String str;
            str.reserve( 30 );
            str = "Cytron Motor, SetPoint: ";
            str += ( float )steerSetpoints.requestedSteerAngle;
            str += "°, timeout: ";
            str += ( bool )( steerSetpoints.lastPacketReceived < timeoutPoint );
            str += ", enabled: ";
            str += ( bool )steerSetpoints.enabled;
            labelStatusOutput->value = str;
            labelStatusOutput->color = ControlColor::Emerald;
            ESPUI.updateControlAsync( labelStatusOutput );
          } break;

          case SteerConfig::OutputType::HydraulicPwm2Coil: {
            String str;
            str.reserve( 30 );
            str = "IBT2 Hydraulic PWM 2 Coil, SetPoint: ";
            str += ( float )steerSetpoints.requestedSteerAngle;
            str += "°, timeout: ";
            str += ( bool )( steerSetpoints.lastPacketReceived < timeoutPoint );
            str += ", enabled: ";
            str += ( bool )steerSetpoints.enabled;
            labelStatusOutput->value = str;
            labelStatusOutput->color = ControlColor::Emerald;
            ESPUI.updateControlAsync( labelStatusOutput );
          } break;

          case SteerConfig::OutputType::HydraulicDanfoss: {
            String str;
            str.reserve( 30 );
            str = "IBT2 Hydraulic Danfoss, SetPoint: ";
            str += ( float )steerSetpoints.requestedSteerAngle;
            str += "°, timeout: ";
            str += ( bool )( steerSetpoints.lastPacketReceived < timeoutPoint );
            str += ", enabled: ";
            str += ( bool )steerSetpoints.enabled;
            labelStatusOutput->value = str;
            labelStatusOutput->color = ControlColor::Emerald;
            ESPUI.updateControlAsync( labelStatusOutput );
          } break;

          default:
            break;
        }
      }
    }

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void
initAutosteer() {
  if( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance ) {
    if( steerConfig.qogPortListenTo != 0 ) {
      initialisation.portListenTo = steerConfig.qogPortListenTo;
    }

    if( steerConfig.qogPortSendTo != 0 ) {
      initialisation.portSendTo = steerConfig.qogPortSendTo;
    }

    if( udpLocalPort.listen( initialisation.portListenTo ) ) {
      udpLocalPort.onPacket( []( AsyncUDPPacket packet ) {
        std::vector< uint8_t > v( packet.data(), packet.data() + packet.length() );
        auto                   begin = v.begin();

        for( ;; ) {
          auto package = create_cbor_package( begin, v.end() );
          if( package != nullptr ) {
            if( cborQueueSelector.isValidChannelId( package->channelId ) ) {
              xQueueSend( cborQueueSelector.getQueue( package->channelId ), &package, 0 );
            } else {
              delete package;
            }
          } else {
            break;
          }
        }
      } );
    }
  }

  // init output
  {
    if( steerConfig.gpioPwm != SteerConfig::Gpio::None ) {
      pinMode( ( uint8_t )steerConfig.gpioPwm, OUTPUT );
      ledcSetup( 0, 1000 /*steerConfig.pwmFrequency*/, 8 );
      ledcAttachPin( ( uint8_t )steerConfig.gpioPwm, 0 );
      ledcWrite( 0, 0 );
    }

    if( steerConfig.gpioDir != SteerConfig::Gpio::None ) {
      pinMode( ( uint8_t )steerConfig.gpioDir, OUTPUT );
      ledcSetup( 0, 1000 /*steerConfig.pwmFrequency*/, 8 );
      ledcAttachPin( ( uint8_t )steerConfig.gpioDir, 1 );
      ledcWrite( 1, 0 );
    }

    if( steerConfig.gpioEn != SteerConfig::Gpio::None ) {
      pinMode( ( uint8_t )steerConfig.gpioEn, OUTPUT );
      digitalWrite( ( uint8_t )steerConfig.gpioEn, LOW );
    }

    switch( steerConfig.outputType ) {
      case SteerConfig::OutputType::SteeringMotorIBT2: {
        if( steerConfig.gpioPwm != SteerConfig::Gpio::None &&
            steerConfig.gpioDir != SteerConfig::Gpio::None &&
            steerConfig.gpioEn != SteerConfig::Gpio::None ) {
          labelStatusOutput->value = "Output configured";
          labelStatusOutput->color = ControlColor::Emerald;
          ESPUI.updateControlAsync( labelStatusOutput );

          initialisation.outputType = SteerConfig::OutputType::SteeringMotorIBT2;
        } else {
          {
            labelStatusOutput->value = "GPIOs not correctly defined";
            labelStatusOutput->color = ControlColor::Carrot;
            ESPUI.updateControlAsync( labelStatusOutput );
          }
        }
      } break;

      case SteerConfig::OutputType::SteeringMotorCytron: {
        if( steerConfig.gpioPwm != SteerConfig::Gpio::None &&
            steerConfig.gpioDir != SteerConfig::Gpio::None ) {
          labelStatusOutput->value = "Output configured";
          labelStatusOutput->color = ControlColor::Emerald;
          ESPUI.updateControlAsync( labelStatusOutput );

          initialisation.outputType = SteerConfig::OutputType::SteeringMotorCytron;
        } else {
          {
            labelStatusOutput->value = "GPIOs not correctly defined";
            labelStatusOutput->color = ControlColor::Carrot;
            ESPUI.updateControlAsync( labelStatusOutput );
          }
        }
      } break;

      case SteerConfig::OutputType::HydraulicPwm2Coil: {
        if( steerConfig.gpioPwm != SteerConfig::Gpio::None &&
            steerConfig.gpioDir != SteerConfig::Gpio::None ) {
          labelStatusOutput->value = "Output configured";
          labelStatusOutput->color = ControlColor::Emerald;
          ESPUI.updateControlAsync( labelStatusOutput );

          initialisation.outputType = SteerConfig::OutputType::HydraulicPwm2Coil;
        } else {
          {
            labelStatusOutput->value = "GPIOs not correctly defined";
            labelStatusOutput->color = ControlColor::Carrot;
            ESPUI.updateControlAsync( labelStatusOutput );
          }
        }
      } break;

      case SteerConfig::OutputType::HydraulicDanfoss: {
        if( steerConfig.gpioPwm != SteerConfig::Gpio::None &&
            steerConfig.gpioDir != SteerConfig::Gpio::None ) {
          labelStatusOutput->value = "Output configured";
          labelStatusOutput->color = ControlColor::Emerald;
          ESPUI.updateControlAsync( labelStatusOutput );

          initialisation.outputType = SteerConfig::OutputType::HydraulicDanfoss;
        } else {
          {
            labelStatusOutput->value = "GPIOs not correctly defined";
            labelStatusOutput->color = ControlColor::Carrot;
            ESPUI.updateControlAsync( labelStatusOutput );
          }
        }
      } break;

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

  xTaskCreate( autosteerWorker100Hz, "autosteerWorker", 3096, NULL, 3, NULL );
}
