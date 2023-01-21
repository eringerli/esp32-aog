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
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <limits.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <Adafruit_Sensor.h>

#include "average.hpp"
#include "jsonFunctions.hpp"
#include "main.hpp"

#include <AutoPID.h>

#include <ESPUI.h>

#include <SPI.h>

#include <ADS131M04.h>

#include <Filters.h>
#include <Filters/Butterworth.hpp>

#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DSOX.h>

#include <Adafruit_SPIDevice.h>

#include <array>

class LSM6DSOX : public Adafruit_LSM6DSOX {
public:
  LSM6DSOX() {}

  void read( float accGyro[6] ) {
    _read();

    accGyro[0] = this->accX;
    accGyro[1] = this->accY;
    accGyro[2] = this->accZ;
    accGyro[3] = this->gyroX;
    accGyro[4] = this->gyroY;
    accGyro[5] = this->gyroZ;
  }
};

class ISM330DHCX : public Adafruit_ISM330DHCX {
public:
  ISM330DHCX() {}

  void read( float accGyro[6] ) {
    _read();

    accGyro[0] = this->accX;
    accGyro[1] = this->accY;
    accGyro[2] = this->accZ;
    accGyro[3] = this->gyroX;
    accGyro[4] = this->gyroY;
    accGyro[5] = this->gyroZ;
  }
};

// SOSFilter<float, 2> filter[4];

// Average<float, float, 10> wasAverage;

// http://www.schwietering.com/jayduino/filtuino/index.php?characteristic=bu&passmode=lp&order=2&usesr=usesr&sr=100&frequencyLow=10&noteLow=&noteHigh=&pw=pw&calctype=float&run=Send

TaskHandle_t  sensorWorkerTask;
TaskHandle_t  networkWorkerTask;
QueueHandle_t networkSendQueue;
void          networkWorker( void* z );

// volatile TickType_t lastAccGyroTick = 0;
// volatile TickType_t lastMagTick     = 0;
// volatile TickType_t lastAdcTick     = 0;

static volatile uint32_t magNotifyValue = 0;

void IRAM_ATTR
adsDrdyInterrupt() {
  //   lastAdcTick = xTaskGetTickCount();

  BaseType_t higherPriorityTaskWoken;

  xTaskNotifyFromISR( sensorWorkerTask,
                      ( uint32_t )( SensorNotifyBits::AdcDataReady ),
                      eSetBits,
                      &higherPriorityTaskWoken );

  // yield if the sensor task has higher priority
  if( higherPriorityTaskWoken ) {
    portYIELD_FROM_ISR();
  }
}

void IRAM_ATTR
accGyroDrdyInterrupt() {
  //   lastAccGyroTick = xTaskGetTickCount();

  BaseType_t higherPriorityTaskWoken;

  xTaskNotifyFromISR( sensorWorkerTask,
                      ( uint32_t )( SensorNotifyBits::AccGyroDataReady ) | magNotifyValue,
                      eSetBits,
                      &higherPriorityTaskWoken );

  magNotifyValue = 0;

  digitalWrite( 32, HIGH );
  digitalWrite( 32, LOW );
  delayMicroseconds( 20 );
  digitalWrite( 32, HIGH );

  // yield if the sensor task has higher priority
  if( higherPriorityTaskWoken ) {
    portYIELD_FROM_ISR();
  }
}

void IRAM_ATTR
magDrdyInterrupt() {
  //   lastMagTick = xTaskGetTickCount();

  magNotifyValue = ( uint32_t )( SensorNotifyBits::MagDataReady );

  digitalWrite( 15, HIGH );
  digitalWrite( 15, LOW );
  delayMicroseconds( 20 );
  digitalWrite( 15, HIGH );
}

constexpr AdcFloatingPoint
sampleRateFromOSR( SteerConfig::Ads131m04SampleRate osr ) {
  return ( AdcFloatingPoint )16000.0 / ( uint8_t )osr;
}

constexpr AdcFloatingPoint
dTFromOSR( SteerConfig::Ads131m04SampleRate osr ) {
  return ( AdcFloatingPoint )( 1 ) / sampleRateFromOSR( osr );
}

AdcFloatingPoint
normalisedCutOffFrequency( AdcFloatingPoint cutOffFrequency ) {
  return ( AdcFloatingPoint )2.0 * cutOffFrequency /
         sampleRateFromOSR( steerConfig.ads131m04SampleRate );
}

SOSFilter< AdcFloatingPoint, 1U >
getFilterButterworth( AdcFloatingPoint cutOffFrequency ) {
  return butter< 2 >( normalisedCutOffFrequency( cutOffFrequency ) );
}

AdcFloatingPoint adcValues[4] = { 0 };

void
sensorWorker( void* z ) {
  pinMode( ( int )steerConfig.gpioAds131m04Drdy, INPUT_PULLUP );
  pinMode( ( int )steerConfig.gpioAccGyroDrdy, INPUT_PULLUP );
  pinMode( ( int )steerConfig.gpioMagDrdy, INPUT_PULLUP );

  LSM6DSOX         lsm6dsox;
  ISM330DHCX       ism330dhcx;
  Adafruit_LIS3MDL lis3mdl;

  ADS131M04 ads131m04( ( uint8_t )steerConfig.gpioAds131m04Cs, steerConfig.spiBusSpeed );

  Serial.println( "ads131m04" );

  {
    ads131m04.begin();

    for( const auto i : { 0, 1, 2, 3 } ) {
      ads131m04.setInputChannelSelection( i, INPUT_CHANNEL_MUX_AIN0P_AIN0N );
    }

    ads131m04.setDrdyFormat( 1 );
    ads131m04.setOsr( ( uint8_t )( steerConfig.ads131m04SampleRate ) );
  }

  Serial.println( "filter" );

  SOSFilter< AdcFloatingPoint, 1U > filter[4] = {
    getFilterButterworth( steerConfig.filterCutOffFrequencies[0] ),
    getFilterButterworth( steerConfig.filterCutOffFrequencies[1] ),
    getFilterButterworth( steerConfig.filterCutOffFrequencies[2] ),
    getFilterButterworth( steerConfig.filterCutOffFrequencies[3] )
  };

  Serial.println( "pid" );

  double  dT = dTFromOSR( steerConfig.ads131m04SampleRate );
  AutoPID pid( -255.0,
               255.0,
               steerConfig.steeringPidKp,
               steerConfig.steeringPidKi,
               steerConfig.steeringPidKd );

  if( steerConfig.steeringPidAutoBangOnFactor ) {
    pid.setBangBang( ( ( double )0xFF / steerSettings.Kp ) *
                       steerConfig.steeringPidAutoBangOnFactor,
                     steerConfig.steeringPidBangOff );
  } else {
    pid.setBangBang( steerConfig.steeringPidBangOn, steerConfig.steeringPidBangOff );
  }

  bool lis3mdlAvaible = lis3mdl.begin_SPI( ( uint8_t )steerConfig.gpioMagCs, &SPI, 10e6 );
  bool lsm6dsoxAvaible =
    lsm6dsox.begin_SPI( ( uint8_t )steerConfig.gpioAccGyroCs, &SPI, 0, 10e6 );
  bool ism330dhcxAvaible =
    ism330dhcx.begin_SPI( ( uint8_t )steerConfig.gpioAccGyroCs, &SPI, 0, 10e6 );

  if( lis3mdlAvaible ) {
    Serial.println( "lis3mdlAvaible" );
    //     lis3mdl.reset();
    //     delay( 50 );
    lis3mdl.setDataRate( LIS3MDL_DATARATE_1000_HZ );

    //     lis3mdl.setDataRate( LIS3MDL_DATARATE_560_HZ );
    lis3mdl.setOperationMode( LIS3MDL_CONTINUOUSMODE );
    //     delay( 5 );
    //     lis3mdl.reset();
    //     delay( 5 );
    //     lis3mdl.setDataRate( LIS3MDL_DATARATE_560_HZ );
    //     delay( 5 );
    //     lis3mdl.reset();
    //     delay( 5 );
    //     lis3mdl.setDataRate( LIS3MDL_DATARATE_560_HZ );
  }

  if( lsm6dsoxAvaible ) {
    Serial.println( "lsm6dsoxAvaible" );
    lsm6dsox.reset();
    delay( 50 );

    lsm6dsox.setAccelDataRate( LSM6DS_RATE_1_66K_HZ );
    lsm6dsox.setAccelRange( LSM6DS_ACCEL_RANGE_2_G );

    lsm6dsox.setGyroDataRate( LSM6DS_RATE_1_66K_HZ );
    lsm6dsox.setGyroRange( LSM6DS_GYRO_RANGE_1000_DPS );

    lsm6dsox.configIntOutputs( true, false );
    lsm6dsox.configInt1( false, true, false, false, false );
  }

  if( ism330dhcxAvaible ) {
    Serial.println( "ism330dhcxAvaible" );
    ism330dhcx.reset();
    delay( 50 );

    ism330dhcx.setAccelDataRate( LSM6DS_RATE_6_66K_HZ );
    ism330dhcx.setAccelRange( LSM6DS_ACCEL_RANGE_2_G );

    ism330dhcx.setGyroDataRate( LSM6DS_RATE_6_66K_HZ );
    ism330dhcx.setGyroRange( LSM6DS_GYRO_RANGE_1000_DPS );

    ism330dhcx.configIntOutputs( false, false );
    ism330dhcx.configInt1( false, true, true, false, false );
  }

  Serial.print( "IMU lis3, ism, dsox: " );
  Serial.print( lis3mdlAvaible );
  Serial.print( ", " );
  Serial.print( ism330dhcxAvaible );
  Serial.print( ", " );
  Serial.println( lsm6dsoxAvaible );

  Serial.println( "interrupts" );
  {
    attachInterrupt( ( int )steerConfig.gpioAds131m04Drdy, adsDrdyInterrupt, FALLING );
    attachInterrupt( ( int )steerConfig.gpioAccGyroDrdy, accGyroDrdyInterrupt, RISING );
    attachInterrupt( ( int )steerConfig.gpioMagDrdy, magDrdyInterrupt, RISING );
  }

  // vTaskDelay( 500 );
  // Serial.println( "sensorWorker 2" );

  if( lis3mdlAvaible ) {
    lis3mdl.read();
    lis3mdl.read();
    lis3mdl.read();
  }

  if( lsm6dsoxAvaible ) {
    float data[6];
    lsm6dsox.read( data );
    lsm6dsox.read( data );
    lsm6dsox.read( data );
  }

  if( ism330dhcxAvaible ) {
    float data[6];
    ism330dhcx.read( data );
    ism330dhcx.read( data );
    ism330dhcx.read( data );
  }

  TickType_t xLastWakeTime = xTaskGetTickCount() + 1000;

  uint32_t adsFreq     = 0;
  uint32_t accGyroFreq = 0;
  uint32_t magFreq     = 0;

  auto imuData = startAccGyrMagTransmission< std::vector< uint8_t > >( 3500 );

  for( ;; ) {
    uint32_t notificationBits;
    if( xTaskNotifyWait( 0, ULONG_MAX, &notificationBits, 5 ) == pdPASS ) {
      ImuDataAccGyrMag accGyrMagData;

      bool accGyroDrdy =
        ( notificationBits & ( uint32_t )( SensorNotifyBits::AccGyroDataReady ) ) ||
        ( digitalRead( ( int )steerConfig.gpioAccGyroDrdy ) );
      bool magDrdy =
        ( notificationBits & ( uint32_t )( SensorNotifyBits::MagDataReady ) ) ||
        ( digitalRead( ( int )steerConfig.gpioMagDrdy ) );

      if( accGyroDrdy ) {
        if( lsm6dsoxAvaible ) {
          ++accGyroFreq;
          lsm6dsox.read( accGyrMagData.acc );
        }
        if( ism330dhcxAvaible ) {
          ++accGyroFreq;
          ism330dhcx.read( accGyrMagData.acc );
        }

        if( magDrdy ) {
          if( lis3mdlAvaible ) {
            ++magFreq;
            lis3mdl.read();

            accGyrMagData.mag[0] = lis3mdl.x_gauss;
            accGyrMagData.mag[1] = lis3mdl.y_gauss;
            accGyrMagData.mag[2] = lis3mdl.z_gauss;

            //             {
            //               addAccGyrMagTransmission( *imuData, accGyrMagData );
            //               if( imuData->size() > 1450 ) {
            //                 xQueueSend( networkSendQueue, &imuData, 0 );
            //                 imuData = startAccGyrMagTransmission< std::vector< uint8_t
            //                 > >( 3500 );
            //               }
            //             }
          }
        } else {
          //           addAccGyrTransmission( *imuData, accGyrMagData );
          //           if( imuData->size() > 1450 ) {
          //             xQueueSend( networkSendQueue, &imuData, 0 );
          //             imuData = startAccGyrMagTransmission< std::vector< uint8_t > >(
          //             3500 );
          //           }
        }
      }

      if( notificationBits &
          ( uint32_t )( SensorNotifyBits::RefreshFiltersSamplerate ) ) {
        Serial.print( "RefreshFiltersSamplerate: " );
        ads131m04.setOsr( ( uint8_t )( steerConfig.ads131m04SampleRate ) );
        Serial.print( ( uint8_t )( steerConfig.ads131m04SampleRate ) );
        Serial.print( ", " );
        dT = dTFromOSR( steerConfig.ads131m04SampleRate );
        Serial.println( dT );

        for( const auto i : { 0, 1, 2, 3 } ) {
          filter[i] = getFilterButterworth( steerConfig.filterCutOffFrequencies[i] );
        }
      }

      if( notificationBits & ( uint32_t )( SensorNotifyBits::RefreshPidValues ) ) {
        Serial.println( "RefreshPidValues" );
        pid.setGains( steerConfig.steeringPidKp,
                      steerConfig.steeringPidKi,
                      steerConfig.steeringPidKd );

        if( steerConfig.steeringPidAutoBangOnFactor ) {
          pid.setBangBang( ( ( double )0xFF / steerSettings.Kp ) *
                             steerConfig.steeringPidAutoBangOnFactor,
                           steerConfig.steeringPidBangOff );
        } else {
          pid.setBangBang( steerConfig.steeringPidBangOn,
                           steerConfig.steeringPidBangOff );
        }
      }

      if( /*false &&*/
          notificationBits & ( uint32_t )( SensorNotifyBits::AdcDataReady ) ) {
        ++adsFreq;
        // auto time1 = micros();

        //         auto adc = ads131m04.readADC();
        //         //         Serial.print( "adc: " );
        //         //         Serial.print( adc.ch[ 0 ] );
        //         //         Serial.print( ", " );
        //         //         Serial.print( adc.ch[ 1 ] );
        //         //         Serial.print( ", " );
        //         //         Serial.print( adc.ch[ 2 ] );
        //         //         Serial.print( ", " );
        //         //         Serial.println( adc.ch[ 3 ] );
        //         // auto time2 = micros();
        //
        //         for( const auto i : { 0, 1, 2, 3 } ) {
        //           adcValues[i] = filter[i]( adc.ch[i] );
        //         }
        // auto time3 = micros();

        if( initialisation.outputType != SteerConfig::OutputType::None ) {
          auto pwm = pid.run( dT,
                              adcValues[steerConfig.wheelAngleInput],
                              steerSetpoints.requestedSteerAngle );

          if( /*steerSetpoints.lastPacketReceived < timeoutPoint ||*/
              ( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance &&
                steerSetpoints.enabled == false ) ) {
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
            if( ( ( pwm < 0 ) && pwm < -steerConfig.steeringPidPwmThreshold ) ||
                ( ( pwm > 0 ) && pwm > steerConfig.steeringPidPwmThreshold ) ) {
              if( steerConfig.invertOutput ) {
                pwm = -pwm;
              }

              if( pwm < 0 && pwm > -steerConfig.steeringPidMinPwm ) {
                pwm = -steerConfig.steeringPidMinPwm;
              }

              if( pwm > 0 && pwm < steerConfig.steeringPidMinPwm ) {
                pwm = steerConfig.steeringPidMinPwm;
              }

              switch( initialisation.outputType ) {
                case SteerConfig::OutputType::SteeringMotorIBT2:
                case SteerConfig::OutputType::HydraulicPwm2Coil: {
                  if( pwm >= 0 ) {
                    ledcWrite( 0, pwm );
                    ledcWrite( 1, 0 );
                  }

                  if( pwm < 0 ) {
                    ledcWrite( 0, 0 );
                    ledcWrite( 1, -pwm );
                  }
                } break;

                case SteerConfig::OutputType::SteeringMotorCytron: {
                  if( pwm >= 0 ) {
                    ledcWrite( 1, 255 );
                  } else {
                    ledcWrite( 0, 255 );
                    pwm = -pwm;
                  }

                  ledcWrite( 0, pwm );

                  if( steerConfig.gpioEn != SteerConfig::Gpio::None ) {
                    digitalWrite( ( uint8_t )steerConfig.gpioEn, HIGH );
                  }
                } break;

                case SteerConfig::OutputType::HydraulicDanfoss: {
                  // go from 25% on: max left, 50% on: center, 75% on: right max
                  if( pwm > 250 ) {
                    pwm = 250;
                  }

                  if( pwm < -250 ) {
                    pwm = -250;
                  }

                  pwm /= 4;
                  pwm += 128;
                  ledcWrite( 0, pwm );
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

              if( steerConfig.gpioEn != SteerConfig::Gpio::None ) {
                digitalWrite( ( uint8_t )steerConfig.gpioEn, LOW );
              }
            }
          }
        }

        //         pwm;
        // auto time4 = micros();

        // Serial.print( "adc: " );
        // Serial.print( pwm );
        // Serial.print( ", " );
        // Serial.print( time2 - time1 );
        // Serial.print( ", " );
        // Serial.print( time3 - time2 );
        // Serial.print( ", " );
        // Serial.print( time4 - time3 );
        // Serial.print( ", " );
        // Serial.println( time4 - time1 );
      }
    }

    //     if( true ) {
    //       static auto lastTick = xTaskGetTickCount();
    //       auto        tick     = xTaskGetTickCount();
    //
    //       if( lastTick < ( tick ) ) {
    //         lastTick = tick;
    //         tick -= 5;
    //         if( lastMagTick < tick ) {
    //           lastMagTick = lastTick;
    //                     Serial.println( "lastMagTick < tick" );
    //           magNotifyValue = ( uint32_t )( SensorNotifyBits::MagDataReady );
    //         }
    //         if( lastAccGyroTick < tick ) {
    //           lastAccGyroTick = lastTick;
    //                     Serial.println( "lastAccGyroTick < tick" );
    //           xTaskNotify( sensorWorkerTask,
    //                        ( uint32_t )( SensorNotifyBits::AccGyroDataReady ) |
    //                          magNotifyValue,
    //                        eSetBits );
    //           magNotifyValue = 0;
    //         }
    //         if( lastAdcTick < tick ) {
    //           lastAdcTick = lastTick;
    //                     Serial.println( "lastAdcTick < tick " );
    //           xTaskNotify(
    //             sensorWorkerTask, ( uint32_t )( SensorNotifyBits::AdcDataReady ),
    //             eSetBits );
    //         }
    //       }
    //     }

    if( xTaskGetTickCount() > xLastWakeTime ) {
      xLastWakeTime = xTaskGetTickCount() + 1000;

      Serial.print( "ads,mag,accGyro,watermark: " );
      Serial.print( adsFreq );
      Serial.print( ", " );
      Serial.print( magFreq );
      Serial.print( ", " );
      Serial.print( accGyroFreq );
      Serial.print( ", " );
      Serial.println( uxTaskGetStackHighWaterMark( sensorWorkerTask ) );

      if( adsFreq < 20 ) {
        Serial.print( "adsFreq < 20: " );
        ads131m04.reset();

        //         ads131m04.readADC();
        //         ads131m04.readADC();
        //         ads131m04.readADC();
        //         ads131m04.readADC();
        //         ads131m04.readADC();
        //         ads131m04.readADC();
        //         ads131m04.readADC();
        //         ads131m04.readADC();
        //         ads131m04.readADC();
        //         ads131m04.readADC();
        //         ads131m04.readADC();
        //         ads131m04.readADC();
      }

      if( magFreq < 5 ) {
        if( lis3mdlAvaible ) {
          Serial.println( "lis3mdlAvaible && magFreq < 5" );
          lis3mdl.reset();
          lis3mdl.setDataRate( LIS3MDL_DATARATE_560_HZ );
          lis3mdl.setOperationMode( LIS3MDL_CONTINUOUSMODE );
          lis3mdl.setRange( LIS3MDL_RANGE_4_GAUSS );

          //         //
          //         lis3mdl.reset();
          // //         delay(30);
          //         lis3mdl.setDataRate( LIS3MDL_DATARATE_560_HZ );
          //
          lis3mdl.read();
          //         lis3mdl.read();
          //         lis3mdl.read();
          //         lis3mdl.read();
          //         lis3mdl.read();
          //         lis3mdl.read();
        } else {
          lis3mdlAvaible =
            lis3mdl.begin_SPI( ( uint8_t )steerConfig.gpioMagCs, &SPI, 10e6 );
        }
      }

      if( lsm6dsoxAvaible && accGyroFreq < 5 ) {
        Serial.println( "lsm6dsoxAvaible && accGyroFreq < 5" );

        lsm6dsox.reset();
        lsm6dsox.setAccelDataRate( LSM6DS_RATE_1_66K_HZ );
        lsm6dsox.setAccelRange( LSM6DS_ACCEL_RANGE_2_G );

        lsm6dsox.setGyroDataRate( LSM6DS_RATE_1_66K_HZ );
        lsm6dsox.setGyroRange( LSM6DS_GYRO_RANGE_1000_DPS );

        lsm6dsox.configIntOutputs( true, false );
        lsm6dsox.configInt1( false, true, false, false, false );

        float data[6];
        lsm6dsox.read( data );
        lsm6dsox.read( data );
        lsm6dsox.read( data );
        //         lsm6dsox.read( data );
        //         lsm6dsox.read( data );
        //         lsm6dsox.read( data );
      }
      if( accGyroFreq < 5 ) {
        if( ism330dhcxAvaible ) {
          Serial.println( "ism330dhcxAvaible && accGyroFreq < 5" );

          ism330dhcx.reset();
          ism330dhcx.setAccelDataRate( LSM6DS_RATE_1_66K_HZ );
          ism330dhcx.setAccelRange( LSM6DS_ACCEL_RANGE_2_G );

          ism330dhcx.setGyroDataRate( LSM6DS_RATE_1_66K_HZ );
          ism330dhcx.setGyroRange( LSM6DS_GYRO_RANGE_1000_DPS );

          ism330dhcx.configIntOutputs( true, false );
          ism330dhcx.configInt1( false, true, false, false, false );

          float data[6];
          ism330dhcx.read( data );
          ism330dhcx.read( data );
          ism330dhcx.read( data );
          //         ism330dhcx.read( data );
          //         ism330dhcx.read( data );
        } else {
          ism330dhcxAvaible =
            ism330dhcx.begin_SPI( ( uint8_t )steerConfig.gpioAccGyroCs, &SPI, 0, 10e6 );
          ism330dhcx.reset();
          ism330dhcx.setAccelDataRate( LSM6DS_RATE_1_66K_HZ );
          ism330dhcx.setAccelRange( LSM6DS_ACCEL_RANGE_2_G );

          ism330dhcx.setGyroDataRate( LSM6DS_RATE_1_66K_HZ );
          ism330dhcx.setGyroRange( LSM6DS_GYRO_RANGE_1000_DPS );

          ism330dhcx.configIntOutputs( true, false );
          ism330dhcx.configInt1( false, true, false, false, false );
        }
      }

      //       lsm6dsoxAvaible =
      //         lsm6dsox.begin_SPI( ( uint8_t )steerConfig.gpioAccGyroCs, &SPI, 0, 10e6
      //         );

      adsFreq     = 0;
      magFreq     = 0;
      accGyroFreq = 0;
    }
  }
}

void
networkWorker( void* z ) {
  std::vector< uint8_t >* buffer = nullptr;
  while( xQueueReceive( networkSendQueue, &buffer, portMAX_DELAY ) == pdPASS ) {
    sendAccGyrMagTransmission( *buffer );
    delete buffer;
  }
}

void
initSensors() {
  networkSendQueue = xQueueCreate( 16, sizeof( std::vector< uint8_t >* ) );

  xTaskCreatePinnedToCore( sensorWorker,
                           "sensorWorker",
                           4096,
                           NULL,
                           ( configMAX_PRIORITIES - 1 ),
                           &sensorWorkerTask,
                           1 );

  xTaskCreatePinnedToCore(
    networkWorker, "networkWorker", 1024, NULL, 10, &networkWorkerTask, 0 );
}
