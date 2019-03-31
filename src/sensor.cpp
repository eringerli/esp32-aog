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

// #include <Wire.h>
// #include <WiredDevice.h>
// #include <RegisterBasedWiredDevice.h>
// #include <Accelerometer.h>
// #include <AccelerometerMMA8451.h>

#include <Adafruit_MMA8451.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Adafruit_ADS1015.h>

#include <ESPUI.h>

#include "main.hpp"

#include "average.hpp"
#include "ringbuffer.hpp"

Adafruit_MMA8451 mma = Adafruit_MMA8451();
Adafruit_BNO055 bno = Adafruit_BNO055( 55 );
Adafruit_ADS1115 ads = Adafruit_ADS1115( 0x48 );

adafruit_bno055_offsets_t bno055CalibrationData;

SteerImuInclinometerData steerImuInclinometerData;

Average<float, float, 20> accXaverage, accYaverage, accZaverage;
float accX, accY, accZ;

volatile uint16_t samplesPerSecond;

Average<float, float, 10> wasAverage;

// http://www.schwietering.com/jayduino/filtuino/index.php?characteristic=bu&passmode=lp&order=2&usesr=usesr&sr=200&frequencyLow=5&noteLow=&noteHigh=&pw=pw&calctype=float&run=Send
//Low pass butterworth filter order=2 alpha1=0.025
class  FilterBuLp2 {
  public:
    FilterBuLp2() {
      v[0] = 0.0;
      v[1] = 0.0;
    }
  private:
    float v[3];
  public:
    float step( float x ) { //class II
      v[0] = v[1];
      v[1] = v[2];
      v[2] = ( 5.542717210280685182e-3 * x )
             + ( -0.80080264666570755150 * v[0] )
             + ( 1.77863177782458481424 * v[1] );
      return
        ( v[0] + v[2] )
        + 2 * v[1];
    }
} accFilterX, accFilterY, accFilterZ;


// http://www.schwietering.com/jayduino/filtuino/index.php?characteristic=bu&passmode=lp&order=2&usesr=usesr&sr=1000&frequencyLow=5&noteLow=&noteHigh=&pw=pw&calctype=float&run=Send
//Low pass butterworth filter order=2 alpha1=0.005
class  FilterBuLp2_2 {
  public:
    FilterBuLp2_2() {
      v[0] = 0.0;
      v[1] = 0.0;
    }
  private:
    float v[3];
  public:
    float step( float x ) { //class II
      v[0] = v[1];
      v[1] = v[2];
      v[2] = ( 2.413590490419614820e-4 * x )
             + ( -0.95654367651120375537 * v[0] )
             + ( 1.95557824031503590945 * v[1] );
      return
        ( v[0] + v[2] )
        + 2 * v[1];
    }
} bnoFilterHeading;

// http://www.schwietering.com/jayduino/filtuino/index.php?characteristic=bu&passmode=lp&order=2&usesr=usesr&sr=100&frequencyLow=5&noteLow=&noteHigh=&pw=pw&calctype=float&run=Send
//Low pass butterworth filter order=2 alpha1=0.05
class  FilterBuLp2_3 {
  public:
    FilterBuLp2_3() {
      v[0] = 0.0;
      v[1] = 0.0;
    }
  private:
    float v[3];
  public:
    float step( float x ) { //class II
      v[0] = v[1];
      v[1] = v[2];
      v[2] = ( 2.008336556421122521e-2 * x )
             + ( -0.64135153805756306422 * v[0] )
             + ( 1.56101807580071816339 * v[1] );
      return
        ( v[0] + v[2] )
        + 2 * v[1];
    }
} wheelAngleSensorFilter;

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets( const adafruit_bno055_offsets_t& calibData ) {
  Serial.print( "Accelerometer: " );
  Serial.print( calibData.accel_offset_x );
  Serial.print( " " );
  Serial.print( calibData.accel_offset_y );
  Serial.print( " " );
  Serial.println( calibData.accel_offset_z );

  Serial.print( "Gyro: " );
  Serial.print( calibData.gyro_offset_x );
  Serial.print( " " );
  Serial.print( calibData.gyro_offset_y );
  Serial.print( " " );
  Serial.println( calibData.gyro_offset_z );

  Serial.print( "Mag: " );
  Serial.print( calibData.mag_offset_x );
  Serial.print( " " );
  Serial.print( calibData.mag_offset_y );
  Serial.print( " " );
  Serial.println( calibData.mag_offset_z );

  Serial.print( "Accel Radius: " );
  Serial.println( calibData.accel_radius );

  Serial.print( "Mag Radius: " );
  Serial.println( calibData.mag_radius );
}

void sensorWorker1HzPoller( void* z ) {
  vTaskDelay( 2000 );
  uint8_t loopCounter = 0;

  constexpr TickType_t xFrequency = 1000;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for ( ;; ) {
    if ( initialisation.imuType == SteerConfig::ImuType::BNO055 ) {

      // Display calibration status for each sensor.
      uint8_t system, gyro, accel, mag = 0;

      if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
        bno.getCalibration( &system, &gyro, &accel, &mag );
        xSemaphoreGive( i2cMutex );
      }

//       Serial.print( "CALIBRATION: Sys=" );
//       Serial.print( system, DEC );
//       Serial.print( " Gyro=" );
//       Serial.print( gyro, DEC );
//       Serial.print( " Accel=" );
//       Serial.print( accel, DEC );
//       Serial.print( " Mag=" );
//       Serial.println( mag, DEC );

      {
        Control* handle = ESPUI.getControl( labelStatusImu );

        if ( system == 3 && gyro == 3 && accel == 3 && mag == 3 ) {
          handle->value = "BNO055 found & calibrated";
          handle->color = ControlColor::Emerald;

          // save to eeprom every 250s
          if ( loopCounter++ > 250 ) {
            loopCounter = 0;

            if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
              bno.getSensorOffsets( bno055CalibrationData );
              xSemaphoreGive( i2cMutex );
            }

            displaySensorOffsets( bno055CalibrationData );

            Serial.println( "Calibration saved to EEPROM" );

            writeEeprom();
          }
        } else {
          String str;
          str.reserve( 40 );
          str += "BNO055 found but not calibrated: S:G:A:M=";
          str += system;
          str += ":";
          str += gyro;
          str += ":";
          str += accel;
          str += ":";
          str += mag;
          handle->value = str;
          handle->color = ControlColor::Carrot;
        }

        ESPUI.updateControl( handle );
      }

//       Serial.print( "Bno calibration Millis: " );
//       Serial.print( millis() );
//       Serial.print( " duration: " );
//       Serial.println( millis() - start );
    }

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void sensorWorker100HzPoller( void* z ) {
  vTaskDelay( 2000 );
  constexpr TickType_t xFrequency = 10;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for ( ;; ) {
    if ( initialisation.imuType == SteerConfig::ImuType::BNO055 ) {

      imu::Vector<3> euler;
//       imu::Quaternion quat;

      if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
//         temp = bno.getTemp();
        euler = bno.getVector( Adafruit_BNO055::VECTOR_EULER );
//         quat = bno.getQuat();
        xSemaphoreGive( i2cMutex );
      }

      float heading = bnoFilterHeading.step( euler.x() );
      heading += ( uint16_t )steerConfig.imuOrientation * 90;

      if ( heading > 360 ) {
        heading -= 360;
      }

      steerImuInclinometerData.bnoAverageHeading += heading;


//       Serial.print( "Current Temperature: " );
//       Serial.println( temp );
//
//       /* Display the floating point data */
// //       Serial.print( "Euler angle X: " );
// //       Serial.print( euler.x() );
// //       Serial.print( "filtered: " );
// //       Serial.print( heading );
// //       Serial.print( "avg: " );
// //       Serial.println( (float)steerImuInclinometerData.bnoAverageHeading );
//       Serial.print( " Y: " );
//       Serial.print( euler.y() );
//       Serial.print( " Z: " );
//       Serial.println( euler.z() );
//
//
//       // Quaternion data
//       Serial.print( "qW: " );
//       Serial.print( quat.w(), 4 );
//       Serial.print( " qX: " );
//       Serial.print( quat.x(), 4 );
//       Serial.print( " qY: " );
//       Serial.print( quat.y(), 4 );
//       Serial.print( " qZ: " );
//       Serial.println( quat.z(), 4 );
//
//       Serial.print( "Bno Millis: " );
//       Serial.println( millis() - start );
    }


    if ( steerConfig.wheelAngleInput != SteerConfig::AnalogIn::None ) {
      float wheelAngleTmp = 0;

      switch ( ( uint8_t )steerConfig.wheelAngleInput ) {
        case ( uint8_t )SteerConfig::AnalogIn::Esp32GpioA2 ...( uint8_t )SteerConfig::AnalogIn::Esp32GpioA12: {
          wheelAngleTmp = analogRead( ( uint8_t )steerConfig.wheelAngleInput );
        }
        break;

        case ( uint8_t )SteerConfig::AnalogIn::ADS1115A0Single ...( uint8_t )SteerConfig::AnalogIn::ADS1115A3Single: {
          if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
            wheelAngleTmp = ads.readADC_SingleEnded(
                              ( uint8_t )steerConfig.wheelAngleInput - ( uint8_t )SteerConfig::AnalogIn::ADS1115A0Single );
            xSemaphoreGive( i2cMutex );
          }
        }
        break;

        case ( uint8_t )SteerConfig::AnalogIn::ADS1115A0A1Differential: {
          if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
            wheelAngleTmp = ads.readADC_Differential_0_1();
            xSemaphoreGive( i2cMutex );
          }
        }
        break;

        case ( uint8_t )SteerConfig::AnalogIn::ADS1115A2A3Differential: {
          if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
            wheelAngleTmp = ads.readADC_Differential_2_3();
            xSemaphoreGive( i2cMutex );
          }
        }
        break;

        default:
          break;
      }

      {
        if ( steerConfig.allowWheelAngleCenterAndCountsOverwrite ) {
          wheelAngleTmp -= steerSettings.wheelAnglePositionZero;
          wheelAngleTmp /= steerSettings.wheelAngleCountsPerDegree;
        } else {
          wheelAngleTmp -= steerConfig.wheelAnglePositionZero;
          wheelAngleTmp /= steerConfig.wheelAngleCountsPerDegree;
        }

        steerSetpoints.wheelAngleRaw = wheelAngleTmp;

        if ( steerConfig.wheelAngleSensorType == SteerConfig::WheelAngleSensorType::TieRodDisplacement ) {
          if ( steerConfig.wheelAngleFirstArmLenght != 0 && steerConfig.wheelAngleSecondArmLenght != 0 &&
               steerConfig.wheelAngleTrackArmLenght != 0 && steerConfig.wheelAngleTieRodStroke != 0 ) {

            auto getDisplacementFromAngle = []( float angle ) {
              // a: 2. arm, b: 1. arm, c: abstand drehpunkt wineklsensor und anschlagpunt 2. arm an der spurstange
              // gegenwinkel: winkel zwischen 1. arm und spurstange
              double alpha = PI - radians( angle );

              // winkel zwischen spurstange und 2. arm
              double gamma = PI - alpha - ( asin( steerConfig.wheelAngleFirstArmLenght * sin( alpha ) / steerConfig.wheelAngleSecondArmLenght ) );

              // auslenkung
              return steerConfig.wheelAngleSecondArmLenght * sin( gamma ) / sin( alpha );
            };

            steerSetpoints.wheelAngleCurrentDisplacement = getDisplacementFromAngle( wheelAngleTmp );

            double relativeDisplacementToStraightAhead =
              // real displacement
              steerSetpoints.wheelAngleCurrentDisplacement -
              // calculate middle of displacement -
              ( getDisplacementFromAngle( steerConfig.wheelAngleMinimumAngle ) + ( steerConfig.wheelAngleTieRodStroke / 2 ) );

            wheelAngleTmp = degrees( asin( relativeDisplacementToStraightAhead / steerConfig.wheelAngleTrackArmLenght ) );
          }
        }

        if ( steerConfig.invertWheelAngleSensor ) {
          wheelAngleTmp *= ( float ) -1;
        }

        wheelAngleTmp -= steerConfig.wheelAngleOffset;

        wheelAngleTmp = wheelAngleSensorFilter.step( wheelAngleTmp );
        steerSetpoints.actualSteerAngle = wheelAngleTmp;
        wasAverage += wheelAngleTmp;
      }

    }

    {
      static uint8_t loopCounter = 0;

      if ( loopCounter++ > 99 ) {
        loopCounter = 0;
        {
          Control* handle = ESPUI.getControl( labelHeading );
          handle->value = ( float )steerImuInclinometerData.heading;
          handle->value += "°";
          ESPUI.updateControl( handle );
        }
        {
          Control* handle = ESPUI.getControl( labelRoll );
          handle->value = ( float )steerImuInclinometerData.roll;
          handle->value += "°";
          ESPUI.updateControl( handle );
        }
        {
          Control* handle = ESPUI.getControl( labelWheelAngle );
          String str;
          str.reserve( 30 );

          if ( steerConfig.wheelAngleSensorType == SteerConfig::WheelAngleSensorType::TieRodDisplacement ) {
            str += ( float )steerSetpoints.actualSteerAngle;
            str += "°, Raw ";
            str += ( float )steerSetpoints.wheelAngleRaw;
            str += "°, Displacement ";
            str += ( float )steerSetpoints.wheelAngleCurrentDisplacement;
            str += "mm";
          } else {
            str += ( float )steerSetpoints.actualSteerAngle;
            str += "°, Raw ";
            str += ( float )steerSetpoints.wheelAngleRaw;
            str += "°";
          }

          handle->value = str;
          ESPUI.updateControl( handle );
        }
      }
    }

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}


void sensorWorker10HzPoller( void* z ) {
  constexpr TickType_t xFrequency = 100;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for ( ;; ) {
    if ( initialisation.inclinoType == SteerConfig::InclinoType::MMA8451 ) {
      sensors_event_t events[32];

      uint8_t numSamples = 0;

      if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
        numSamples = mma.getEventsFromFifo( events );
        xSemaphoreGive( i2cMutex );
      }

      for ( uint8_t i = 0; i < numSamples; i++ ) {
        float x = accFilterX.step( events[i].acceleration.x );
        float y = accFilterY.step( events[i].acceleration.y );
        float z = accFilterZ.step( events[i].acceleration.z );

        accXaverage += x;
        accYaverage += y;
        accZaverage += z;

        accX = x;
        accY = y;
        accZ = z;
      }

      float fXg = accXaverage;
      float fYg = accYaverage;
      float fZg = accZaverage;
      float roll  = ( atan2( -fYg, fZg ) * 180.0 ) / M_PI;
      float pitch = ( atan2( fXg, sqrt( fYg * fYg + fZg * fZg ) ) * 180.0 ) / M_PI;

      switch ( steerConfig.inclinoOrientation ) {
        case SteerConfig::InclinoOrientation::Forwards:
          steerImuInclinometerData.roll = -pitch;
          steerImuInclinometerData.pitch = roll;
          break;

        case SteerConfig::InclinoOrientation::Backwards:
          steerImuInclinometerData.roll = pitch;
          steerImuInclinometerData.pitch = -roll;
          break;

        case SteerConfig::InclinoOrientation::Left:
          steerImuInclinometerData.roll = -roll;
          steerImuInclinometerData.pitch = -pitch;
          break;

        case SteerConfig::InclinoOrientation::Right:
          steerImuInclinometerData.roll = roll;
          steerImuInclinometerData.pitch = pitch;
          break;
      }

      steerImuInclinometerData.roll -= steerConfig.rollOffset;

      {
        static uint8_t loopCounter = 0;

        if ( loopCounter++ > 9 ) {
          loopCounter = 0;
          Control* handle = ESPUI.getControl( labelRoll );
          String str;
          str.reserve( 10 );
          str = String( steerImuInclinometerData.roll ) + "°";
          handle->value = str;
          ESPUI.updateControl( handle );
        }
      }
    }

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void initSensors() {
  Wire.setClock( 400000 );

  if ( steerConfig.inclinoType == SteerConfig::InclinoType::MMA8451 ) {
    Control* handle = ESPUI.getControl( labelStatusInclino );

    if ( mma.begin() ) {
      handle->value = "MMA8451 found & initialized";
      handle->color = ControlColor::Emerald;
      initialisation.inclinoType = SteerConfig::InclinoType::MMA8451;

      mma.setRange( MMA8451_RANGE_2_G );
      mma.setDataRate( MMA8451_DATARATE_200_HZ );
      mma.setFifoSettings( MMA8451_FIFO_CIRCULAR );
    } else {
      handle->value = "MMA8451 not found";
      handle->color = ControlColor::Alizarin;
      initialisation.inclinoType = SteerConfig::InclinoType::None;
    }

    ESPUI.updateControl( handle );
  }

  if ( steerConfig.imuType == SteerConfig::ImuType::BNO055 ) {
    Control* handle = ESPUI.getControl( labelStatusImu );

    if ( bno.begin() ) {
      handle->value = "BNO055 found & initialized";
      handle->color = ControlColor::Emerald;
      initialisation.imuType = SteerConfig::ImuType::BNO055;

      displaySensorOffsets( bno055CalibrationData );

      if ( bno055CalibrationData.mag_radius != 0xffff ) {
        bno.setSensorOffsets( bno055CalibrationData );
      }

      bno.setExtCrystalUse( true );
    } else {
      handle->value = "BNO055 not found";
      handle->color = ControlColor::Alizarin;
      initialisation.imuType = SteerConfig::ImuType::None;
    }

    ESPUI.updateControl( handle );
  }

  // initialise ads1115 everytime, even if not avaible (no answer in the init -> just sending)
  {
    ads.setGain( GAIN_TWOTHIRDS );   // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
    // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
    // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
    // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
    // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
    // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

    ads.begin();
    ads.setSPS( ADS1115_DR_860SPS );

    Control* handle = ESPUI.getControl( labelStatusAdc );
    handle->value = "ADC1115 initialized";
    handle->color = ControlColor::Emerald;
    initialisation.wheelAngleInput = steerConfig.wheelAngleInput;
    ESPUI.updateControl( handle );
  }

  xTaskCreate( sensorWorker1HzPoller, "sensorWorker1HzPoller", 4096, NULL, 1, NULL );
  xTaskCreate( sensorWorker10HzPoller, "sensorWorker10HzPoller", 4096, NULL, 5, NULL );
  xTaskCreate( sensorWorker100HzPoller, "sensorWorker100HzPoller", 4096, NULL, 2, NULL );
}



