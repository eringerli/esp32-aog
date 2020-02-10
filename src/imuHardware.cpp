#include <imu.hpp>
#include <imuHardware.hpp>
#include <main.hpp>
#include <SparkFunLSM9DS1.h>

LSM9DS1 imuHardwareLsm9Ds1;


bool imuHardwareLSM9DS1Init() {
  // reset to make sure data is valid after "warmstart"
  if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
    Wire.beginTransmission(0x1C);  //magnetometer
  	Wire.write(0x21);
  	Wire.write(12);
  	Wire.endTransmission();
    Wire.beginTransmission(0x6A); // accelerometer/gyroscope
  	Wire.write(0x22);
  	Wire.write(1);
  	Wire.endTransmission();
    xSemaphoreGive( i2cMutex );
  }
  delay(10);
  uint lsm9ds1Init = 0;
  if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
    lsm9ds1Init = imuHardwareLsm9Ds1.begin(0x6A, 0x1C);
    usb.print("DEBUG: return code von LSM9DS1 begin(): ");
    usb.println(lsm9ds1Init);
    xSemaphoreGive( i2cMutex );
    if (lsm9ds1Init) {
      imuSettings.hasAccel = true;
      imuSettings.hasGyro = true;
      imuSettings.hasMag = true;
      imuReadData = &imuHardwareLSM9DS1Aquire;
      usb.println("IMU Init - LSM9DS1 successfull");
    } else {
      usb.println("IMU Init - LSM9DS1 failed");
    }
  }

  return lsm9ds1Init != 0;
}

void imuHardwareLSM9DS1Aquire(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* mx, float* my, float* mz) {
  if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
    imuHardwareLsm9Ds1.readMag();
    imuHardwareLsm9Ds1.readGyro();
    imuHardwareLsm9Ds1.readAccel();
    xSemaphoreGive( i2cMutex );
  }

  *gx = imuHardwareLsm9Ds1.calcGyro(imuHardwareLsm9Ds1.gx);
  *gy = imuHardwareLsm9Ds1.calcGyro(imuHardwareLsm9Ds1.gy);
  *gz = imuHardwareLsm9Ds1.calcGyro(imuHardwareLsm9Ds1.gz);

  imuHardwareLsm9Ds1.calcAccel(imuHardwareLsm9Ds1.ax);
  imuHardwareLsm9Ds1.calcAccel(imuHardwareLsm9Ds1.ay);
  imuHardwareLsm9Ds1.calcAccel(imuHardwareLsm9Ds1.az);

  imuHardwareLsm9Ds1.calcMag(imuHardwareLsm9Ds1.mx);
  imuHardwareLsm9Ds1.calcMag(imuHardwareLsm9Ds1.my);
  imuHardwareLsm9Ds1.calcMag(imuHardwareLsm9Ds1.mz);
}
