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
  // configuration
  // Use either IMU_MODE_I2C or IMU_MODE_SPI
  imuHardwareLsm9Ds1.settings.device.commInterface = IMU_MODE_I2C;
  // [mAddress] sets the I2C address of the LSM9DS1's magnetometer.
  imuHardwareLsm9Ds1.settings.device.mAddress = 0x1C;
  // [agAddress] sets the I2C address of the LSM9DS1's accelerometer/gyroscope.
  imuHardwareLsm9Ds1.settings.device.agAddress = 0x6A;
  // [enabled] turns the gyro on or off.
  imuHardwareLsm9Ds1.settings.gyro.enabled = true;  // Enable the gyro
  // [scale] sets the full-scale range of the gyroscope.
  // scale can be set to either 245, 500, or 2000
  imuHardwareLsm9Ds1.settings.gyro.scale = 245; // Set scale to +/-245dps
  // [sampleRate] sets the output data rate (ODR) of the gyro
  // sampleRate can be set between 1-6
  // 1 = 14.9    4 = 238
  // 2 = 59.5    5 = 476
  // 3 = 119     6 = 952
  imuHardwareLsm9Ds1.settings.gyro.sampleRate = 3;
  // [bandwidth] can set the cutoff frequency of the gyro.
  // Allowed values: 0-3. Actual value of cutoff frequency
  // depends on the sample rate. (Datasheet section 7.12)
  imuHardwareLsm9Ds1.settings.gyro.bandwidth = 0;
  // [lowPowerEnable] turns low-power mode on or off.
  imuHardwareLsm9Ds1.settings.gyro.lowPowerEnable = false; // LP mode off
  // [enabled] turns the acclerometer on or off.
  imuHardwareLsm9Ds1.settings.accel.enabled = true; // Enable accelerometer
  // [enableX], [enableY], and [enableZ] can turn on or off
  // select axes of the acclerometer.
  imuHardwareLsm9Ds1.settings.accel.enableX = true; // Enable X
  imuHardwareLsm9Ds1.settings.accel.enableY = true; // Enable Y
  imuHardwareLsm9Ds1.settings.accel.enableZ = true; // Enable Z
  // [scale] sets the full-scale range of the accelerometer.
  // accel scale can be 2, 4, 8, or 16
  imuHardwareLsm9Ds1.settings.accel.scale = 2;
  // [sampleRate] sets the output data rate (ODR) of the
  // accelerometer. ONLY APPLICABLE WHEN THE GYROSCOPE IS
  // DISABLED! Otherwise accel sample rate = gyro sample rate.
  // accel sample rate can be 1-6
  // 1 = 10 Hz    4 = 238 Hz
  // 2 = 50 Hz    5 = 476 Hz
  // 3 = 119 Hz   6 = 952 Hz
  imuHardwareLsm9Ds1.settings.accel.sampleRate = 3;
  // [enabled] turns the magnetometer on or off.
  imuHardwareLsm9Ds1.settings.mag.enabled = true; // Enable magnetometer
  // [scale] sets the full-scale range of the magnetometer
  // mag scale can be 4, 8, 12, or 16
  imuHardwareLsm9Ds1.settings.mag.scale = 8;
  // [sampleRate] sets the output data rate (ODR) of the
  // magnetometer.
  // mag data rate can be 0-7:
  // 0 = 0.625 Hz  4 = 10 Hz
  // 1 = 1.25 Hz   5 = 20 Hz
  // 2 = 2.5 Hz    6 = 40 Hz
  // 3 = 5 Hz      7 = 80 Hz
  imuHardwareLsm9Ds1.settings.mag.sampleRate = 7;
  // [enabled] turns the temperature sensor on or off.
  imuHardwareLsm9Ds1.settings.temp.enabled = false;
  // [tempCompensationEnable] enables or disables
  // temperature compensation of the magnetometer.
  imuHardwareLsm9Ds1.settings.mag.tempCompensationEnable = true;
  // [XYPerformance] sets the x and y-axis performance of the
  // magnetometer to either:
  // 0 = Low power mode      2 = high performance
  // 1 = medium performance  3 = ultra-high performance
  imuHardwareLsm9Ds1.settings.mag.XYPerformance = 3; // Ultra-high perform.
  // [ZPerformance] does the same thing, but only for the z
  imuHardwareLsm9Ds1.settings.mag.ZPerformance = 3; // Ultra-high perform.
  // [lowPowerEnable] enables or disables low power mode in
  // the magnetometer.
  imuHardwareLsm9Ds1.settings.mag.lowPowerEnable = false;
  // [operatingMode] sets the operating mode of the
  // magnetometer. operatingMode can be 0-2:
  // 0 = continuous conversion
  // 1 = single-conversion
  // 2 = power down
  imuHardwareLsm9Ds1.settings.mag.operatingMode = 0;
  uint lsm9ds1Init = 0;
  if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
    lsm9ds1Init = imuHardwareLsm9Ds1.begin();
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
