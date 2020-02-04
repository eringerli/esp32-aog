#ifndef imu_HPP
#define imu_HPP
#include <stdint.h>


// updates the data, accel, gyros, magnetometer (each x, y, z)
extern void (*imuReadData)(float*, float*, float*, float*, float*, float*, float*, float*, float*);

struct ImuSettings {
  bool hasAccel = false;
  bool hasGyro = false;
  bool hasMag = false;

  bool calibrateMag = false;
  int callbackGyroCalibration = 0; // if not 0, calibrate gyros and reset color of this UI-COntrol

  float mountingRoll = 0;
  float mountingPitch = 0;
  float mountingYaw = 0;


  struct GenericImuCalibrationData {
    GenericImuCalibrationData() {
      // init with neutral element
      mag_hardiron[0] = 0.0;
      mag_hardiron[1] = 0.0;
      mag_hardiron[2] = 0.0;
      mag_softiron[0] = 1.0;
      mag_softiron[1] = 1.0;
      mag_softiron[2] = 1.0;

      gyro_zero_offsets[0] = 0;
      gyro_zero_offsets[1] = 0;
      gyro_zero_offsets[2] = 0;
    };

    // hard iron compensation
    float mag_hardiron[3];
    // Soft iron error compensation matrix
    float mag_softiron[3];
    // Offsets applied to compensate for gyro zero-drift error for x/y/z
    float gyro_zero_offsets[3];
  };
  GenericImuCalibrationData calibrationData;

  float roll = 9999/16.0;
  float heading = 9999/16.0;

  bool sendRoll = false;
  bool sendHeading = false;
};
extern ImuSettings imuSettings;

void imuInit();
void imuTask(void *z);
void imuStatusUpdate();
void imuCalibrationCalcMagnetometer(float (*magCalData)[3][2]);

#endif
