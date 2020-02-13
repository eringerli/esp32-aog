#include <ESPUI.h>
#include "main.hpp"
#include "imu.hpp"
#include "udpHandler.hpp"
#include "webUi.hpp"
#include <MadgwickAHRS.h>
#include <MahonyAHRS.h>
#include <Quaternion.h>
#include "imuHardware.hpp"

void (*imuReadData)(float*, float*, float*, float*, float*, float*, float*, float*, float*);
int imuWebStatus;
ImuSettings imuSettings;

void imuInit() {
  imuWebStatus = ESPUI.addControl( ControlType::Label, "Status:", "No imu active", ControlColor::Turquoise, webTabIMU );

  // init the requested Imu
  uint8_t imuSelection = preferences.getUChar("imuChip", 0);
  uint16_t sel = ESPUI.addControl( ControlType::Select, "IMU", (String)imuSelection, ControlColor::Wetasphalt, webTabIMU,
    []( Control * control, int id ) {
      preferences.putUChar("imuChip", control->value.toInt());
      control->color = ControlColor::Carrot;
      ESPUI.updateControl( control );
      webChangeNeedsReboot();
    } );
  ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
  ESPUI.addControl( ControlType::Option, "LSM9DS1", "1", ControlColor::Alizarin, sel );
  ESPUI.addControl( ControlType::Option, "FXAS2100/FXOS8700", "2", ControlColor::Alizarin, sel );
  ESPUI.addControl( ControlType::Option, "MMA8451", "3", ControlColor::Alizarin, sel );

  // activate Imu
  switch (imuSelection) {
    case 1:
      if (!imuHardwareLSM9DS1Init()) {
        status.hardwareStatus = Status::Hardware::error;
      }
      break;
    // case 2:
    //   imuHardwareFXAS2100FXOS8700Init();
    //   break;
    // case 3:
    //   imuHardwareMMA8451Init();
      break;
    default:
      break;
  }

  // imu
  //if (!imuHardwareLSM9DS1Init()) {
  //  hwInitErrors = false;
  //}

  if (imuReadData) {  // some IMU is configured
    imuSettings.mountingRoll = preferences.getFloat("imuRoll", imuSettings.mountingRoll);
    ESPUI.addControl( ControlType::Number, "Mounting correction Roll", (String)imuSettings.mountingRoll, ControlColor::Wetasphalt, webTabIMU,
      []( Control * control, int id ) {
        imuSettings.mountingRoll = control->value.toFloat();
        preferences.putFloat("imuRoll", imuSettings.mountingRoll);
        control->color = ControlColor::Carrot;
        ESPUI.updateControl( control );
      } );
    imuSettings.mountingPitch = preferences.getFloat("imuPitch", imuSettings.mountingPitch);
    ESPUI.addControl( ControlType::Number, "Mounting correction Pitch", (String)imuSettings.mountingPitch, ControlColor::Wetasphalt, webTabIMU,
      []( Control * control, int id ) {
        imuSettings.mountingPitch = control->value.toFloat();
        preferences.putFloat("imuPitch", imuSettings.mountingPitch);
        control->color = ControlColor::Carrot;
        ESPUI.updateControl( control );
      } );
    imuSettings.mountingYaw = preferences.getFloat("imuYaw", imuSettings.mountingYaw);
    ESPUI.addControl( ControlType::Number, "Mounting correction Roll", (String)imuSettings.mountingYaw, ControlColor::Wetasphalt, webTabIMU,
      []( Control * control, int id ) {
        imuSettings.mountingYaw = control->value.toFloat();
        preferences.putFloat("imuYaw", imuSettings.mountingYaw);
        control->color = ControlColor::Carrot;
        ESPUI.updateControl( control );
      } );
    if (imuSettings.hasAccel) {
      imuSettings.sendRoll = preferences.getBool("imuSendRoll");
      ESPUI.addControl( ControlType::Switcher, "Send IMU roll", String( (int)imuSettings.sendRoll ) , ControlColor::Wetasphalt, webTabIMU,
        []( Control * control, int id ) {
          imuSettings.sendRoll = (boolean)control->value.toInt();
          preferences.putBool("imuSendRoll", imuSettings.sendRoll );
          control->color = ControlColor::Carrot;
          ESPUI.updateControl( control );
        } );
    }
    if (imuSettings.hasGyro) {
      ESPUI.addControl( ControlType::Button, "Calibrate gyros", "Start" , ControlColor::Wetasphalt, webTabIMU,
        []( Control * control, int id ) {
          if ( id == B_UP ) {
            imuSettings.callbackGyroCalibration = control->id;
            control->color = ControlColor::Carrot;
            ESPUI.updateControl( control );
          }
        } );
    }
    if (imuSettings.hasMag) {
      ESPUI.addControl( ControlType::Switcher, "Calibrate magnetometer", "0" , ControlColor::Wetasphalt, webTabIMU,
        []( Control * control, int id ) {
          imuSettings.calibrateMag = (boolean)control->value.toInt();
          if (imuSettings.calibrateMag) {
            control->color = ControlColor::Carrot;
            ESPUI.updateControl( control );
          } else { // finished calibration
            control->color = ControlColor::Wetasphalt;
            ESPUI.updateControl( control );
          }
        } );
    }
    if (imuSettings.hasGyro || imuSettings.hasMag) {
      // load data
      imuSettings.calibrationData.gyro_zero_offsets[0] = preferences.getFloat("imuGyroCalX", imuSettings.calibrationData.gyro_zero_offsets[0]);
      imuSettings.calibrationData.gyro_zero_offsets[1] = preferences.getFloat("imuGyroCalY", imuSettings.calibrationData.gyro_zero_offsets[1]);
      imuSettings.calibrationData.gyro_zero_offsets[2] = preferences.getFloat("imuGyroCalZ", imuSettings.calibrationData.gyro_zero_offsets[2]);
      imuSettings.calibrationData.mag_hardiron[0] = preferences.getFloat("imuMagHardX", imuSettings.calibrationData.mag_hardiron[0]);
      imuSettings.calibrationData.mag_hardiron[1] = preferences.getFloat("imuMagHardY", imuSettings.calibrationData.mag_hardiron[1]);
      imuSettings.calibrationData.mag_hardiron[2] = preferences.getFloat("imuMagHardZ", imuSettings.calibrationData.mag_hardiron[2]);
      imuSettings.calibrationData.mag_softiron[0] = preferences.getFloat("imuMagSoftX", imuSettings.calibrationData.mag_softiron[0]);
      imuSettings.calibrationData.mag_softiron[1] = preferences.getFloat("imuMagSoftY", imuSettings.calibrationData.mag_softiron[1]);
      imuSettings.calibrationData.mag_softiron[2] = preferences.getFloat("imuMagSoftZ", imuSettings.calibrationData.mag_softiron[2]);
      // gui
      ESPUI.addControl( ControlType::Button, "Calibration data", "Save" , ControlColor::Wetasphalt, webTabIMU,
        []( Control * control, int id ) {
          if ( id == B_UP ) {
            control->color = ControlColor::Carrot;
            ESPUI.updateControl( control );
            // save data
            preferences.putFloat("imuGyroCalX", imuSettings.calibrationData.gyro_zero_offsets[0]);
            preferences.putFloat("imuGyroCalY", imuSettings.calibrationData.gyro_zero_offsets[1]);
            preferences.putFloat("imuGyroCalZ", imuSettings.calibrationData.gyro_zero_offsets[2]);
            preferences.putFloat("imuMagHardX", imuSettings.calibrationData.mag_hardiron[0]);
            preferences.putFloat("imuMagHardY", imuSettings.calibrationData.mag_hardiron[1]);
            preferences.putFloat("imuMagHardZ", imuSettings.calibrationData.mag_hardiron[2]);
            preferences.putFloat("imuMagSoftX", imuSettings.calibrationData.mag_softiron[0]);
            preferences.putFloat("imuMagSoftY", imuSettings.calibrationData.mag_softiron[1]);
            preferences.putFloat("imuMagSoftZ", imuSettings.calibrationData.mag_softiron[2]);
          }
        } );
    }
    if (imuSettings.hasAccel && (imuSettings.hasGyro || imuSettings.hasMag)) {
      imuSettings.sendHeading = preferences.getBool("imuSendHeading");
      ESPUI.addControl( ControlType::Switcher, "Send IMU heading", String( (int)imuSettings.sendHeading ) , ControlColor::Wetasphalt, webTabIMU,
        []( Control * control, int id ) {
          imuSettings.sendHeading = (boolean)control->value.toInt();
          preferences.putBool("imuSendHeading", imuSettings.sendHeading );
          control->color = ControlColor::Carrot;
          ESPUI.updateControl( control );
        } );
    }

    // WebUI is ready, calibration data is loaded start task
    xTaskCreate( imuTask, "IMU", 8192, NULL, 4, NULL );
  }
}

void imuTask(void *z) {
  vTaskDelay( 2074 );
  constexpr TickType_t xFrequency = 20; // run at 50hz
  TickType_t xLastWakeTime = xTaskGetTickCount();
  // variables for the raw values
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  // for imu calibration
  float magCalData[3][2] = {{32767, -32767}, {32767, -32767}, {32767, -32767}};
  float gyroCalData[3] = {0, 0, 0};
  int gyroCalibrationCounter = 0;
  // for sensor fusion, depending if 6 or 9 axis
  Madgwick madgwick; // 9 axis
  Mahony mahony; // 6 axis

  // filter for roll and heading
  //Low pass butterworth filter order=2 alpha1=0.1
  //http://www.schwietering.com/jayduino/filtuino/index.php?characteristic=bu&passmode=lp&order=2&usesr=usesr&sr=50&frequencyLow=5&noteLow=&noteHigh=&pw=pw&calctype=float&run=Send
  class  FilterBuLp2_imu {
  	public:
  		FilterBuLp2_imu() {
  			v[0]=0.0;
  			v[1]=0.0;
  		}
  	private:
  		float v[3];
  	public:
  		float step(float x) { //class II
  			v[0] = v[1];
  			v[1] = v[2];
  			v[2] = (6.745527388907189559e-2 * x)
  				 + (-0.41280159809618854894 * v[0])
  				 + (1.14298050253990091107 * v[1]);
  			return
  				 (v[0] + v[2])
  				+2 * v[1];
  		}
  } rollFilter, headingFilter;

  // init Mahony or Madgwick
  if (imuSettings.hasAccel && imuSettings.hasGyro && imuSettings.hasMag) {
    madgwick.begin(50);
  }
  if (imuSettings.hasAccel && imuSettings.hasGyro && !imuSettings.hasMag) {
    mahony.begin(50);
  }

  // debug
  bool debugImu = false;
  int debugCounter = 0;

  // loop
  while (true) {
    // get new data
    imuReadData(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    // debug raw
    if (debugImu && (debugCounter++ % 20) == 0) {
      usb.print("IMU raw data. ax: ");
      usb.print(ax);
      usb.print("  ay: ");
      usb.print(ay);
      usb.print("  az: ");
      usb.print(az);
      usb.print("  gx: ");
      usb.print(gx);
      usb.print("  gy: ");
      usb.print(gy);
      usb.print("  gz: ");
      usb.print(gz);
      usb.print("  mx: ");
      usb.print(mx);
      usb.print("  my: ");
      usb.print(my);
      usb.print("  mz: ");
      usb.print(mz);
      usb.println();
    }


    // calibration if requested
    //
    //   gyros
    if (imuSettings.callbackGyroCalibration > 0) {
      gyroCalData[0] += gx;
      gyroCalData[1] += gy;
      gyroCalData[2] += gz;
      if (gyroCalibrationCounter == 1023) { //enough data
        imuSettings.calibrationData.gyro_zero_offsets[0] = gyroCalData[0]/1024;
        imuSettings.calibrationData.gyro_zero_offsets[1] = gyroCalData[1]/1024;
        imuSettings.calibrationData.gyro_zero_offsets[2] = gyroCalData[2]/1024;
        gyroCalibrationCounter = 0;
        // change ControlColor
        ESPUI.getControl( imuSettings.callbackGyroCalibration )->color = ControlColor::Emerald;
        ESPUI.updateControl( imuSettings.callbackGyroCalibration );
        imuSettings.callbackGyroCalibration = 0;
        gyroCalData[0] = 0;
        gyroCalData[1] = 0;
        gyroCalData[2] = 0;
        // debug
        usb.print("Gyro calibration finisched: ");
        usb.print(imuSettings.calibrationData.gyro_zero_offsets[0]);
        usb.print(", ");
        usb.print(imuSettings.calibrationData.gyro_zero_offsets[1]);
        usb.print(", ");
        usb.println(imuSettings.calibrationData.gyro_zero_offsets[2]);
      } else {
        gyroCalibrationCounter++;
      }
    }
    //   magnetometer
    if (imuSettings.calibrateMag) {
      if (mx < magCalData[0][0]) {
        magCalData[0][0] = mx;
      }
      if (mx > magCalData[0][1]) {
        magCalData[0][1] = mx;
      }
      if (my < magCalData[1][0]) {
        magCalData[1][0] = my;
      }
      if (my > magCalData[1][1]) {
        magCalData[1][1] = my;
      }
      if (mz < magCalData[2][0]) {
        magCalData[2][0] = mz;
      }
      if (mz > magCalData[2][1]) {
        magCalData[2][1] = mz;
      }
    }
    if (!imuSettings.calibrateMag && magCalData[0][0] != 32767) { // just finished a calibration run => update data
      imuCalibrationCalcMagnetometer(&magCalData);
    }

    // always apply compensation, does not hurt if not necessary
    mx = (mx - imuSettings.calibrationData.mag_hardiron[0]) * imuSettings.calibrationData.mag_softiron[0];
    my = (my - imuSettings.calibrationData.mag_hardiron[1]) * imuSettings.calibrationData.mag_softiron[1];
    mz = (mz - imuSettings.calibrationData.mag_hardiron[2]) * imuSettings.calibrationData.mag_softiron[2];

    gx -= imuSettings.calibrationData.gyro_zero_offsets[0];
    gy -= imuSettings.calibrationData.gyro_zero_offsets[1];
    gz -= imuSettings.calibrationData.gyro_zero_offsets[2];

    // debug output of unprocessed Values
    if (debugImu && (debugCounter % 20) == 0) {
      usb.print("IMU data after correction. ax: ");
      usb.print(ax);
      usb.print("  ay: ");
      usb.print(ay);
      usb.print("  az: ");
      usb.print(az);
      usb.print("  gx: ");
      usb.print(gx);
      usb.print("  gy: ");
      usb.print(gy);
      usb.print("  gz: ");
      usb.print(gz);
      usb.print("  mx: ");
      usb.print(mx);
      usb.print("  my: ");
      usb.print(my);
      usb.print("  mz: ");
      usb.print(mz);
      usb.println();
    }

    // calculate mounting correction
    Quaternion correction = Quaternion::from_euler_rotation( radians( imuSettings.mountingRoll ),
                          radians( imuSettings.mountingPitch ),
                          radians( imuSettings.mountingYaw ) );
    correction.normalize();

    // calculate roll
    if (imuSettings.hasAccel) {
      Quaternion orientation = Quaternion(ax, ay, az);
      orientation.normalize();
      orientation = orientation.rotate(correction);
      double roll, placeholder;
      orientation.to_euler_rotation(&placeholder, &placeholder, &roll);
      roll *= 57.2957795131; //180/pi
      imuSettings.roll = rollFilter.step((float)roll);
      if (imuSettings.sendRoll) {
        udpActualData.roll = imuSettings.roll;
      }
    }


    // calculate heading
    Quaternion imuHeading;
    if (imuSettings.hasAccel && imuSettings.hasGyro){
      if (imuSettings.hasMag) { // 9 axis
        madgwick.update(gx, gy, gz, ax, ay, az, mx, my, mz);
        imuHeading = Quaternion::from_euler_rotation(
                      madgwick.getRollRadians(),
                      madgwick.getPitchRadians(),
                      madgwick.getYawRadians());
      } else {  // 6 axis
        mahony.updateIMU(gx, gy, gz, ax, ay, az);
        imuHeading = Quaternion::from_euler_rotation(
                      mahony.getRollRadians(),
                      mahony.getPitchRadians(),
                      mahony.getYawRadians());
      }
      imuHeading = imuHeading.rotate(correction);
      double heading, placeholder;
      imuHeading.to_euler_rotation(&heading, &placeholder, &placeholder);
      heading *= 57.2957795131; //180/pi
      heading = fmod(heading, 360);
      imuSettings.heading = headingFilter.step((float)heading);
      if (imuSettings.sendHeading) {
        udpActualData.heading = imuSettings.heading;
      }
    }

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }


}

void imuStatusUpdate() {
  String str;
  str.reserve( 70 );
  str = "Roll: ";
  if (imuSettings.hasAccel) {
    str += String(imuSettings.roll,1);
  } else {
    str += "NA";
  }
  str += "<br />Heading: ";
  if (imuSettings.hasAccel && imuSettings.hasGyro){
    str += String(imuSettings.heading,1);
  } else {
    str += "NA";
  }
  if (imuReadData){
    Control* label = ESPUI.getControl( imuWebStatus );
    label->value = str;
    ESPUI.updateControl( imuWebStatus );
  }
}


void imuCalibrationCalcMagnetometer(float (*magCalData)[3][2]) {
  //disable further updates
  imuSettings.calibrateMag = false;

  // calibration after https://appelsiini.net/2018/calibrate-magnetometer/
  // calculate hard iron compensation
  imuSettings.calibrationData.mag_hardiron[0] = ((*magCalData)[0][0] + (*magCalData)[0][1]) / 2;
  imuSettings.calibrationData.mag_hardiron[1] = ((*magCalData)[1][0] + (*magCalData)[1][1]) / 2;
  imuSettings.calibrationData.mag_hardiron[2] = ((*magCalData)[2][0] + (*magCalData)[2][1]) / 2;

  // apply the hard iron correction on the raw data
  (*magCalData)[0][0] -= imuSettings.calibrationData.mag_hardiron[0];
  (*magCalData)[1][0] -= imuSettings.calibrationData.mag_hardiron[1];
  (*magCalData)[2][0] -= imuSettings.calibrationData.mag_hardiron[2];
  (*magCalData)[0][1] -= imuSettings.calibrationData.mag_hardiron[0];
  (*magCalData)[1][1] -= imuSettings.calibrationData.mag_hardiron[1];
  (*magCalData)[2][1] -= imuSettings.calibrationData.mag_hardiron[2];

  // calculate soft iron data
  imuSettings.calibrationData.mag_softiron[0] = ((*magCalData)[0][1] - (*magCalData)[0][0]) / 2;
  imuSettings.calibrationData.mag_softiron[1] = ((*magCalData)[1][1] - (*magCalData)[1][0]) / 2;
  imuSettings.calibrationData.mag_softiron[2] = ((*magCalData)[2][1] - (*magCalData)[2][0]) / 2;

  float avgDelta = (imuSettings.calibrationData.mag_softiron[0] + imuSettings.calibrationData.mag_softiron[1] + imuSettings.calibrationData.mag_softiron[2]) / 3;

  imuSettings.calibrationData.mag_softiron[0] = avgDelta / imuSettings.calibrationData.mag_softiron[0];
  imuSettings.calibrationData.mag_softiron[1] = avgDelta / imuSettings.calibrationData.mag_softiron[1];
  imuSettings.calibrationData.mag_softiron[2] = avgDelta / imuSettings.calibrationData.mag_softiron[2];

  // debug
  usb.print("Magnetometer calibration finisched. Hard iron: ");
  usb.print(imuSettings.calibrationData.mag_hardiron[0]);
  usb.print(", ");
  usb.print(imuSettings.calibrationData.mag_hardiron[1]);
  usb.print(", ");
  usb.print(imuSettings.calibrationData.mag_hardiron[2]);
  usb.print(" - softiron: ");
  usb.print(imuSettings.calibrationData.mag_softiron[0]);
  usb.print(", ");
  usb.print(imuSettings.calibrationData.mag_softiron[1]);
  usb.print(", ");
  usb.println(imuSettings.calibrationData.mag_softiron[2]);

  // reset magCalData for next run
  (*magCalData)[0][0] = 32767;
  (*magCalData)[0][1] = -32767;
  (*magCalData)[1][0] = 32767;
  (*magCalData)[1][1] = -32767;
  (*magCalData)[2][0] = 32767;
  (*magCalData)[2][1] = -32767;
}
