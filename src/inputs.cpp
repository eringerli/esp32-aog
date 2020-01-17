#include "inputs.hpp"
#include "webUi.hpp"
#include "main.hpp"
#include "ioAccess.hpp"
#include "udpHandler.hpp"
#include <ESPUI.h>

int inputsWasWebStatus;
InputsWasData inputsWasSetup;
InputsSwitchesConfig inputsSwitchesSetup;

// filter for steering angle
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
} wheelAngleSensorFilter;


// reads work & steerswitches
void inputsSwitchesInit() {
  // Webinterface
  // Workswitch
  inputsSwitchesSetup.workSwitchPort = preferences.getUChar("inputsWsIo", 253);
  uint16_t sel = ESPUI.addControl( ControlType::Select, "Workswitch input", (String)inputsSwitchesSetup.workSwitchPort, ControlColor::Wetasphalt, webTabWorkSteerSwitch,
    []( Control * control, int id ) {
      inputsSwitchesSetup.workSwitchPort = control->value.toInt();
      preferences.putUChar("inputsWsIo", inputsSwitchesSetup.workSwitchPort);
      control->color = ControlColor::Carrot;
      ESPUI.updateControl( control );
    } );
  ESPUI.addControl( ControlType::Option, "True", "254", ControlColor::Alizarin, sel );
  ESPUI.addControl( ControlType::Option, "False", "253", ControlColor::Alizarin, sel );
  ioAccessWebListAnalogIn(sel);

  inputsSwitchesSetup.workSwitchThreshold = preferences.getUChar("inputsWsSp", 50);
  ESPUI.addControl( ControlType::Number, "Workswitch switch-point in %", String(inputsSwitchesSetup.workSwitchThreshold), ControlColor::Wetasphalt, webTabWorkSteerSwitch,
    []( Control * control, int id ) {
      inputsSwitchesSetup.workSwitchThreshold = control->value.toInt();
      preferences.putUChar("inputsWsSp", inputsSwitchesSetup.workSwitchThreshold);
      control->color = ControlColor::Carrot;
      ESPUI.updateControl( control );
    } );

    // Steer switch
    inputsSwitchesSetup.steerSwitchPort = preferences.getUChar("inputsSsIo", 253);
    sel = ESPUI.addControl( ControlType::Select, "Steerswitch input", (String)inputsSwitchesSetup.steerSwitchPort, ControlColor::Wetasphalt, webTabWorkSteerSwitch,
      []( Control * control, int id ) {
        inputsSwitchesSetup.steerSwitchPort = control->value.toInt();
        preferences.putUChar("inputsSsIo", inputsSwitchesSetup.steerSwitchPort);
        control->color = ControlColor::Carrot;
        ESPUI.updateControl( control );
      } );
    ESPUI.addControl( ControlType::Option, "True", "254", ControlColor::Alizarin, sel );
    ESPUI.addControl( ControlType::Option, "False", "253", ControlColor::Alizarin, sel );
    ioAccessWebListAnalogIn(sel);

    inputsSwitchesSetup.steerSwitchThreshold = preferences.getUChar("inputsSsSp", 50);
    ESPUI.addControl( ControlType::Number, "Steerswitch switch-point in %", String(inputsSwitchesSetup.steerSwitchThreshold), ControlColor::Wetasphalt, webTabWorkSteerSwitch,
      []( Control * control, int id ) {
        inputsSwitchesSetup.steerSwitchThreshold = control->value.toInt();
        preferences.putUInt("inputsSsSp", inputsSwitchesSetup.steerSwitchThreshold);
        control->color = ControlColor::Carrot;
        ESPUI.updateControl( control );
      } );

    inputsSwitchesSetup.steerSwitchInvert = preferences.getBool("inputsSsInv");
    ESPUI.addControl( ControlType::Switcher, "Steerswitch invert value", String( (int)inputsSwitchesSetup.steerSwitchInvert ) , ControlColor::Wetasphalt, webTabWorkSteerSwitch,
      []( Control * control, int id ) {
        inputsSwitchesSetup.steerSwitchInvert = (boolean)control->value.toInt();
        preferences.putBool("inputsSsInv", inputsSwitchesSetup.steerSwitchInvert );
        control->color = ControlColor::Carrot;
        ESPUI.updateControl( control );
      } );

    inputsSwitchesSetup.steerSwitchIsButton = preferences.getBool("inputsSsButton");
    ESPUI.addControl( ControlType::Switcher, "Steerswitch is button", String( (int)inputsSwitchesSetup.steerSwitchIsButton ) , ControlColor::Wetasphalt, webTabWorkSteerSwitch,
      []( Control * control, int id ) {
        inputsSwitchesSetup.steerSwitchIsButton = (boolean)control->value.toInt();
        preferences.putBool("inputsSsButton", inputsSwitchesSetup.steerSwitchIsButton );
        control->color = ControlColor::Carrot;
        ESPUI.updateControl( control );
      } );

    // Additional SteerEnable
    inputsSwitchesSetup.steerEnablePort = preferences.getUChar("inputsSEIo", 254);
    sel = ESPUI.addControl( ControlType::Select, "Steer enabled input", (String)inputsSwitchesSetup.steerEnablePort, ControlColor::Wetasphalt, webTabWorkSteerSwitch,
      []( Control * control, int id ) {
        inputsSwitchesSetup.steerEnablePort = control->value.toInt();
        preferences.putUChar("inputsSEIo", inputsSwitchesSetup.steerEnablePort);
        control->color = ControlColor::Carrot;
        ESPUI.updateControl( control );
      } );
    ESPUI.addControl( ControlType::Option, "True", "254", ControlColor::Alizarin, sel );
    ESPUI.addControl( ControlType::Option, "False", "253", ControlColor::Alizarin, sel );
    ioAccessWebListAnalogIn(sel);

    inputsSwitchesSetup.steerEnableThreshold = preferences.getUChar("inputsSESp", 50);
    ESPUI.addControl( ControlType::Number, "Steerenabled switch-point in %", String(inputsSwitchesSetup.steerEnableThreshold), ControlColor::Wetasphalt, webTabWorkSteerSwitch,
      []( Control * control, int id ) {
        inputsSwitchesSetup.steerEnableThreshold = control->value.toInt();
        preferences.putUInt("inputsSESp", inputsSwitchesSetup.steerEnableThreshold);
        control->color = ControlColor::Carrot;
        ESPUI.updateControl( control );
      } );

    inputsSwitchesSetup.steerEnableInvert = preferences.getBool("inputsSEInv");
    ESPUI.addControl( ControlType::Switcher, "Steer enable invert value", String( (int)inputsSwitchesSetup.steerEnableInvert ) , ControlColor::Wetasphalt, webTabWorkSteerSwitch,
      []( Control * control, int id ) {
        inputsSwitchesSetup.steerEnableInvert = (boolean)control->value.toInt();
        preferences.putBool("inputsSEInv", inputsSwitchesSetup.steerEnableInvert );
        control->color = ControlColor::Carrot;
        ESPUI.updateControl( control );
      } );

    // start task
    xTaskCreate( inputsSwitchesTask, "Switches", 4096, NULL, 4, NULL );
}

// against gliches/noise: two values must show same value for a change + a hysteresis
void inputsSwitchesTask(void *z) {
  bool steerSwitchLastValidState = false;
  bool steerSwitchLastState = false;
  bool workSwitchLastState = false;
  bool steerEnabledLastState = false;

  for ( ;; ) {
    // workSwitch
    float currentValue = fabs(ioAccessGetAnalogInput(inputsSwitchesSetup.workSwitchPort));
    if (udpActualData.workSwitch) {
      if (currentValue < ((inputsSwitchesSetup.workSwitchThreshold - inputsHysteresis)/100.0)) {
        if (workSwitchLastState == false) {
          udpActualData.workSwitch = false;
        }
        workSwitchLastState = false;
      } else {
        workSwitchLastState = true;
      }
    } else { // current workswitch false
      if (currentValue > ((inputsSwitchesSetup.workSwitchThreshold + inputsHysteresis)/100.0)) {
        if (workSwitchLastState == true) {
          udpActualData.workSwitch = true;
        }
        workSwitchLastState = true;
      } else {
        workSwitchLastState = false;
      }
    }

    // steer enable input
    currentValue = fabs(ioAccessGetAnalogInput(inputsSwitchesSetup.steerEnablePort));
    if (steerEnabledLastState && currentValue < ((inputsSwitchesSetup.workSwitchThreshold - inputsHysteresis)/100.0)) {
      steerEnabledLastState = false;
    } else if (!steerEnabledLastState && currentValue > ((inputsSwitchesSetup.workSwitchThreshold + inputsHysteresis)/100.0)) {
      steerEnabledLastState = true;
    }


    // steerswitch
    currentValue = fabs(ioAccessGetAnalogInput(inputsSwitchesSetup.steerSwitchPort));
    if (inputsSwitchesSetup.steerSwitchInvert) { // invert the raw value => hysteresislogic has only one case, also the "rising edge" logik for the button
      currentValue = 1.0 - currentValue;
    }
    bool newValue = steerSwitchLastValidState;
    if (steerSwitchLastValidState) {
      if (currentValue < ((inputsSwitchesSetup.steerSwitchThreshold - inputsHysteresis)/100.0)) {
        newValue = false;
      } else {
        newValue = true;
      }
    } else { // current steerswitch false
      if (currentValue > ((inputsSwitchesSetup.steerSwitchThreshold + inputsHysteresis)/100.0)) {
        newValue = true;
      } else {
        newValue = false;
      }
    }
    if (steerEnabledLastState) {
      if (!steerSwitchLastValidState && steerSwitchLastState && newValue) { // false => true
        steerSwitchLastValidState = true;
        if (inputsSwitchesSetup.steerSwitchIsButton) {
          udpActualData.steerSwitch = !udpActualData.steerSwitch;
        } else {
          udpActualData.steerSwitch = true;
        }
      } else if (steerSwitchLastValidState && !steerSwitchLastState && !newValue) { // true => false
        steerSwitchLastValidState = false;
        if (!inputsSwitchesSetup.steerSwitchIsButton) {
          udpActualData.steerSwitch = false;
        }
      }
    } else {
      udpActualData.steerSwitch = true; // disabled
    }
    steerSwitchLastState = newValue;

    vTaskDelay( 16 );
  }
}

// calculates wheel angle
void inputsWheelAngleInit() {
  inputsWasWebStatus = ESPUI.addControl( ControlType::Label, "Status:", "", ControlColor::Turquoise, webTabSteeringAngle );
  inputsWasSetup.inputPort = preferences.getUChar("inputsWasIo", 255);
  uint16_t sel = ESPUI.addControl( ControlType::Select, "Wheel angle sensor input", (String)inputsWasSetup.inputPort, ControlColor::Wetasphalt, webTabSteeringAngle,
    []( Control * control, int id ) {
      inputsWasSetup.inputPort = control->value.toInt();
      preferences.putUChar("inputsWasIo", inputsWasSetup.inputPort);
      control->color = ControlColor::Carrot;
      ESPUI.updateControl( control );
      webChangeNeedsReboot();
    } );
  ESPUI.addControl( ControlType::Option, "None", "255", ControlColor::Alizarin, sel );
  ioAccessWebListAnalogIn(sel);
  inputsWasSetup.invertSensor = preferences.getBool("inputsWasInv");
  ESPUI.addControl( ControlType::Switcher, "Invert Signal", String( (int)inputsWasSetup.invertSensor ) , ControlColor::Wetasphalt, webTabSteeringAngle,
    []( Control * control, int id ) {
      inputsWasSetup.invertSensor = (boolean)control->value.toInt();
      preferences.putBool("inputsWasInv", inputsWasSetup.invertSensor);
      control->color = ControlColor::Carrot;
    } );
  inputsWasSetup.center = preferences.getFloat("inputsWasCenter", 0.5);
  ESPUI.addControl( ControlType::Number, "Wheel angle sensor center", (String)inputsWasSetup.center, ControlColor::Wetasphalt, webTabSteeringAngle,
    []( Control * control, int id ) {
      inputsWasSetup.center = control->value.toFloat();
      preferences.putFloat("inputsWasCenter", inputsWasSetup.center);
      control->color = ControlColor::Carrot;
      ESPUI.updateControl( control );
    } );

  inputsWasSetup.correction = preferences.getUChar("inputsWasCorr", 0);
  sel = ESPUI.addControl( ControlType::Select, "Correktion", (String)inputsWasSetup.correction, ControlColor::Wetasphalt, webTabSteeringAngle,
    []( Control * control, int id ) {
      inputsWasSetup.correction = control->value.toInt();
      preferences.putUChar("inputsWasCorr", inputsWasSetup.correction);
      control->color = ControlColor::Carrot;
      ESPUI.updateControl( control );
      webChangeNeedsReboot();
    } );
  ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
  ESPUI.addControl( ControlType::Option, "Ackermann (sensor left)", "1", ControlColor::Alizarin, sel );
  ESPUI.addControl( ControlType::Option, "Ackermann (sensor right)", "2", ControlColor::Alizarin, sel );
  sel = preferences.getUChar("inputsWasCorr", 0);
  if ( sel == 1 || sel == 2) {
    // data for Ackermann korrection
    inputsWasSetup.wheelbase = preferences.getInt("inputsWasWB", 450);
    ESPUI.addControl( ControlType::Number, "Wheelbase (cm)", (String)inputsWasSetup.wheelbase, ControlColor::Wetasphalt, webTabSteeringAngle,
      []( Control * control, int id ) {
        inputsWasSetup.wheelbase = control->value.toInt();
        preferences.putInt("inputsWasWB", inputsWasSetup.wheelbase);
        control->color = ControlColor::Carrot;
        ESPUI.updateControl( control );
      } );
    inputsWasSetup.trackWidth = preferences.getInt("inputsWasTW", 200);
    ESPUI.addControl( ControlType::Number, "Track width (cm)", (String)inputsWasSetup.trackWidth, ControlColor::Wetasphalt, webTabSteeringAngle,
      []( Control * control, int id ) {
        inputsWasSetup.trackWidth = control->value.toInt();
        preferences.putInt("inputsWasTW", inputsWasSetup.trackWidth);
        control->color = ControlColor::Carrot;
        ESPUI.updateControl( control );
      } );
  } // end of ackermann
  inputsWasSetup.degreMultiplier = preferences.getFloat("inputsWasMult", 75.5);
  ESPUI.addControl( ControlType::Number, "Multiplier to degrees", (String)inputsWasSetup.degreMultiplier, ControlColor::Wetasphalt, webTabSteeringAngle,
    []( Control * control, int id ) {
      inputsWasSetup.degreMultiplier = control->value.toFloat();
      preferences.putFloat("inputsWasMult", inputsWasSetup.degreMultiplier);
      control->color = ControlColor::Carrot;
      ESPUI.updateControl( control );
    } );

// start task
xTaskCreate( inputsWheelAngleTask, "WAS", 4096, NULL, 8, NULL );

} // end WAS init

void inputsWheelAngleTask(void *z) {
  constexpr TickType_t xFrequency = 20;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
    float newInput = fabs(ioAccessGetAnalogInput(inputsWasSetup.inputPort)); // use fabs is the signal goes to negative numbers, eg. uses 5V for preferences
    inputsWasSetup.statusRaw = newInput;
    if (inputsWasSetup.invertSensor) {
      newInput = 1 - newInput;
    }
    // center = 0
    newInput = newInput - inputsWasSetup.center;

    // multiply to get the degres
    newInput = newInput * inputsWasSetup.degreMultiplier;
    inputsWasSetup.statusDegrees = newInput;

    // Ackermann (ignore everything below 0,5°)
    if ( (inputsWasSetup.correction == 1  || inputsWasSetup.correction == 2 ) && (newInput > 0.5 || newInput < -0.5)) {
      // just for the human, nicer names
      bool negativeAngle = newInput < 0;
      float mathAngle = abs(newInput) * PI / 180;

      // calculate the distance of the adjacent side of the triangle (turning point rear axle <-> turn circle center)
      float distance = inputsWasSetup.wheelbase / tan( mathAngle );
      // add or substract half the trackWidth
      if ( ( negativeAngle && inputsWasSetup.correction == 1 )
          || ( ! negativeAngle && inputsWasSetup.correction == 2 ) ) {
          distance += inputsWasSetup.trackWidth / 2;
      } else {
        distance -= inputsWasSetup.trackWidth / 2;
      }

      // now calculate the virtual wheel in the center
      mathAngle = atan(inputsWasSetup.wheelbase / distance);

      // convert back to degrees and add go back to negative/positive
      if (negativeAngle) {
        newInput = mathAngle * 180 / PI * -1;
      } else {
        newInput = mathAngle * 180 / PI;
      }
    } // end of Ackermann

    // filter the values a bit
    newInput = wheelAngleSensorFilter.step( newInput );
    // update data
    udpActualData.steerAngleActual = newInput;

    // wait for next cycle
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

  } // end while loop
}

void inputsWheelAngleStatusUpdate() {
  String str;
  str.reserve( 70 );

  str = "Raw: ";
  str += String(inputsWasSetup.statusRaw, 3);
  str += "<br />Raw degrees: ";
  str += String(inputsWasSetup.statusDegrees, 1);
  str += "<br />Final: ";
  str += String(udpActualData.steerAngleActual, 1);
  if (inputsWasWebStatus != 0 ){
    Control* labelGpsStatus = ESPUI.getControl( inputsWasWebStatus );
    labelGpsStatus->value = str;
    ESPUI.updateControl( inputsWasWebStatus );
  }
}
