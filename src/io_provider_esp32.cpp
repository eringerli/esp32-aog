#include <stdint.h>
#include <string>
#include <esp32-hal-adc.h>
#include "io_provider.cpp"

class IoProvider_ESP32 : public IoProvider {
public:
  IoProvider_ESP32() {
  }
  bool init() {
    analogReadResolution(10); // Default of 12 is not very linear. Recommended to use 10 or 11 depending on needed resolution.
    analogSetAttenuation(ADC_11db); // Default is 11db which is very noisy. But needed for full scale range  Recommended to use 2.5 or 6.
    return true;
  }

  // configure a port, default nothing enabled
  bool configureAsDigitalInput(uint8_t port, bool usePullUpDown, bool pullDirectionUp) {
    if (configuration[port] != unconfigured) {
      return false;
    }
    // set up the port as input, no interrupt, error handling is a TODO
    configuration[port] = digitalInput;
    gpio_pad_select_gpio(portMapping[port]);
    gpio_set_direction(portMapping[port], GPIO_MODE_INPUT);
    gpio_intr_disable(portMapping[port]);
    if (usePullUpDown) {
      if (pullDirectionUp) {
        gpio_set_pull_mode(portMapping[port], GPIO_PULLUP_ONLY);
      } else {
        gpio_set_pull_mode(portMapping[port], GPIO_PULLDOWN_ONLY);
      }
    } else {
      gpio_set_pull_mode(portMapping[port], GPIO_FLOATING);
    }
    return true;
  }

  bool configureAsAnalogInput(uint8_t port) {
    if (configuration[port] != unconfigured) {
      return false;
    }
    configuration[port] = analogInput;
    gpio_pad_select_gpio(portMapping[port]);
    gpio_set_direction(portMapping[port], GPIO_MODE_INPUT);
    gpio_intr_disable(portMapping[port]);
    gpio_set_pull_mode(portMapping[port], GPIO_FLOATING);
    return true;
  }

  bool configureAsDigitalOutput(uint8_t port) {
    if (configuration[port] != unconfigured) {
      return false;
    }
    configuration[port] = digitalOutput;
    gpio_pad_select_gpio(portMapping[port]);
    gpio_intr_disable(portMapping[port]);
    gpio_set_pull_mode(portMapping[port], GPIO_FLOATING);
    gpio_set_direction(portMapping[port], GPIO_MODE_OUTPUT);
    return true;
  }

  bool configureAsPwmOutput(uint8_t port, uint16_t frequency) {
    if (configuration[port] != unconfigured) {
      return false;
    }
    for (int i = 0; i < 4; i++) {
      if (pwmChannels[i].inUse == false) {
        // internals
        configuration[port] = pwmOutput;
        pwmChannels[i].inUse = true;
        pwmChannels[i].usedBay = portMapping[port];
        // pin
        gpio_pad_select_gpio(portMapping[port]);
        gpio_intr_disable(portMapping[port]);
        gpio_set_pull_mode(portMapping[port], GPIO_FLOATING);
        gpio_set_direction(portMapping[port], GPIO_MODE_OUTPUT);
        // pwm
        ledcSetup(i, frequency, 8);
        ledcWrite(i, 0);
        ledcAttachPin(portMapping[port], i);
        return true;
      }
    }
    return false;
  }

  bool configureAsRawIo(uint8_t port) {
    if (configuration[port] != unconfigured) {
      return false;
    }
    configuration[port] = rawIo;
    return true;
  }

// returns the pin number of a port on the microcontroler.
// should only be used if necessary (e.g. setting up serial ports)
// other application has to configure the pin
gpio_num_t getRawIo(uint8_t port) const {
  return portMapping[port];
}

// normal interactions during runtime
bool getDigitalInput(uint8_t port)  const {
  return digitalRead(portMapping[port]);
}

uint16_t getAnalogInput(uint8_t port)  const {
  return (uint16_t)analogRead(portMapping[port]);
}
uint16_t getAnalogInputScaled(uint8_t port)  const {
  return getAnalogInput(port) << 6; //6 bits shifted since 10 bit adc
}
void setDigitalOutput(uint8_t port, bool state) {
  if (state) {
    digitalWrite(portMapping[port], HIGH);
  } else {
    digitalWrite(portMapping[port], LOW);
  }
};
void setPwmOutput(uint8_t port, uint8_t dutyCycle) {
  for (int i = 0; i < 4; i++) {
    if (pwmChannels[i].inUse == true && pwmChannels[i].usedBay == portMapping[port]) {
      ledcWrite(i, dutyCycle);
    }
  }
};

private:
  std::string getName() const {return "ESP32"; }
  const portDefinition ports[32] = {
    {"GPIO  0", true, false, true, true, true},
    {"GPIO  1", true, false, true, true, true},
    {"GPIO  2", true, false, true, true, true},
    {"GPIO  3", true, false, true, true, true},
    {"GPIO  4", true, false, true, true, true},
    {"GPIO  5", true, false, true, true, true},
    {"GPIO 12", true, false, true, true, true},
    {"GPIO 13", true, false, true, true, true},
    {"GPIO 14", true, false, true, true, true},
    {"GPIO 15", true, false, true, true, true},
    {"GPIO 16", true, false, true, true, true},
    {"GPIO 17", true, false, true, true, true},
    {"GPIO 18", true, false, true, true, true},
    {"GPIO 19", true, false, true, true, true},
    {"GPIO 21", true, false, true, true, true},
    {"GPIO 22", true, false, true, true, true},
    {"GPIO 23", true, false, true, true, true},
    {"GPIO 25", true, false, true, true, true},
    {"GPIO 26", true, false, true, true, true},
    {"GPIO 27", true, false, true, true, true},
    {"GPIO 32", true, false, true, true, true},
    {"GPIO 33", true, false, true, false, true},
    {"GPIO 34", true, true, false, false, true},
    {"GPIO 35", true, true, false, false, true},
    {"GPIO 36", true, true, false, false, true},
    {"GPIO 39", true, true, false, false, true} };
  const gpio_num_t portMapping[32] = {
    GPIO_NUM_0, GPIO_NUM_1, GPIO_NUM_2, GPIO_NUM_3, GPIO_NUM_4, GPIO_NUM_5,
    GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_15, GPIO_NUM_16, GPIO_NUM_17,
    GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_21, GPIO_NUM_22, GPIO_NUM_23, GPIO_NUM_25,
    GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_34, GPIO_NUM_35,
    GPIO_NUM_36, GPIO_NUM_39};
  struct pwmChannel {
    bool inUse = false;
    gpio_num_t usedBay;
    pwmChannel(){};
    };
  pwmChannel pwmChannels[4];
};
