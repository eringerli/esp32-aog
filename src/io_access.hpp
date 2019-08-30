#include <stdint.h>
#include <string>
#include "io_provider.cpp"


class IoAccess {
  // mapping the internal uint8_t to different IO access providers.
  // Every access provider is responsible for 32 IOs
  // Value 255 is the reserved value for invalid pins
public:
  bool addIoProvider(IoProviders type, uint8_t position) {
    // check if position exists
    if (position >= sizeof(provider) / sizeof(provider[0])) {
      return false;
    }
    // check if free
    if (provider[position] != nullptr) {
      return false;
    }
    // looks good, create IO provider
    switch ( type ) {
      case None:
        return true;
      case ESP32_IO:
        provider[position] = new IoProvider_ESP32();
        return provider[position] -> init();
      case ADS1115_48:
        provider[position] = new IoProvider_ADS115(0x48);
        return provider[position] -> init();
      case FXL6408_43:
        provider[position] = new IoProvider_FXL6408(0x43);
        return provider[position] -> init();
      default:
        return false;
      }
  }
  // returns the capabilities
  bool isDigitalInput(uint8_t port) {return provider[port / 32] -> isDigitalInput(port % 32); };
  bool isAnalogInput(uint8_t port) {return provider[port / 32] ->  isAnalogInput(port % 32);}
  bool isDigitalOutput(uint8_t port) {return provider[port / 32] ->  isDigitalOutput(port % 32);}
  bool isPwmOutput(uint8_t port) {return provider[port / 32] ->  isPwmOutput(port % 32);};
  bool isRawOutput(uint8_t port) {return provider[port / 32] ->  isRawOutput(port % 32);};

  // configure a port, default nothing enabled
  bool configureAsDigitalInput(uint8_t port, bool usePullUpDown, bool pullDirectionUp) {return provider[port / 32] ->  configureAsDigitalInput(port % 32, usePullUpDown, pullDirectionUp);}
  bool configureAsAnalogInput(uint8_t port) {return provider[port / 32] ->  configureAsAnalogInput(port % 32);}
  bool configureAsDigitalOutput(uint8_t port) {return provider[port / 32] ->  configureAsDigitalOutput(port % 32);}
  bool configureAsPwmOutput(uint8_t port, uint16_t frequency) {return provider[port / 32] ->  configureAsPwmOutput(port % 32, frequency);}
  bool configureAsRawIo(uint8_t port) {return provider[port / 32] ->  configureAsRawIo(port % 32);}

  // returns the pin number of a port on the microcontroler.
  // should only be used if necessary (e.g. setting up serial ports)
  // other application has to configure the pin
  uint8_t getRawIo(uint8_t port) {return provider[port / 32] -> getRawIo(port % 32);}

  // normal interactions during runtime
  bool getDigitalInput(uint8_t port) {return provider[port / 32] ->  getDigitalInput(port % 32);}
  uint16_t getAnalogInput(uint8_t port) {return provider[port / 32] ->  getAnalogInput(port % 32);}
  uint16_t getAnalogInputScaled(uint8_t port) {return provider[port / 32] ->  getAnalogInputScaled(port % 32);}
  void setDigitalOutput(uint8_t port, bool state) {provider[port / 32] -> setDigitalOutput(port % 32, state);}
  void setPwmOutput(uint8_t port, uint8_t dutyCycle) {provider[port / 32] -> setPwmOutput(port % 32, dutyCycle);}


  const string getPortName(uint8_t port) {return provider[port / 32] -> getPortName(port  % 32);}

private:
  // interfaces contains the different IO access providers.
  // currently only 4 "slots" (0-31, 32-63, 64-95, 96-127), can be extended
  // to 8 slots
  IoProvider *provider[4];
};
