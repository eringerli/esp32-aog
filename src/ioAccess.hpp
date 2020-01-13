#include <Arduino.h>
#include <Adafruit_ADS1015.h>

#ifndef ioAccess_HPP
#define ioAccess_HPP

  bool ioAccessInitAsDigitalOutput(uint8_t port);
  bool ioAccessInitAsDigitalInput(uint8_t port, bool usePullUpDown, bool pullDirectionUp);
  bool ioAccessInitAsAnalogInput(uint8_t port);
  bool ioAccessInitPwmChannel(uint8_t channel, uint frequency);
  bool ioAccessInitAttachToPwmChannel(uint8_t port, uint8_t channel);

  void ioAccessSetDigitalOutput(uint8_t port, bool value);
  void ioAccessSetPwmUtil(uint8_t channel, int dutyCycle);
  bool ioAccessGetDigitalInput(uint8_t port);
  float ioAccessGetAnalogInput(uint8_t port); // scaled to -1 to 1 (or 0-1 if no negative value is possible)

  bool ioAccess_FXL6408_init(uint8_t address) ;
  bool ioAccess_FXL6408_configureAsDigitalOutput(uint8_t address, uint8_t port);
  void ioAccess_FXL6408_setDigitalOutput(byte i2cAddress, uint8_t port, bool state);
  bool ioAccess_FXL6408_configureAsDigitalInput(byte i2cAddress, uint8_t port, bool usePullUpDown, bool pullDirectionUp);
  bool ioAccess_FXL6408_getDigitalInput(byte i2cAddress, uint8_t port);
  uint8_t ioAccess_FXL6408_setByteI2C(byte i2cAddress, byte i2cregister, byte value);
  uint8_t ioAccess_FXL6408_getByteI2C(byte i2cAddress, int i2cregister);

  extern Adafruit_ADS1115 ioAccess_FXL6408_Output[4];
  bool ioAccess_ads1115_init(uint8_t address) ;

  // helper for Webinterface, parameter is allways the parent element for the pulldown
  extern void (*ioAccessWebListAnalogIn)(int);
  extern void (*ioAccessWebListDigitalOut)(int);

  // helper for hwSetupF9PIoBoardMotor1
  extern void (*ioAccessMotor1)(int);

#endif
