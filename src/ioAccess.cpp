#include "main.hpp"
#include "ioAccess.hpp"

/*
##
## IO Ports
##
Port 0-40 reserved for the respective ports on an esp32
Port 41-44 ADS1115 Adress 0x48 Port 0-3
Port 45 ADS1115 Adress 0x48 Differential Port 0/1
Port 46 ADS1115 Adress 0x48 Differential Port 0/3
Port 47 ADS1115 Adress 0x48 Differential Port 1/3
Port 48 ADS1115 Adress 0x48 Differential Port 2/3
Port 47-72 reserved for other possible ADS adresses
Port 73-80 FXL6408 Address 0x43
Port 81-88 FXL6408 Address 0x44

Port 253 has special meaning, always false/0.0
Port 254 has special meaning, always true/1.0
Port 255 has special meaning (not configured)
##
## PWM Channel
##
Channel 0-15 ESP32
*/

Adafruit_ADS1115 ioAccess_ads1115[4];
void (*ioAccessWebListAnalogIn)(int);
void (*ioAccessWebListDigitalOut)(int);
void (*ioAccessMotor1)(int);

bool ioAccessInitAsDigitalOutput(uint8_t port) {
  switch (port) {
    case 0:
    case 2:
    case 4 ... 5:
    case 12 ... 33: {
        // all "normal" ESP32 Outputs, excluded serial to usb even if in theory usable
        pinMode(port, OUTPUT);
        gpio_num_t espPort = static_cast<gpio_num_t>(port);
        gpio_pad_select_gpio(espPort);
        gpio_intr_disable(espPort);
        gpio_set_pull_mode(espPort, GPIO_FLOATING);
        gpio_set_direction(espPort, GPIO_MODE_OUTPUT);
      };
      return true;
      break;
    case 73 ... 88:
      return ioAccess_FXL6408_configureAsDigitalOutput((0x43 + (port - 73) / 8), ((port - 73) % 8));
      break;
    default:
      return false;
  }
}

void ioAccessSetDigitalOutput(uint8_t port, bool value) {
  switch (port) {
    case 0:
    case 2:
    case 4 ... 5:
    case 12 ... 33: {
        // all "normal" ESP32 Outputs, excluded serial to usb even if in theory usable
        if (value) {
          digitalWrite(static_cast<gpio_num_t>(port), HIGH);
        } else {
          digitalWrite(static_cast<gpio_num_t>(port), LOW);
        }
      };
      break;
    case 73 ... 88:
      return ioAccess_FXL6408_setDigitalOutput((0x43 + (port - 73) / 8), ((port - 73) % 8), value);
      break;
  }
}

bool ioAccessInitAsDigitalInput(uint8_t port, bool usePullUpDown, bool pullDirectionUp) {
   switch (port) {
    case 0:
    case 2:
    case 4 ... 5:
    case 12 ... 39: {
        // all "normal" ESP32 Inputs, excluded serial to usb even if in theory usable
        pinMode(port, INPUT);
        gpio_num_t espPort = static_cast<gpio_num_t>(port);
        gpio_pad_select_gpio(espPort);
        gpio_set_direction(espPort, GPIO_MODE_INPUT);
        gpio_intr_disable(espPort);
        if (usePullUpDown) {
          if (pullDirectionUp) {
            gpio_set_pull_mode(espPort, GPIO_PULLUP_ONLY);
          } else {
            gpio_set_pull_mode(espPort, GPIO_PULLDOWN_ONLY);
          }
        } else {
          gpio_set_pull_mode(espPort, GPIO_FLOATING);
        }
      };
      return true;
      break;
    case 73 ... 88:
      return ioAccess_FXL6408_configureAsDigitalInput((0x43 + (port - 73) / 8), ((port - 73) % 8), usePullUpDown, pullDirectionUp);
      break;
    default:
      return false;
  }
}

bool ioAccessInitAsAnalogInput(uint8_t port) {
  switch (port) {
   case 32 ... 36:
   case 39:
       // all "normal" ESP32 Inputs, excluded serial to usb even if in theory usable
       pinMode(port, INPUT);
       // set everytime, just to be sure
       analogReadResolution(10); // Default of 12 is not very linear. Recommended to use 10 or 11 depending on needed resolution.
       analogSetAttenuation(ADC_11db); // Default is 11db which is very noisy. But needed for full scale range  Recommended to use 2.5 or 6.
     return true;
     break;
   case 41 ... 72:
     // just assume has been set up, no additionla init necessary
     return true;
     break;
   default:
     return false;
 }
}

bool ioAccessInitPwmChannel(uint8_t channel, uint frequency){
  switch (channel) {
    case 0 ... 15:
      ledcSetup(channel, frequency, 10);
      ledcWrite(channel, 0);
      return true;
      break;
    default:
      return false;
  }
}
bool ioAccessInitAttachToPwmChannel(uint8_t port, uint8_t channel){
  switch (port){
    case 0:
    case 2:
    case 4 ... 5:
    case 12 ... 33: {
        // check if pwm channel is possible
        if (channel < 0 || channel > 15) {
          return false;
        }
        // all "normal" ESP32 Outputs, excluded serial to usb even if in theory usable
        gpio_num_t espPort = static_cast<gpio_num_t>(port);
        gpio_pad_select_gpio(espPort);
        gpio_intr_disable(espPort);
        gpio_set_pull_mode(espPort, GPIO_FLOATING);
        gpio_set_direction(espPort, GPIO_MODE_OUTPUT);
        pinMode(port, OUTPUT);
        ledcAttachPin(port, channel);
      };
      return true;
      break;
    default:
      return false;
  }}
void ioAccessSetPwmUtil(uint8_t channel, int dutyCycle){
  switch (channel) {
    case 0 ... 15:
      ledcWrite(channel, dutyCycle);
      break;
  }
}


bool ioAccessGetDigitalInput(uint8_t port){
  switch (port) {
    case 0:
    case 2:
    case 4 ... 5:
    case 12 ... 39:
      return digitalRead(port);
      break;
    case 73 ... 88:
      return ioAccess_FXL6408_getDigitalInput((0x43 + (port - 73) / 8), ((port - 73) % 8));
      break;
    case 253:
      return false;
      break;
    case 254:
      return true;
      break;
    default:
      return false;
  }
}
float ioAccessGetAnalogInput(uint8_t port){
  switch (port) {
    case 32 ... 36:
    case 39:
      return analogRead(port)/1023.0;
      break;
    case 41 ... 72:
      {
        uint8_t adsNumber = (port - 41) / 8;
        uint8_t diffCombination = (port - 41) % 8;
        switch (diffCombination) {
          case 0 ... 3: {
              if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
                float value = ioAccess_ads1115[adsNumber].readADC_SingleEnded(diffCombination) / 32768.0;
                xSemaphoreGive( i2cMutex );
                return value;
              }
            }
            break;
          case 4: {
              if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
                float value = ioAccess_ads1115[adsNumber].readADC_Differential_0_1() / 32768.0;
                xSemaphoreGive( i2cMutex );
                return value;
              }
            }
            break;
          case 5: {
              if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
                float value = ioAccess_ads1115[adsNumber].readADC_Differential_0_3() / 32768.0;
                xSemaphoreGive( i2cMutex );
                return value;
              }
            }
            break;
          case 6: {
              if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
                float value = ioAccess_ads1115[adsNumber].readADC_Differential_1_3() / 32768.0;
                xSemaphoreGive( i2cMutex );
                return value;
              }
            }
            break;
          case 7: {
              if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
                float value = ioAccess_ads1115[adsNumber].readADC_Differential_2_3() / 32768.0;
                xSemaphoreGive( i2cMutex );
                return value;
              }
            }
            break;
          default:
            return -3;
        }
      }
      break;
    case 253:
      return 0.0;
      break;
    case 254:
      return 1.0;
      break;
    default:
      return -2;
  }
  return -5;
}


//helper
uint8_t setBit(uint8_t byte, uint8_t position, bool value) {
  uint8_t pattern = 0b00000001 << position;
  if (value) {
    return byte | pattern;
  }  else {
    pattern = ~pattern;
    return byte & pattern;
  }
}


// FXL6408
bool ioAccess_FXL6408_init(uint8_t address) {
  int returnValues = 0;
  returnValues += ioAccess_FXL6408_setByteI2C(address, 0x07, 0b11111111); // Output High-Z (not driven)
  returnValues += ioAccess_FXL6408_setByteI2C(address, 0x03, 0b11111111); // Everything Output
  returnValues += ioAccess_FXL6408_setByteI2C(address, 0x05, 0b00000000); // (Disabled) Outputs to low)
  returnValues += ioAccess_FXL6408_setByteI2C(address, 0x0B, 0b00000000); // No Pullup/down
  returnValues += ioAccess_FXL6408_setByteI2C(address, 0x11, 0b11111111); // No interrupts
  return returnValues == 0;
}

bool ioAccess_FXL6408_configureAsDigitalOutput(uint8_t address, uint8_t port) {
    int returnValues = 0;
    // default low
    ioAccess_FXL6408_setDigitalOutput(address, port, false);
    // disable High-Z
    returnValues += ioAccess_FXL6408_setByteI2C(address, 0x07, setBit(ioAccess_FXL6408_getByteI2C(address, 0x07), port, false));
    // direction
    returnValues += ioAccess_FXL6408_setByteI2C(address, 0x03, setBit(ioAccess_FXL6408_getByteI2C(address, 0x03), port, true));

    return returnValues == 0;
  }

void ioAccess_FXL6408_setDigitalOutput(byte i2cAddress, uint8_t port, bool state) {
  static uint8_t ioAccess_FXL6408_Output[2];
  uint8_t oldRegister = ioAccess_FXL6408_Output[i2cAddress - 0x43];
  ioAccess_FXL6408_Output[i2cAddress - 0x43] = setBit(ioAccess_FXL6408_Output[i2cAddress - 0x43], port, state);
  if (oldRegister != ioAccess_FXL6408_Output[i2cAddress - 0x43]) {
    ioAccess_FXL6408_setByteI2C(i2cAddress, 0x05, ioAccess_FXL6408_Output[i2cAddress - 0x43]);
  }
};

bool ioAccess_FXL6408_configureAsDigitalInput(byte i2cAddress, uint8_t port, bool usePullUpDown, bool pullDirectionUp) {
  int returnValues = 0;
  // pullUp/Down
  returnValues += ioAccess_FXL6408_setByteI2C(i2cAddress, 0x0D, setBit(ioAccess_FXL6408_getByteI2C(i2cAddress, 0x0D), port, pullDirectionUp));
  returnValues += ioAccess_FXL6408_setByteI2C(i2cAddress, 0x0B, setBit(ioAccess_FXL6408_getByteI2C(i2cAddress, 0x0B), port, usePullUpDown));
  // direction
  returnValues += ioAccess_FXL6408_setByteI2C(i2cAddress, 0x03, setBit(ioAccess_FXL6408_getByteI2C(i2cAddress, 0x03), port, false));

  return returnValues == 0;
}

bool ioAccess_FXL6408_getDigitalInput(byte i2cAddress, uint8_t port)  {
   uint8_t value = ioAccess_FXL6408_getByteI2C(i2cAddress, 0xF);
   value = (value >> port) & 1; // shift so the port is at last bit, then mask
   return value == 1;
 }

  uint8_t ioAccess_FXL6408_getByteI2C(byte i2cAddress, int i2cregister) {
    uint8_t result = -1;
    if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
      Wire.beginTransmission(i2cAddress);
      Wire.write(i2cregister);
      Wire.endTransmission(false);
      Wire.requestFrom(i2cAddress, 1, (int)true);
      result = Wire.read();
      xSemaphoreGive( i2cMutex );
    }
    return result;
  }

  uint8_t ioAccess_FXL6408_setByteI2C(byte i2cAddress, byte i2cregister, byte value) {
      uint8_t result = 255;
      if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
        Wire.beginTransmission(i2cAddress);
        Wire.write(i2cregister);
        Wire.write(value);
        result = Wire.endTransmission();
        xSemaphoreGive( i2cMutex );
      }
      return result;
    }


// ADS1115

bool ioAccess_ads1115_init(uint8_t address) {
  // really basic init check, does someone respond on this Address
  if ( xSemaphoreTake( i2cMutex, 1000 ) == pdTRUE ) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      ioAccess_ads1115[ address - 0x48 ] = Adafruit_ADS1115( address );
      ioAccess_ads1115[ address - 0x48 ].setGain( GAIN_TWOTHIRDS );   // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
      ioAccess_ads1115[ address - 0x48 ].begin();
      ioAccess_ads1115[ address - 0x48 ].setSPS( ADS1115_DR_860SPS );
      xSemaphoreGive( i2cMutex );
      return true;
    }
    xSemaphoreGive( i2cMutex );
  }
  return false;
}
