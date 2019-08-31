#include <stdint.h>
#include <string>

enum class IoProviders : uint8_t {
  None = 0,
  ESP32_IO = 1,
  ADS1115_48 = 8,
  FXL6408_43 = 16
} ;

class IoProvider {

public:
  // take care of (necessary) initialisation
  virtual bool init() = 0;

  // returns the capabilities
  bool isDigitalInput(uint8_t port) const {return ports[port].digitalInput;};
  bool isAnalogInput(uint8_t port) const {return ports[port].analogInput;};
  bool isDigitalOutput(uint8_t port) const {return ports[port].digitalOutput;};
  bool isPwmOutput(uint8_t port) const {return ports[port].pwmOutput;};
  bool israwIo(uint8_t port) const {return ports[port].rawIo;};

  // configure a port, default nothing enabled
  bool configureAsDigitalInput(uint8_t port, bool usePullUpDown, bool pullDirectionUp) {return false;}
  bool configureAsAnalogInput(uint8_t port) {return false;}
  bool configureAsDigitalOutput(uint8_t port) {return false;}
  bool configureAsPwmOutput(uint8_t port, uint16_t frequency) {return false;}
  bool configureAsRawIo(uint8_t port) {return false;}

  // returns the pin number of a port on the microcontroler.
  // should only be used if necessary (e.g. setting up serial ports)
  // other application has to configure the pin
  uint8_t getRawIo(uint8_t port)  const {
    // for all except ESP32 always invalid
    return 255;
  }

  // normal interactions during runtime
  bool getDigitalInput(uint8_t port)  const {return false;}
  uint16_t getAnalogInput(uint8_t port)  const {return UINT16_MAX;}
  uint16_t getAnalogInputScaled(uint8_t port)  const {return UINT16_MAX;}
  void setDigitalOutput(uint8_t port, bool state) {};
  void setPwmOutput(uint8_t port, uint8_t dutyCycle) {};


  const std::string getPortName(uint8_t port) const {
    std::string name = getName();
    name.append(" ");
    name.append(ports[port].humanName);
    return name;
  }


protected:
  struct portDefinition {
    std::string humanName;
    bool digitalInput;
    bool analogInput;
    bool digitalOutput;
    bool pwmOutput;
    bool rawIo;
    portDefinition(){};
    portDefinition(std::string name, bool digitalIn, bool analogIn, bool digitalOut, bool pwmOut, bool raw ){
      humanName = name;
      digitalInput = digitalIn;
      analogInput = analogIn;
      digitalOutput = digitalOut;
      pwmOutput = pwmOut;
      rawIo = raw;
    };
  };

  enum portState {
    unconfigured,
    digitalInput,
    analogInput,
    digitalOutput,
    pwmOutput,
    rawIo,
    disabled
  };  portState configuration[32];


private:
  virtual std::string getName() const = 0;
  static const portDefinition ports[32];
};
