#include <stdint.h>
#include <string>
#include "io_provider.cpp"

class IoProvider_ADS115 : public IoProvider {
public:
  IoProvider_ADS115(uint8_t address) {
    i2cAddress = address;
  }
  bool init() {
    return false;
  }

private:
  uint8_t i2cAddress;
  std::string getName() const {return "ADS1115"; }
};
