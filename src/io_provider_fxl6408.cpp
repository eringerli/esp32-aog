#include <stdint.h>
#include <string>
#include "io_provider.cpp"

class IoProvider_FXL6408 : public IoProvider {
public:
  IoProvider_FXL6408(uint8_t address) {
    i2cAddress = address;
  }
  bool init() {
    return false;
  }

private:
  uint8_t i2cAddress;
  std::string getName() const {return "FXL6408"; }
};
