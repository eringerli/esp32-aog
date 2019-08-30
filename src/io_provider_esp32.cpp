#include <stdint.h>
#include <string>
#include "io_provider.cpp"

class IoProvider_ESP32 : public IoProvider {
public:
  IoProvider_ESP32() {
  }
  bool init() {
    return true;
  }

private:
  std::string getName() const {return "ESP32"; }
};
