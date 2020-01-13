#ifndef gpsCommon_HPP
#define gpsCommon_HPP

#include <AsyncUDP.h>

extern AsyncUDP gpsCommonUdpSocket;
constexpr uint gpsCommonPortDataToAog = 9999;
constexpr uint gpsCommonPortOwn = 5588;

extern int gpsCommonWebStatus;

struct GpsRtcmData {
  enum class RtcmSource : uint8_t {
    none = 0,
    UDP = 1,
    Ntrip = 3
  } rtcmSource = RtcmSource::none;

  enum class RtcmStatus : uint8_t {
    setup = 0,
    working = 1,
    error = 2,
  } rtcmStatus = RtcmStatus::setup;

  enum class RtcmDestination : uint8_t {
    gps1 = 0,
    gps2 = 1,
    both = 2,
  } rtcmDestination = RtcmDestination::gps1;
  uint lastReceived = 0;
};
extern GpsRtcmData gpsRtcmData;

struct GpsNmeaOutput {
  bool udpOutput = false;
  bool serialOutput = false;

  uint lastSent = 0;
  enum class GpsQuality : uint8_t {
    none = 0,
    gps = 1,
    dgps = 2,
    floatrtk = 5,
    fixedrtk = 4,
  } qpsquality = GpsQuality::none;

  String lastGGA;
};
extern GpsNmeaOutput gpsNmeaOutput;

void gpsSendNmeaString(String data);
void gpsCommonStatus();
void gpsCommonInit();
#endif
