#include <ESPUI.h>

#include "main.hpp"
#include "gpsCommon.hpp"
#include "webUi.hpp"

int gpsCommonWebStatus;

AsyncUDP gpsCommonUdpSocket;

GpsRtcmData gpsRtcmData;
GpsNmeaOutput gpsNmeaOutput;

void gpsSendNmeaString(String data) {
  if (gpsNmeaOutput.udpOutput) {
    gpsCommonUdpSocket.broadcastTo( ( uint8_t* )data.c_str(), ( uint16_t )data.length(), 9999 );
  }
  if (gpsNmeaOutput.serialOutput) {
    rs232.print(data);
  }
  gpsNmeaOutput.lastSent = millis();
}

void gpsCommonInit() {
  // Status
  gpsCommonWebStatus = ESPUI.addControl( ControlType::Label, "Status:", "", ControlColor::Turquoise, webTabGPS );

  // nnmea outputs
  gpsNmeaOutput.udpOutput = preferences.getBool("gpsOutUdp");
  ESPUI.addControl( ControlType::Switcher, "UDP Output", String( (int)gpsNmeaOutput.udpOutput ) , ControlColor::Wetasphalt, webTabGPS,
    []( Control * control, int id ) {
      gpsNmeaOutput.udpOutput = (boolean)control->value.toInt();
      preferences.putBool("gpsOutUdp", gpsNmeaOutput.udpOutput);
      control->color = ControlColor::Carrot;
    } );
  gpsNmeaOutput.serialOutput = preferences.getBool("gpsOutSerial");
  ESPUI.addControl( ControlType::Switcher, "Serial Output", String( (int)gpsNmeaOutput.serialOutput ) , ControlColor::Wetasphalt, webTabGPS,
    []( Control * control, int id ) {
      gpsNmeaOutput.serialOutput = (boolean)control->value.toInt();
      preferences.putBool("gpsOutSerial", gpsNmeaOutput.serialOutput);
      control->color = ControlColor::Carrot;
    } );
  // init if neccessary
  if ( gpsNmeaOutput.udpOutput ) {
    if ( !gpsCommonUdpSocket.connected() && !gpsCommonUdpSocket.listen(gpsCommonPortOwn)) {
      usb.println ("ERROR: Starting UDP Listener for sending nmea over udp failed");
    }
  }
}

void gpsCommonStatus() {
  String str;
  str.reserve( 70 );

  str = "Last sent: ";
  if (gpsNmeaOutput.lastSent == 0) {
    str += "never<br />";
  } else {
    str += millis() - gpsNmeaOutput.lastSent;
    str += "ms<br />";
  }
  str += "Quality: ";
  switch ( gpsNmeaOutput.qpsquality ) {
    case GpsNmeaOutput::GpsQuality::none:
      str += "No GPS Fix";
      break;
    case GpsNmeaOutput::GpsQuality::gps:
      str += "GPS Fix";
      break;
    case GpsNmeaOutput::GpsQuality::dgps:
      str += "DGPS Fix";
      break;
    case GpsNmeaOutput::GpsQuality::fixedrtk:
      str += "RTK Fix";
      break;
    case GpsNmeaOutput::GpsQuality::floatrtk:
      str += "RTK Float";
      break;
    default:
      str += "?";
      break;
  }
  str += "<br />RTCM: ";
  switch ( gpsRtcmData.rtcmStatus ) {
    case GpsRtcmData::RtcmStatus::setup:
      str += "Setup";
      break;
    case GpsRtcmData::RtcmStatus::working:
      str += "Working";
      break;
    case GpsRtcmData::RtcmStatus::error:
      str += "Error";
      break;
    default:
      str += "?";
      break;
  }
  if (gpsCommonWebStatus != 0 ){
    Control* labelGpsStatus = ESPUI.getControl( gpsCommonWebStatus );
    labelGpsStatus->value = str;
    ESPUI.updateControl( labelGpsStatus );
  }
}
