#include "main.hpp"
#include <Preferences.h>
#include <ESPUI.h>
#include <HTTPClient.h>
#include "gpsRtcm.hpp"
#include "webUi.hpp"


void gpsRtcmSetup(GpsRtcmData::RtcmDestination rtcmdestination) {
  gpsRtcmData.rtcmSource = (GpsRtcmData::RtcmSource)preferences.getUChar("gpsRtcmSource",0);
  gpsRtcmData.rtcmDestination = rtcmdestination;

  // web
  uint16_t sel = ESPUI.addControl( ControlType::Select, "RTCM source", String( (int)gpsRtcmData.rtcmSource ) , ControlColor::Wetasphalt, webTabGPS,
    []( Control * control, int id ) {
      preferences.putUChar("gpsRtcmSource", control->value.toInt());
      webChangeNeedsReboot();
    } );
  ESPUI.addControl( ControlType::Option, "None", "0", ControlColor::Alizarin, sel );
  ESPUI.addControl( ControlType::Option, "UDP", "1", ControlColor::Alizarin, sel );
  ESPUI.addControl( ControlType::Option, "Ntrip", "3", ControlColor::Alizarin, sel );
  // only ntrip needs more options
  if (gpsRtcmData.rtcmSource == GpsRtcmData::RtcmSource::Ntrip) {
    ESPUI.addControl( ControlType::Text, "Ntrip Server", preferences.getString("gpsNtripServer", ""), ControlColor::Wetasphalt, webTabGPS,
      []( Control * control, int id ) {
        preferences.putString("gpsNtripServer", control->value);
        control->color = ControlColor::Carrot;
        ESPUI.updateControl( control );
        webChangeNeedsReboot();
      } );
    int control = ESPUI.addControl( ControlType::Number, "Ntrip port", String(preferences.getUInt("gpsNtripPort", 2101)), ControlColor::Wetasphalt, webTabGPS,
      []( Control * control, int id ) {
        preferences.putUInt("gpsNtripPort", control->value.toInt());
        control->color = ControlColor::Carrot;
        ESPUI.updateControl( control );
        webChangeNeedsReboot();
      } );
    ESPUI.addControl( ControlType::Min, "Min", "1", ControlColor::Peterriver, control );
    ESPUI.addControl( ControlType::Max, "Max", "65535", ControlColor::Peterriver, control );
    ESPUI.addControl( ControlType::Step, "Step", "1", ControlColor::Peterriver, control );
    ESPUI.addControl( ControlType::Text, "Ntrip Mountpoint", preferences.getString("gpsNtripMount", ""), ControlColor::Wetasphalt, webTabGPS,
      []( Control * control, int id ) {
        preferences.putString("gpsNtripMount", control->value);
        control->color = ControlColor::Carrot;
        ESPUI.updateControl( control );
        webChangeNeedsReboot();
      } );
    ESPUI.addControl( ControlType::Text, "Ntrip user", preferences.getString("gpsNtripUser", ""), ControlColor::Wetasphalt, webTabGPS,
      []( Control * control, int id ) {
        preferences.putString("gpsNtripUser", control->value);
        control->color = ControlColor::Carrot;
        ESPUI.updateControl( control );
        webChangeNeedsReboot();
      } );
    ESPUI.addControl( ControlType::Text, "Ntrip password", preferences.getString("gpsNtripPw", ""), ControlColor::Wetasphalt, webTabGPS,
      []( Control * control, int id ) {
        preferences.putString("gpsNtripPw", control->value);
        control->color = ControlColor::Carrot;
        ESPUI.updateControl( control );
        webChangeNeedsReboot();
      } );
    ESPUI.addControl( ControlType::Number, "Ntrip GGA interval", String(preferences.getUInt("gpsNtripGga", 30)), ControlColor::Wetasphalt, webTabGPS,
      []( Control * control, int id ) {
        preferences.putUInt("gpsNtripGga", control->value.toInt());
        control->color = ControlColor::Carrot;
        ESPUI.updateControl( control );
        webChangeNeedsReboot();
      } );
  }
  // init source
  switch (gpsRtcmData.rtcmSource) {
    case GpsRtcmData::RtcmSource::none:
      // no source, nothing to do
      break;
    case GpsRtcmData::RtcmSource::UDP:
      if ( !gpsCommonUdpSocket.connected() && !gpsCommonUdpSocket.listen(gpsCommonPortOwn)) {
        usb.println("ERROR: Starting UDP Listener for rtcm failed");
      }
      gpsRtcmCreateUdpReceiveHandler();
      break;
    case GpsRtcmData::RtcmSource::Ntrip:
      xTaskCreate( gpsRtcmNtripReceiver, "Ntrip Receiver", 4096, NULL, 4, NULL );
      break;
  }
}

void gpsRtcmCreateUdpReceiveHandler() {
  usb.println("Creating handler for RTCM Data");
  gpsCommonUdpSocket.onPacket([](AsyncUDPPacket packet) {
      gpsRtcmData.rtcmStatus = GpsRtcmData::RtcmStatus::working;
      while (packet.peek() != -1) {
        byte data = (byte)packet.read();
        if ( gpsRtcmData.rtcmDestination == GpsRtcmData::RtcmDestination::gps1 || gpsRtcmData.rtcmDestination == GpsRtcmData::RtcmDestination::both ) {
          gps1.write(data);
        }
        if ( gpsRtcmData.rtcmDestination == GpsRtcmData::RtcmDestination::gps2 || gpsRtcmData.rtcmDestination == GpsRtcmData::RtcmDestination::both ) {
          gps2.write(data);
        }
      }
    }
  );
}

void gpsRtcmNtripReceiver( void* z ) {
  vTaskDelay( 1000 );

  String rtkCorrectionURL;
  rtkCorrectionURL.reserve( 200 );
  rtkCorrectionURL = "http://";
  if ( preferences.getString("gpsNtripUser", "").length() > 1 ) {
    rtkCorrectionURL += preferences.getString("gpsNtripUser", "");
    rtkCorrectionURL += ":";
    rtkCorrectionURL += preferences.getString("gpsNtripPw", "");
    rtkCorrectionURL += "@";
  }

  rtkCorrectionURL += preferences.getString("gpsNtripServer", "");

  rtkCorrectionURL += ":";
  rtkCorrectionURL += preferences.getUInt("gpsNtripPort", 2101);

  rtkCorrectionURL += "/";
  rtkCorrectionURL += preferences.getString("gpsNtripMount", "");
  if ( rtkCorrectionURL.length() <= 8 ) {
    usb.println("Abort ntrip client, too short url");
    gpsRtcmData.rtcmStatus = GpsRtcmData::RtcmStatus::error;
    // delete this task
    TaskHandle_t myself = xTaskGetCurrentTaskHandle();
    vTaskDelete( myself );

    return;
  }

  // loop
  for ( ;; ) {
    HTTPClient http;
    http.begin( rtkCorrectionURL );
    http.setUserAgent( "NTRIP CoffeetracNTRIPClient" );
    int httpCode = http.GET();
    uint ggaInterval = preferences.getUInt("gpsNtripGga", 30) * 1000;

    if ( httpCode > 0 ) {
      // HTTP header has been send and Server response header has been handled

      // file found at server
      if ( httpCode == HTTP_CODE_OK ) {
        gpsRtcmData.rtcmStatus = GpsRtcmData::RtcmStatus::working;
        // create buffer for read
        constexpr uint16_t buffSize = 1436;
        uint8_t* buff = ( uint8_t* )malloc( buffSize );

        // get tcp stream
        WiFiClient* stream = http.getStreamPtr();

        time_t timeoutSendGGA = millis() + ggaInterval;

        // read all data from server
        while ( http.connected() ) {
          // get available data size
          size_t size = stream->available();

          if ( size ) {
            int c = stream->readBytes( buff, ( ( size > buffSize ) ? buffSize : size ) );

            // write it to gps
            if ( gpsRtcmData.rtcmDestination == GpsRtcmData::RtcmDestination::gps1 || gpsRtcmData.rtcmDestination == GpsRtcmData::RtcmDestination::both ) {
              gps1.write( buff, c );
            }
            if ( gpsRtcmData.rtcmDestination == GpsRtcmData::RtcmDestination::gps2 || gpsRtcmData.rtcmDestination == GpsRtcmData::RtcmDestination::both ) {
              gps2.write( buff, c );
            }
          }
          // send own position to ntrip server
          if ( millis() > timeoutSendGGA ) {
            if ( gpsNmeaOutput.lastGGA.length() > 10 ) {
              if ( gpsNmeaOutput.lastGGA.lastIndexOf( '\n' ) == -1 ) {
                gpsNmeaOutput.lastGGA += "\r\n";
              }
              stream->write( gpsNmeaOutput.lastGGA.c_str(), gpsNmeaOutput.lastGGA.length() );
            }

            timeoutSendGGA = millis() + ggaInterval;
          }
          vTaskDelay( 3 );
        }
      }
    }

    // update WebUI
    gpsRtcmData.rtcmStatus = GpsRtcmData::RtcmStatus::error;

    http.end();
    vTaskDelay( 1000 );
  }

// delete this task
  TaskHandle_t myself = xTaskGetCurrentTaskHandle();
  vTaskDelete( myself );
}
