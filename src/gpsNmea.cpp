#include "main.hpp"
#include "gpsNmea.hpp"
#include "gpsCommon.hpp"
#include <MicroNMEA.h>

constexpr size_t gpsNmeaBufferSize = 120;
char gpsNmeaBuffer[gpsNmeaBufferSize];
MicroNMEA nmea( gpsNmeaBuffer, gpsNmeaBufferSize );


void gpsNmeasingleReader() {
  xTaskCreate( gpsNmeasingleReaderTask, "GPS-singleNmea", 4096, NULL, 4, NULL );
}
void gpsNmeasingleReaderTask(void *z) {
  String sentence;
  sentence.reserve( gpsNmeaBufferSize );
  gpsNmeaOutput.lastGGA.reserve( gpsNmeaBufferSize );

  for ( ;; ) {
    uint16_t cnt = gps1.available();

    for ( uint16_t i = 0; i < cnt; i++ ) {
      if ( nmea.process( gps1.read() ) ) {
        if ( strcmp( nmea.getMessageID(), "GGA" ) == 0 ) {
          gpsNmeaOutput.lastGGA = nmea.getSentence();
          gpsNmeaOutput.qpsquality = (GpsNmeaOutput::GpsQuality)nmea.getQuality();
        }
        sentence = nmea.getSentence();
        sentence += "\r\n";
        gpsSendNmeaString(sentence);
      }
    }
    vTaskDelay( 10 );
  }
}
