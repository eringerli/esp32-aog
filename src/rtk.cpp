// MIT License
//
// Copyright (c) 2020 Christian Riggenbach
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <WiFi.h>
#include <WiFiMulti.h>

// #include <ESPAsyncTCP.h>
#include <vector>

#include <HTTPClient.h>

#include <ESPUI.h>

#include <MicroNMEA.h>

#include "jsonFunctions.hpp"
#include "main.hpp"

String lastGN;

constexpr size_t NmeaBufferSize = 120;
char             nmeaBuffer[NmeaBufferSize];
MicroNMEA        nmea( nmeaBuffer, NmeaBufferSize );

AsyncUDP udpGpsData;

AsyncServer*                       server;
static std::vector< AsyncClient* > clients;

static void
handleError( void* arg, AsyncClient* client, int8_t error ) {
  //   Serial.printf( "\n connection error %s from client %s \n", client->errorToString(
  //   error ), client->remoteIP().toString().c_str() );
}

static void
handleData( void* arg, AsyncClient* client, void* data, size_t len ) {
  Serial2.write( ( uint8_t* )data, len );
}

static void
handleDisconnect( void* arg, AsyncClient* client ) {
  //   Serial.printf( "\n client %s disconnected \n",
  //   client->remoteIP().toString().c_str() );

  // remove client from vector
  clients.erase(
    std::remove_if( clients.begin(),
                    clients.end(),
                    [client]( AsyncClient* itClient ) { return itClient == client; } ),
    clients.end() );
}

static void
handleTimeOut( void* arg, AsyncClient* client, uint32_t time ) {
  //   Serial.printf( "\n client ACK timeout ip: %s \n",
  //   client->remoteIP().toString().c_str() );
}
static void
handleNewClient( void* arg, AsyncClient* client ) {
  //   Serial.printf( "\n new client has been connected to server, ip: %s",
  //   client->remoteIP().toString().c_str() );

  // add to list
  clients.push_back( client );

  // register events
  client->onData( &handleData, NULL );
  client->onError( &handleError, NULL );
  client->onDisconnect( &handleDisconnect, NULL );
  client->onTimeout( &handleTimeOut, NULL );
}

static char receiveBuffer[400];

void
nmeaWorker( void* z ) {
  String sentence;
  sentence.reserve( NmeaBufferSize );
  lastGN.reserve( NmeaBufferSize );

  if( steerConfig.sendNmeaDataTcpPort != 0 ) {
    server = new AsyncServer( steerConfig.sendNmeaDataTcpPort );
    server->onClient( &handleNewClient, server );
    server->begin();
  }

  constexpr TickType_t xFrequency    = 10;
  TickType_t           xLastWakeTime = xTaskGetTickCount();

  for( ;; ) {
    uint16_t cnt =
      std::min( ( uint16_t )Serial2.available(), ( uint16_t )sizeof( receiveBuffer ) );

    //     Serial.print("nmeaWorker: ");
    //     Serial.println(cnt);

    if( cnt ) {
      for( uint16_t i = 0; i < cnt; ++i ) {
        receiveBuffer[i] = Serial2.read();
      }

      // send sentence to all connected clients on the TCP-Socket
      for( auto client : clients ) {
        // reply to client
        if( client->space() > cnt && client->canSend() ) {
          client->write( receiveBuffer, cnt );
          client->send();
        }
      }

      if( steerConfig.mode == SteerConfig::Mode::QtOpenGuidance &&
          steerConfig.qogChannelIdGpsDataOut != 0 ) {
        sendDataTransmission( steerConfig.qogChannelIdGpsDataOut, receiveBuffer, cnt );
      }

      switch( steerConfig.sendNmeaDataTo ) {
        case SteerConfig::SendNmeaDataTo::UDP: {
          udpGpsData.broadcastTo(
            ( uint8_t* )receiveBuffer, cnt, initialisation.sendNmeaDataUdpPort );
        } break;

        case SteerConfig::SendNmeaDataTo::Serial: {
          Serial.write( ( const uint8_t* )receiveBuffer, cnt );
        } break;

        case SteerConfig::SendNmeaDataTo::Serial1: {
          Serial1.write( ( const uint8_t* )receiveBuffer, cnt );
        } break;

        case SteerConfig::SendNmeaDataTo::Serial2: {
          Serial2.write( ( const uint8_t* )receiveBuffer, cnt );
        } break;

        default:
          break;
      }

      for( uint16_t i = 0; i < cnt; ++i ) {
        char c = receiveBuffer[i];

        if( nmea.process( c ) ) {
          if( strcmp( nmea.getMessageID(), "GGA" ) == 0 ) {
            lastGN = nmea.getSentence();
          }

          if( steerConfig.sendNmeaDataTo != SteerConfig::SendNmeaDataTo::None ) {
            sentence = nmea.getSentence();

            //           {
            //             gpsData.TOW;
            //             gpsData.vn;
            //             gpsData.ve;
            //             gpsData.vd;
            //             gpsData.lat;
            //             gpsData.lon;
            //             gpsData.alt;
            //           }

            //           // snap of the checksum, if it exists
            //           if ( uint8_t occurence = sentence.lastIndexOf( '*' ) > 0 ) {
            //             sentence.remove( occurence );
            //           }
            //
            //           // add the checksum
            //           char checksum[3] = {'\0'};
            //           nmea.generateChecksum( sentence.c_str(), checksum );
            //           sentence += "*";
            //           sentence += checksum;
            sentence += "\r\n";
          }
        }

        //       Serial.write( );
      }
    }

    {
      static uint8_t loopCounter = 0;

      static uint8_t cntTmp = 0;

      if( loopCounter++ >= ( 1000 / xFrequency ) ) {
        loopCounter = 0;

        String str;
        str.reserve( 200 );

        str += String( cntTmp++ );
        str += "<br/>";

        str +=
          "<table style='margin:auto;'><tr><td style='text-align:left; padding: 0px 5px;'>Lat:</td><td style='text-align:left; padding: 0px 5px;'>";
        str += String( ( float )nmea.getLatitude() / 1000000, 6 );
        str +=
          "</td></tr><tr><td style='text-align:left; padding: 0px 5px;'>Lon:</td><td style='text-align:left; padding: 0px 5px;'>";
        str += String( ( float )nmea.getLongitude() / 1000000, 6 );
        str +=
          "</td></tr><tr><td style='text-align:left; padding: 0px 5px;'>Alt:</td><td style='text-align:left; padding: 0px 5px;'>";
        str += ( float )nmea.getAltitude() / 1000;
        str +=
          "</td></tr><tr><td style='text-align:left; padding: 0px 5px;'>HDOP:</td><td style='text-align:left; padding: 0px 5px;'>";
        str += ( float )nmea.getHDOP() / 10;
        str +=
          "</td></tr><tr><td style='text-align:left; padding: 0px 5px;'>Age:</td><td style='text-align:left; padding: 0px 5px;'>";
        str += ( float )nmea.getAgeOfDGPS() / 10;
        str +=
          "</td></tr><tr><td style='text-align:left; padding: 0px 5px;'>Quality:</td><td style='text-align:left; padding: 0px 5px;'>";

        switch( nmea.getQuality() ) {
          case 0:
            str += "No GPS Fix</td></tr></table>";
            break;

          case 1:
            str += "GPS Fix</td></tr></table>";
            break;

          case 2:
            str += "DGPS Fix</td></tr></table>";
            break;

          case 3:
            str += "PPS Fix</td></tr></table>";
            break;

          case 4:
            str += "RTK Fix</td></tr></table>";
            break;

          case 5:
            str += "RTK Float</td></tr></table>";
            break;

          default:
            str += "?</td></tr></table>";
            break;
        }

        labelStatusGps->value = str;
        ESPUI.updateControlAsync( labelStatusGps );
      }
    }

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void
ntripWorker( void* z ) {
  vTaskDelay( 2000 );

  initialisation.rtkCorrectionURL.reserve( 200 );
  initialisation.rtkCorrectionURL = "http://";

  if( *( steerConfig.rtkCorrectionUsername ) != '\0' ) {
    initialisation.rtkCorrectionURL += steerConfig.rtkCorrectionUsername;

    if( *( steerConfig.rtkCorrectionPassword ) != '\0' ) {
      initialisation.rtkCorrectionURL += ":";
      initialisation.rtkCorrectionURL += steerConfig.rtkCorrectionPassword;
    }

    initialisation.rtkCorrectionURL += "@";
  }

  initialisation.rtkCorrectionURL += steerConfig.rtkCorrectionServer;

  if( steerConfig.rtkCorrectionPort != 0 ) {
    initialisation.rtkCorrectionURL += ":";
    initialisation.rtkCorrectionURL += steerConfig.rtkCorrectionPort;
  }

  initialisation.rtkCorrectionURL += "/";

  if( *( steerConfig.rtkCorrectionMountpoint ) != '\0' ) {
    initialisation.rtkCorrectionURL += steerConfig.rtkCorrectionMountpoint;
  }

  if( initialisation.rtkCorrectionURL.length() <= 8 ) {
    // update WebUI
    {
      labelStatusNtrip->value = "Cannot connect to " + initialisation.rtkCorrectionURL;
      labelStatusNtrip->color = ControlColor::Carrot;
      ESPUI.updateControlAsync( labelStatusNtrip );
    }

    // delete this task
    TaskHandle_t myself = xTaskGetCurrentTaskHandle();
    vTaskDelete( myself );

    return;
  }

  for( ;; ) {
    HTTPClient http;
    http.begin( initialisation.rtkCorrectionURL );
    http.setUserAgent( "NTRIP Esp32NTRIPClient" );
    int httpCode = http.GET();

    if( httpCode > 0 ) {
      // HTTP header has been send and Server response header has been handled

      // file found at server
      if( httpCode == HTTP_CODE_OK ) {
        // update WebUI
        {
          labelStatusNtrip->value = "Connected to " + initialisation.rtkCorrectionURL;
          labelStatusNtrip->color = ControlColor::Emerald;
          ESPUI.updateControlAsync( labelStatusNtrip );
        }

        // create buffer for read
        constexpr uint16_t buffSize = 1436;
        uint8_t*           buff     = ( uint8_t* )malloc( buffSize );

        // get tcp stream
        WiFiClient* stream = http.getStreamPtr();

        time_t timeoutSendGGA =
          millis() + ( steerConfig.ntripPositionSendIntervall * 1000 );

        // read all data from server
        while( http.connected() ) {
          // get available data size
          size_t size = stream->available();

          if( size ) {
            int c = stream->readBytes( buff, ( ( size > buffSize ) ? buffSize : size ) );

            // write it to Serial
            Serial2.write( buff, c );
          }

          if( millis() > timeoutSendGGA ) {
            String nmeaToSend;
            nmeaToSend.reserve( sizeof( SteerConfig::rtkCorrectionNmeaToSend ) );

            if( steerConfig.rtkCorrectionNmeaToSend[0] != '\0' ) {
              nmeaToSend = steerConfig.rtkCorrectionNmeaToSend;
            } else {
              nmeaToSend = lastGN;
            }

            if( nmeaToSend.length() ) {
              // calculate checksum if not correct
              if( !nmea.testChecksum( nmeaToSend.c_str() ) ) {
                // snap off the checksum, if it exists
                {
                  uint8_t occurence = nmeaToSend.lastIndexOf( "*" );

                  if( occurence > 0 ) {
                    nmeaToSend.remove( occurence );
                  }
                }

                // add the checksum
                char checksum[] = { '*', '\0', '\0', '\0' };
                nmea.generateChecksum( nmeaToSend.c_str(), &checksum[1] );
                nmeaToSend += checksum;

                // update checksum, also in the WebUI
                if( steerConfig.rtkCorrectionNmeaToSend[0] != '\0' ) {
                  nmeaToSend.toCharArray( steerConfig.rtkCorrectionNmeaToSend,
                                          sizeof( steerConfig.rtkCorrectionNmeaToSend ) );

                  {
                    textNmeaToSend->value.reserve( 80 );
                    textNmeaToSend->value = steerConfig.rtkCorrectionNmeaToSend;
                    ESPUI.updateControlAsync( textNmeaToSend );
                  }
                }
              }

              if( nmeaToSend.lastIndexOf( '\n' ) == -1 ) {
                nmeaToSend += "\r\n";
              }

              stream->write( nmeaToSend.c_str(), nmeaToSend.length() );
            }

            timeoutSendGGA = millis() + ( steerConfig.ntripPositionSendIntervall * 1000 );
          }

          vTaskDelay( 1 );
        }

        free( buff );
      }
    }

    // update WebUI
    {
      textNmeaToSend->value = "Cannot connect to " + initialisation.rtkCorrectionURL;
      textNmeaToSend->color = ControlColor::Carrot;
      ESPUI.updateControlAsync( textNmeaToSend );
    }

    http.end();
    vTaskDelay( 1000 );
  }

  // delete this task
  TaskHandle_t myself = xTaskGetCurrentTaskHandle();
  vTaskDelete( myself );
}

void
initRtkCorrection() {
  if( steerConfig.sendNmeaDataUdpPort != 0 ) {
    initialisation.sendNmeaDataUdpPort = steerConfig.sendNmeaDataUdpPort;
  }

  if( steerConfig.sendNmeaDataUdpPortFrom != 0 ) {
    udpGpsData.listen( steerConfig.sendNmeaDataUdpPortFrom );
  }

  Serial2.begin( steerConfig.rtkCorrectionBaudrate, SERIAL_8N1, TX, RX );

  if( steerConfig.rtkCorrectionType == SteerConfig::RtkCorrectionType::Ntrip ) {
    xTaskCreate( ntripWorker, "ntripWorker", 3064, NULL, 8, NULL );
  }

  xTaskCreate( nmeaWorker, "nmeaWorker", 2048, NULL, 6, NULL );
}
