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

#include "main.hpp"

#include "../lib/cbor-lite/include/cbor-lite/codec-fp.h"

#pragma once

extern void loadSavedConfig();
extern void saveConfig();

extern DynamicJsonDocument loadJsonFromFile( const char* fileName );
extern void saveJsonToFile( const DynamicJsonDocument& doc, const char* fileName );

extern void parseJsonToSteerConfig( DynamicJsonDocument& doc, SteerConfig& config );
extern DynamicJsonDocument parseSteerConfigToJson( const SteerConfig& config );

template< typename T >
T*
startAccGyrMagTransmission( uint16_t channelId ) {
  auto* buffer = new std::vector< std::uint8_t >;
  buffer->reserve( 1500 );
  CborLite::encodeMapSize( *buffer, uint8_t( 2 ) );
  CborLite::encodeText( *buffer, std::string( "cid" ) );
  CborLite::encodeUnsigned( *buffer, channelId );
  CborLite::encodeText( *buffer, std::string( "imu" ) );
  CborLite::encodeArrayIndefinite( *buffer );

  return buffer;
}

template< typename T >
void
addAccGyrMagTransmission( T& buffer, ImuDataAccGyrMag data ) {
  CborLite::encodeMapSize( buffer, uint8_t( 2 ) );

  CborLite::encodeText( buffer, std::string( "ts" ) );
  CborLite::encodeUnsigned( buffer, micros() );

  CborLite::encodeText( buffer, std::string( "agm" ) );
  CborLite::encodeArraySize( buffer, uint8_t( 9 ) );

  for( const auto& i : data.acc ) {
    CborLite::encodeSingleFloat( buffer, i );
  }
  for( const auto& i : data.gyr ) {
    CborLite::encodeSingleFloat( buffer, i );
  }
  for( const auto& i : data.mag ) {
    CborLite::encodeSingleFloat( buffer, i );
  }
}

template< typename T >
void
addAccGyrTransmission( T& buffer, ImuDataAccGyrMag data ) {
  CborLite::encodeMapSize( buffer, uint8_t( 2 ) );

  CborLite::encodeText( buffer, std::string( "ts" ) );
  CborLite::encodeUnsigned( buffer, micros() );

  CborLite::encodeText( buffer, std::string( "ag" ) );
  CborLite::encodeArraySize( buffer, uint8_t( 6 ) );

  for( const auto& i : data.acc ) {
    CborLite::encodeSingleFloat( buffer, i );
  }
  for( const auto& i : data.gyr ) {
    CborLite::encodeSingleFloat( buffer, i );
  }
}

template< typename T >
void
sendAccGyrMagTransmission( T& buffer ) {
  CborLite::encodeBreakIndefinite( buffer );
  udpSendFrom.broadcastTo( buffer.data(), buffer.size(), initialisation.portSendTo );
}

void sendDataTransmission( const uint16_t channelId, const char* data, const size_t len );
