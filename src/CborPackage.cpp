// MIT License
//
// Copyright (c) 2022 Christian Riggenbach
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

#include "CborPackage.hpp"

#include "main.hpp"

void
CborPackageBase::sendPackage() {
  std::vector< uint8_t > buffer;
  buffer.reserve( 24 );

  convertToCBOR( buffer );

  udpSendFrom.broadcastTo( buffer.data(), buffer.size(), steerConfig.qogPortSendTo );
}

void
CborPackageState::convertToCBOR( std::vector< uint8_t >& buffer ) {
  CborLite::encodeMapSize( buffer, uint8_t( 3 ) );

  CborLite::encodeText( buffer, std::string( "cid" ) );
  CborLite::encodeUnsigned( buffer, this->channelId );

  CborLite::encodeText( buffer, std::string( "ts" ) );
  CborLite::encodeUnsigned( buffer, micros() );

  CborLite::encodeText( buffer, std::string( "s" ) );
  CborLite::encodeBool( buffer, state );
};

void
CborPackageNumber::convertToCBOR( std::vector< uint8_t >& buffer ) {
  CborLite::encodeMapSize( buffer, uint8_t( 3 ) );
  CborLite::encodeText( buffer, std::string( "cid" ) );
  CborLite::encodeUnsigned( buffer, this->channelId );

  CborLite::encodeText( buffer, std::string( "ts" ) );
  CborLite::encodeUnsigned( buffer, micros() );

  CborLite::encodeText( buffer, std::string( "n" ) );
  CborLite::encodeDoubleFloat( buffer, number );
};
