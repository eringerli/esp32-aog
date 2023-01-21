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

#include <vector>

#include "../lib/cbor-lite/include/cbor-lite/codec-fp.h"

#pragma once

class CborPackageBase {
public:
  enum class Type { Base, State, Number, Data };

  CborPackageBase( Type type, uint16_t channelId )
      : type( type ), channelId( channelId ) {}

  Type type = Type::Base;

  uint16_t channelId = 0;

  virtual void convertToCBOR( std::vector< uint8_t >& buffer ) = 0;

  void sendPackage();
};

class CborPackageState : public CborPackageBase {
public:
  bool state = false;

  CborPackageState( uint16_t channelId, bool state )
      : CborPackageBase( Type::State, channelId ), state( state ) {}

  virtual void convertToCBOR( std::vector< uint8_t >& buffer ) override;
};

class CborPackageNumber : public CborPackageBase {
public:
  double number = 0;

  CborPackageNumber( uint16_t channelId, double number )
      : CborPackageBase( Type::State, channelId ), number( number ) {}

  virtual void convertToCBOR( std::vector< uint8_t >& buffer ) override;
};

template< typename InputIterator >
CborPackageBase*
create_cbor_package( InputIterator&  pos,
                     InputIterator   end,
                     CborLite::Flags flags = CborLite::Flag::none ) {
  size_t nItems = 0;
  auto   len    = CborLite::decodeMapSize( pos, end, nItems, flags );
  if( nItems > 2 ) {
    return nullptr;
  }

  std::string str;
  CborLite::decodeText( pos, end, str, flags );
  if( str != "cid" ) {
    return nullptr;
  }

  uint16_t channelId;
  CborLite::decodeUnsigned( pos, end, channelId, flags );

  // "ts"
  {
    CborLite::decodeText( pos, end, str, flags );
    uint32_t ts;
    CborLite::decodeUnsigned( pos, end, ts, flags );
  }

  CborLite::decodeText( pos, end, str, flags );
  if( str == "n" ) {
    double number;
    CborLite::decodeDoubleFloat( pos, end, number, flags );

    return new CborPackageNumber( channelId, number );
  } else if( str == "s" ) {
    bool state;
    CborLite::decodeBool( pos, end, state, flags );

    return new CborPackageState( channelId, state );
  }

  return nullptr;
}
