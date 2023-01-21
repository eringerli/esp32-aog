// Copyright (c) 2014, Christian Riggenbach
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// Neither the name of Magahugu.ch nor the names of its contributors may be
//    used to endorse or promote products derived from this software without
//    specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <main.hpp>

template< class T, size_t Size, class size_type = uint16_t >
class RingBuffer {
private:
  // Pointer zum Puffer
  T Buffer[Size];

  // gespeicherte Daten
  volatile size_type dataInBuffer;

  // Lese- und Schreibe-Pointer
  volatile T* readPointer;
  volatile T* writePointer;

public:
  RingBuffer()
      : Buffer(), dataInBuffer( 0 ), readPointer( Buffer ), writePointer( Buffer ) {}

  bool isEmpty() { return dataInBuffer == 0; }

  size_type sizeOfBuffer() { return dataInBuffer; }

  void addToBufferBack( const T c ) {
    TCritSect critical();

    *writePointer++ = c;

    // bei Überlauf auch readPointer inkrementieren
    if( dataInBuffer < ( Size * sizeof( T ) ) ) {
      dataInBuffer++;
    } else {
      if( ++readPointer >= &Buffer[Size] ) {
        readPointer = Buffer;
      }
    }

    // Überlauf
    if( writePointer >= &( Buffer[Size] ) ) {
      writePointer = Buffer;
    }
  }

  void operator>>( T& c ) { c = takeFront(); }

  size_type operator<<( const T c ) {
    addToBufferBack( c );
    return dataInBuffer;
  }

  T takeFront() {
    T         c = 0;
    TCritSect critical();

    // auf leeren Puffer prüfen
    if( dataInBuffer ) {
      // hole das Zeichen
      c = *readPointer++;

      // dataInBuffer ändern
      dataInBuffer--;

      // Überlauf
      if( readPointer >= &( Buffer[Size] ) ) {
        readPointer = Buffer;
      }
    }

    return c;
  }

  T& viewFront() { return *readPointer; }

  void incrementFront() {
    TCritSect critical();

    // auf leeren Puffer prüfen
    if( dataInBuffer ) {
      // hole das Zeichen
      readPointer++;

      // dataInBuffer ändern
      dataInBuffer--;

      // Überlauf
      if( readPointer >= &( Buffer[Size] ) ) {
        readPointer = Buffer;
      }
    }
  }

  void flush() {
    TCritSect critical();
    readPointer  = writePointer;
    dataInBuffer = 0;
  }
};
