// MIT License
//
// Copyright (c) 2019 Christian Riggenbach
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

#include <ESPUI.h>

#include "main.hpp"

volatile uint16_t idleCtrCore0 = 0;
volatile uint16_t idleCtrCore1 = 0;
void core0IdleWorker( void* z ) {
  constexpr TickType_t xFrequency = 1;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while ( 1 ) {
    idleCtrCore0++;
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void core1IdleWorker( void* z ) {
  constexpr TickType_t xFrequency = 1;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while ( 1 ) {
    idleCtrCore1++;
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void idleStatsWorker( void* z ) {
  constexpr TickType_t xFrequency = 1000;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while ( 1 ) {
    String str;
    str.reserve( 40 );
    str += ( "Core0: " );
    str += 1000 - idleCtrCore0;
    str += "‰<br/>";
    str += "Core1: ";
    str += 1000 - idleCtrCore1;
    str += "‰";

    Control* labelLoadHandle = ESPUI.getControl( labelLoad );
    labelLoadHandle->value = str;
    ESPUI.updateControl( labelLoadHandle );

    idleCtrCore0 = 0;
    idleCtrCore1 = 0;

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}


void initIdleStats() {
  xTaskCreatePinnedToCore( core0IdleWorker, "Core0IdleWorker", 1024, NULL, tskIDLE_PRIORITY, NULL, 0 );
  xTaskCreatePinnedToCore( core1IdleWorker, "Core1IdleWorker", 1024, NULL, tskIDLE_PRIORITY, NULL, 1 );
  xTaskCreate( idleStatsWorker, "IdleStats", 4096, NULL, 10, NULL );
}
