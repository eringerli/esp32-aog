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
#include "esp_freertos_hooks.h"
#include "esp_heap_trace.h"

#include "main.hpp"

volatile uint16_t idleCtrCore0 = 0;
volatile uint16_t idleCtrCore1 = 0;

bool core0IdleWorker( void ) {
  static TickType_t xLastWakeTime0;

  if( xLastWakeTime0 != xTaskGetTickCount() ) {
    xLastWakeTime0 = xTaskGetTickCount();
    idleCtrCore0++;
  }

  return true;
}

bool core1IdleWorker( void ) {
  static TickType_t xLastWakeTime1;

  if( xLastWakeTime1 != xTaskGetTickCount() ) {
    xLastWakeTime1 = xTaskGetTickCount();
    idleCtrCore1++;
  }

  return true;
}

void idleStatsWorker( void* z ) {
  constexpr TickType_t xFrequency = 1000;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  String str;
  str.reserve( 150 );

  multi_heap_info_t heapInfo;

  while( 1 ) {
    heap_caps_get_info( &heapInfo, MALLOC_CAP_8BIT );

    str = "Core0: ";
    str += 1000 - idleCtrCore0;
    str += "‰<br/>";
    str += "Core1: ";
    str += 1000 - idleCtrCore1;
    str += "‰<br/>Uptime: ";
    str += millis() / 1000;
    str += "s<br/>Heap free: ";
    str += heapInfo.total_free_bytes / 1024;
    str += "kB of ";
    str += heapInfo.total_allocated_bytes / 1024;
    str += "kB<br/>Lowest ever free Heap: ";
    str += heapInfo.minimum_free_bytes / 1024;
    str += "kB<br/>Largest free block on Heap: ";
    str += heapInfo.largest_free_block / 1024;
    str += "kB";

    Control* labelLoadHandle = ESPUI.getControl( labelLoad );
    labelLoadHandle->value = str;
    ESPUI.updateControlAsync( labelLoadHandle );

    idleCtrCore0 = 0;
    idleCtrCore1 = 0;

    ESPUI.updateControlAsyncTransmit();

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void initIdleStats() {
  esp_register_freertos_idle_hook_for_cpu( core0IdleWorker, 0 );
  esp_register_freertos_idle_hook_for_cpu( core1IdleWorker, 1 );
  xTaskCreate( idleStatsWorker, "IdleStats", 2048, NULL, 10, NULL );
}
