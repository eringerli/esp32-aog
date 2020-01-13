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
#include "webUi.hpp"
#include "idleStats.hpp"

volatile uint16_t idleCtrCore0 = 0;
volatile uint16_t idleCtrCore1 = 0;

bool core0IdleWorker( void ) {
  static TickType_t xLastWakeTime0;
  if (xLastWakeTime0 != xTaskGetTickCount()) {
    xLastWakeTime0 = xTaskGetTickCount();
    idleCtrCore0++;
  }
  return true;
}

bool core1IdleWorker( void ) {
  static TickType_t xLastWakeTime1;
  if (xLastWakeTime1 != xTaskGetTickCount()) {
    xLastWakeTime1 = xTaskGetTickCount();
    idleCtrCore1++;
  }
  return true;
}

void idleStats() {
  String str;
  str.reserve( 50 );

  str = "Core0: ";
  str += 1000 - idleCtrCore0;
  str += "‰<br/>Core1: ";
  str += 1000 - idleCtrCore1;
  str += "‰<br/>Uptime: ";
  str += millis() / 1000;
  str += "s<br/>Heap: ";
  str += ESP.getFreeHeap()/1024;
  str += "kB";

  Control* labelLoadHandle = ESPUI.getControl( webLabelLoad );
  labelLoadHandle->value = str;
  ESPUI.updateControl( labelLoadHandle );

  idleCtrCore0 = 0;
  idleCtrCore1 = 0;
}


void initIdleStats() {
  esp_register_freertos_idle_hook_for_cpu(core0IdleWorker, 0);
  esp_register_freertos_idle_hook_for_cpu(core1IdleWorker, 1);
}
