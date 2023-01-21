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

#pragma once

#include <algorithm>
#include <memory>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "main.hpp"

#include "CborPackage.hpp"

#include <map>

class CborQueueSelector {
public:
  CborQueueSelector() {}

  void addQueue( uint16_t channelId, QueueHandle_t queue ) {
    queueMap[channelId] = queue;
  }

  bool isValidChannelId( uint16_t channelId ) { return queueMap.count( channelId ); }

  QueueHandle_t getQueue( uint16_t channelId ) { return queueMap.at( channelId ); }

private:
  std::map< uint16_t, QueueHandle_t > queueMap;
};
