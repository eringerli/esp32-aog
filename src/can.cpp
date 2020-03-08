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
#include <ESP32CAN.h>
#include <CAN_config.h>

#include "main.hpp"

CAN_device_t CAN_cfg;

constexpr uint8_t rxQueueSize = 10;


constexpr uint8_t  IsobusPgPos   = 8;
constexpr uint32_t IsobusPgnMask = 0x03FFFF;

constexpr uint16_t j1939PgnEEC1 = 61444;
constexpr uint16_t j1939PgnWBSD = 65096;

constexpr uint16_t j1939PgnPHS = 65093;
constexpr uint16_t j1939PgnFHS = 65094;

constexpr uint16_t j1939PgnRPTO = 65091;
constexpr uint16_t j1939PgnFPTO = 65092;

// see https://gurtam.com/files/ftp/CAN/ (especialy J1939.zip)

void canWorker10Hz( void* z ) {
  constexpr TickType_t xFrequency = 100;

  CAN_frame_t canFrame;

  while( 1 ) {
    if( xQueueReceive( CAN_cfg.rx_queue, &canFrame, xFrequency ) == pdTRUE ) {
      if( canFrame.FIR.B.FF == CAN_frame_ext ) {

        uint16_t pgn = ( canFrame.MsgID >> IsobusPgPos ) & IsobusPgnMask;

        switch( pgn ) {

          // Electronic Engine Controller 1
          case j1939PgnEEC1: {
            steerCanData.motorRpm = ( canFrame.data.u8[4] << 8 | canFrame.data.u8[3] ) / 8;
          }
          break;

          // Wheel-based Speed and Distance
          case j1939PgnWBSD: {
            steerCanData.speed = ( canFrame.data.u8[1] << 8 | canFrame.data.u8[0] ) / 1000 * 3.6;
          }
          break;

          // Primary or Rear Hitch Status
          case j1939PgnPHS: {
            steerCanData.rearHitchPosition = canFrame.data.u8[0];
          }
          break;

          // Secondary or Front Hitch Status
          case j1939PgnFHS: {
            steerCanData.frontHitchPosition = canFrame.data.u8[0];
          }
          break;

          // Primary or Rear Power Take off Output Shaft
          case j1939PgnRPTO: {
            steerCanData.rearPtoRpm = canFrame.data.u8[1] << 8 | canFrame.data.u8[0];
          }
          break;

          // Secondary or Front Power Take off Output Shaft
          case j1939PgnFPTO: {
            steerCanData.frontPtoRpm = canFrame.data.u8[1] << 8 | canFrame.data.u8[0];
          }
          break;
        }
      }
    }

    {
      static uint8_t loopTimeToWaitTo = 0;

      if( loopTimeToWaitTo < millis() ) {

        Control* handle = ESPUI.getControl( labelStatusCan );
        String str;
        str.reserve( 200 );

        str = "<table style='margin:auto;'><tr><td style='text-align:left; padding: 0px 5px;'>Wheel-based Speed:</td><td style='text-align:left; padding: 0px 5px;'>";
        str += String( steerCanData.speed );
        str += "</td></tr><tr><td style='text-align:left; padding: 0px 5px;'>Motor RPM:</td><td style='text-align:left; padding: 0px 5px;'>";
        str += String( steerCanData.motorRpm );
        str += "</td></tr><tr><td style='text-align:left; padding: 0px 5px;'>Front Hitch Position:</td><td style='text-align:left; padding: 0px 5px;'>";
        str += String( steerCanData.frontHitchPosition );
        str += "</td></tr><tr><td style='text-align:left; padding: 0px 5px;'>Rear Hitch Position:</td><td style='text-align:left; padding: 0px 5px;'>";
        str += String( steerCanData.rearHitchPosition );
        str += "</td></tr><tr><td style='text-align:left; padding: 0px 5px;'>Front PTO RPM:</td><td style='text-align:left; padding: 0px 5px;'>";
        str += String( steerCanData.frontPtoRpm );
        str += "</td></tr><tr><td style='text-align:left; padding: 0px 5px;'>Rear PTO RPM:</td><td style='text-align:left; padding: 0px 5px;'>";
        str += String( steerCanData.rearPtoRpm );
        str += "</td></tr></table>";

        handle->value = str;
        ESPUI.updateControlAsync( handle );

        loopTimeToWaitTo = millis() + xFrequency;
      }
    }
  }
}


void initCan() {
  if( steerConfig.canBusEnabled ) {
    CAN_cfg.speed = ( CAN_speed_t )steerConfig.canBusSpeed;
    CAN_cfg.tx_pin_id = ( gpio_num_t )steerConfig.canBusRx;
    CAN_cfg.rx_pin_id = ( gpio_num_t )steerConfig.canBusTx;
    CAN_cfg.rx_queue = xQueueCreate( rxQueueSize, sizeof( CAN_frame_t ) );
    // Init CAN Module
    ESP32Can.CANInit();

    xTaskCreate( canWorker10Hz, "canWorker", 1024, NULL, 5, NULL );
  }
}
