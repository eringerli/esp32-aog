#include <ESPUI.h>

#include "udpHandler.hpp"
#include "webUi.hpp"
#include "hwSetup.hpp"

UdpFromAogData udpAogData;
UdpActualData udpActualData;

AsyncUDP udpHandlerListener;
AsyncUDP udpHandlerSender;

constexpr uint udpPortDataFromAog = 8888;
constexpr uint udpPortDataToAog = 9999;
constexpr uint udpPortOwn = 5577;

void udpHandlerInit(){
  if(! udpHandlerListener.listen(udpPortDataFromAog)) {
    usb.println ("ERROR: Starting UDP Listener for steering failed");
  } else {
    udpHandlerCreateReceiveHandler();
  }

  // sender
  udpHandlerSender.listen(udpPortOwn);
  xTaskCreate( udpHandlerSendData, "UdpPGNSender", 4096, NULL, 10, NULL );
}

void udpHandlerCreateReceiveHandler() {
  udpHandlerListener.onPacket([](AsyncUDPPacket packet) {
      uint8_t* data = packet.data();
      uint16_t pgn = data[1] + ( data[0] << 8 );
      switch ( pgn ) {
        case 0x7FFE: {
          udpAogData.distanceFromGuidanceLine = data[5] + ( data[4] << 8 );
          udpAogData.requiredSteerAngle = (( int16_t )( data[7] + ( data[6] << 8 ) )) / 100.0;
          udpAogData.uTurnRelais = data[8];
          udpAogData.lastReceived7FFE = millis();
        }
        break;
      }
    }
  );
}

void udpHandlerSendData( void* z ) {
  constexpr TickType_t xFrequency = 100; // every 100ms, so data at aog is relativly current without overwhelming the process

  while ( 1 ) {
    byte toSend[] = {0x7F,0x7D,0,0,0,0,0,0,0,0}; // steering data

    // steer angles
    int16_t temp = round(100 * udpActualData.steerAngleActual);
    toSend[2] = (byte)(temp >> 8);
    toSend[3] = (byte)(temp);

    // heading
    temp = round(16 * udpActualData.heading);
    toSend[4] = (byte)(temp >> 8);
    toSend[5] = (byte)(temp);

    // roll
    temp = round(16 * udpActualData.roll);
    toSend[6] = (byte)(temp >> 8);
    toSend[7] = (byte)(temp);

    // switches
    temp = 0;
    temp |= (udpActualData.remoteSwitch << 2); //put remote in bit 2
    temp |= ((!udpActualData.steerSwitch) << 1);   //put steerswitch status in bit 1 position, AOG interprets false as active
    temp |= udpActualData.workSwitch;
    toSend[8] = (byte)temp;

    //pwm
    temp = udpActualData.pwm / 2; // internal PWM +- 255
    toSend[9] = (byte)temp;

    //send
    if (udpHandlerSender.broadcastTo( toSend, sizeof(toSend), 9999 )) {
      udpActualData.lastSent = millis();
    }

    // wait
    vTaskDelay( xFrequency );
  }
}


void udpHandlerWebUpdate() {
  String str;
  str.reserve( 40 );

  str = "7FFE: ";
  str += udpHandlerTimeGenerator(udpAogData.lastReceived7FFE);
  str += "<br/>";
  str += "7FFD: ";
  str += udpHandlerTimeGenerator(udpActualData.lastSent);

  Control* labelPgnStatus = ESPUI.getControl( webLabelPgnStatus );
  labelPgnStatus->value = str;
  ESPUI.updateControl( webLabelPgnStatus );
}

String udpHandlerTimeGenerator(uint last) {
  uint duration = millis() - last;
  String ret;
  ret.reserve(6);
  if (last == 0 || duration > 9000000){
    ret = "never";
  } else if (duration < 10000) {
    ret = duration;
    ret += "ms";
  } else {
    ret = duration / 1000;
    ret += "s";
  }
  return ret;
}
