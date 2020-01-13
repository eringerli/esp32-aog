#ifndef gpsRtcm_HPP
#define gpsRtcm_HPP

#include "gpsCommon.hpp"

void gpsRtcmSetup(GpsRtcmData::RtcmDestination rtcmdestination);
void gpsRtcmCreateUdpReceiveHandler();
void gpsRtcmBtReceiver( void* z );
void gpsRtcmNtripReceiver( void* z );

#endif
