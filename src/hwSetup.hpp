#ifndef hwSetup_HPP
#define hwSetup_HPP
#include <Preferences.h>
#include <ETH.h>
#include <DNSServer.h>

void hwSetupInitial();
void hwSetupWebSetup();
char* hwSetupHardwareIdToName(uint8_t setup);

void hwSetupF9PIoBoardNmea();
void hwSetupF9PIoBoardWebAnalogIn(int parent);
void hwSetupF9PIoBoardWebDigitalOut(int parent);
void hwSetupF9PIoBoardMotor1(int pwm);

void hwSetupNodeMcuCytronNmea();
void hwSetupNodeMcuIbt2Nmea();
void hwSetupNodeMcuWebAnalogIn(int parent);
void hwSetupNodeMcuWebDigitalOut(int parent);
void hwSetupNodeMcuCytronMotor1(int pwm);
void hwSetupNodeMcuIbt2Motor1(int pwm);

#endif
