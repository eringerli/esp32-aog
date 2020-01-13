#ifndef network_HPP
#define network_HPP
#include <ETH.h>
#include <DNSServer.h>

void hwSetupNetworkAp(bool emergencyMode = true);
void hwSetupNetworkClient();
void hwSetupNetworkLan8720(uint8_t phy_addr, int power, int mdc, int mdio);
void hwSetupWifiMonitor( void* z );
void hwSetupEthernetEvent(WiFiEvent_t event);
void hwSetupWebNetwork();

extern bool hwSetupHasEthernet;
extern IPAddress hwSetupOwnAdress;
extern DNSServer hwSetupDnsServer;
#endif
