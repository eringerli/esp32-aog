#include <WiFi.h>
#include <Preferences.h>
#include <DNSServer.h>
#include "network.hpp"
#include "webUi.hpp"
#include <ESPUI.h>
#include "main.hpp"
#include <ETH.h>

DNSServer hwSetupDnsServer;

bool hwSetupHasEthernet = false;

IPAddress hwSetupOwnAdress;


void hwSetupNetworkAp(bool emergencyMode) {
  IPAddress localIp = IPAddress( 172, 23, 42, 1 );
  char hostname[33];
  strcpy(hostname, "ESP-AOG"); // default
  preferences.getString("networkHostname", hostname, 33);

  char network[33];
  strcpy(network, ""); // default
  preferences.getString("networkSSID", network, 33);

  char password[33];
  strcpy(password, ""); // default
  preferences.getString("networkPassword", password, 33);

  WiFi.setHostname( hostname );
  WiFi.mode( WIFI_AP );
  delay(100); // lazzy, instead of waiting for SYSTEM_EVENT_AP_START
  WiFi.softAPConfig( localIp, localIp, IPAddress( 255, 255, 255, 0 ) );
  if (emergencyMode || strlen(password) < 8 || strlen(network) < 4) {
    WiFi.softAP( hostname );
  } else {
      WiFi.softAP( network, password);
  }
  hwSetupDnsServer.start( 53, "*", localIp );
  status.networkStatus = Status::Network::accessPoint;
  hwSetupOwnAdress = localIp;
}

void hwSetupNetworkClient() {
  char hostname[33];
  strcpy(hostname, "ESP-AOG"); // default
  preferences.getString("networkHostname", hostname, 33);

  char network[33];
  strcpy(network, ""); // default
  preferences.getString("networkSSID", network, 33);

  char password[33];
  strcpy(password, ""); // default
  preferences.getString("networkPassword", password, 33);

  // start init
  WiFi.setHostname( hostname );
  WiFi.mode(WIFI_STA);
  delay(100);

  char ipString[17];
  strcpy(ipString, ""); // default
  preferences.getString("networkIpAddress", ipString, 17);
  IPAddress ip;
  // valid fixed IP => configure fixed IP
  if (ip.fromString(ipString)) {
    usb.print("INFO: WiFi Client uses fixed IP: ");
    usb.println(ipString);
    char gatewayString[17];
    strcpy(gatewayString, ""); // default
    preferences.getString("networkIpGateway", gatewayString, 17);
    IPAddress gateway;
    gateway.fromString(gatewayString);

    char dnsString[17];
    strcpy(dnsString, ""); // default
    preferences.getString("networkIpDns", dnsString, 17);
    IPAddress dns;
    dns.fromString(dnsString);

    // configure fixed IP
    WiFi.config(ip, dns, gateway);
    hwSetupOwnAdress = ip;
  } else {
    usb.println("INFO: WiFi Client used dhcp");
  }

  WiFi.begin( network, password );
  status.networkStatus = Status::Network::connecting;
  WiFi.setAutoReconnect(true);

  // start "monitor" thread, simply sets teh network status nd if disconnected too long, restart
  xTaskCreate( hwSetupWifiMonitor, "WifiMonitor", 2048, NULL, 1, NULL );

}

void hwSetupWifiMonitor( void* z ) {
  // if in total 5s not connected, reboot. should resolve any wifi connection drops
  int counter = 0;
  while (counter < 25) {
    if (WiFi.status() == WL_CONNECTED) {
      hwSetupOwnAdress = WiFi.localIP();
      if (status.networkStatus != Status::Network::connected ) {
        status.networkStatus = Status::Network::connected;
        usb.print("IP: ");
        usb.println(hwSetupOwnAdress);
      }
      counter = 0;
    } else {
      counter++;
      usb.print("No network, state: ");
      usb.println(String(WiFi.status()));
    }
    delay(200);
  }
  usb.println("No network, reboot");
  ESP.restart();
}

void hwSetupNetworkLan8720(uint8_t phy_addr, int power, int mdc, int mdio) {
  char hostname[33];
  strcpy(hostname, "ESP-AOG"); // default
  preferences.getString("networkHostname", hostname, 33);

  // start init
  WiFi.onEvent(hwSetupEthernetEvent);
  ETH.begin(phy_addr, power, mdc, mdio, ETH_PHY_LAN8720, ETH_CLOCK_GPIO17_OUT);
  ETH.setHostname( hostname );
  delay(100);

  char ipString[17];
  strcpy(ipString, ""); // default
  preferences.getString("networkIpAddress", ipString, 17);
  IPAddress ip;
  // valid fixed IP => configure fixed IP
  if (ip.fromString(ipString)) {
    char gatewayString[17];
    strcpy(gatewayString, ""); // default
    preferences.getString("networkIpGateway", gatewayString, 17);
    IPAddress gateway;
    gateway.fromString(gatewayString);

    char dnsString[17];
    strcpy(dnsString, ""); // default
    preferences.getString("networkIpDns", dnsString, 17);
    IPAddress dns;
    dns.fromString(dnsString);

    // configure fixed IP
    ETH.config(ip, gateway, IPAddress(255, 255, 255, 0),dns, IPAddress(0, 0, 0, 0));
    hwSetupOwnAdress = ip;
  }
}

void hwSetupEthernetEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_ETH_START:
      status.networkStatus = Status::Network::disconnected;
      break;
    case SYSTEM_EVENT_ETH_CONNECTED:
      status.networkStatus = Status::Network::connecting;
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      status.networkStatus = Status::Network::connected;
      hwSetupOwnAdress = ETH.localIP();
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
      status.networkStatus = Status::Network::disconnected;
      break;
    default:
      break;
  }
}

void hwSetupWebNetwork() {
  uint8_t curNetwork = preferences.getUChar("networkSetup", 0);
  ESPUI.addControl( ControlType::Text, "Network-Hostname", preferences.getString("networkHostname", "ESP-AOG"), ControlColor::Wetasphalt, webTabHardware,
    []( Control * control, int id ) {
      preferences.putString("networkHostname", control->value);
      control->color = ControlColor::Carrot;
      ESPUI.updateControl( control );
      webChangeNeedsReboot();
    } );

    uint16_t sel = ESPUI.addControl( ControlType::Select, "Network-Interface", (String)curNetwork, ControlColor::Wetasphalt, webTabHardware,
      []( Control * control, int id ) {
        preferences.putUChar("networkSetup", control->value.toInt());
        control->color = ControlColor::Carrot;
        ESPUI.updateControl( control );
        webChangeNeedsReboot();
      } );
    ESPUI.addControl( ControlType::Option, "Wifi Access Point", "0", ControlColor::Alizarin, sel );
    ESPUI.addControl( ControlType::Option, "Wifi Client", "1", ControlColor::Alizarin, sel );
    if (hwSetupHasEthernet) {
      ESPUI.addControl( ControlType::Option, "Wired Ethernet", "2", ControlColor::Alizarin, sel );
    }

    if (curNetwork == 0 || curNetwork == 1 ) { // Wifi
      ESPUI.addControl( ControlType::Text, "AP Name / SSID (max. 32 char)", preferences.getString("networkSSID", ""), ControlColor::Wetasphalt, webTabHardware,
        []( Control * control, int id ) {
          preferences.putString("networkSSID", control->value);
          control->color = ControlColor::Carrot;
          ESPUI.updateControl( control );
          webChangeNeedsReboot();
        } );
      ESPUI.addControl( ControlType::Text, "Wifi Password (8 - 32 char)", preferences.getString("networkPassword", ""), ControlColor::Wetasphalt, webTabHardware,
        []( Control * control, int id ) {
          preferences.putString("networkPassword", control->value);
          control->color = ControlColor::Carrot;
          ESPUI.updateControl( control );
          webChangeNeedsReboot();
        } );
    }

    if (curNetwork > 0) { // not AP mode
      ESPUI.addControl( ControlType::Text, "IP Address (invalid = dhcp)", preferences.getString("networkIpAddress", "dhcp"), ControlColor::Wetasphalt, webTabHardware,
        []( Control * control, int id ) {
          preferences.putString("networkIpAddress", control->value);
          control->color = ControlColor::Carrot;
          ESPUI.updateControl( control );
          webChangeNeedsReboot();
        } );
      ESPUI.addControl( ControlType::Text, "Gateway", preferences.getString("networkIpGateway", "0.0.0.0"), ControlColor::Wetasphalt, webTabHardware,
        []( Control * control, int id ) {
          preferences.putString("networkIpGateway", control->value);
          control->color = ControlColor::Carrot;
          ESPUI.updateControl( control );
          webChangeNeedsReboot();
        } );
      ESPUI.addControl( ControlType::Text, "DNS Server", preferences.getString("networkIpDns", "0.0.0.0"), ControlColor::Wetasphalt, webTabHardware,
        []( Control * control, int id ) {
          preferences.putString("networkIpDns", control->value);
          control->color = ControlColor::Carrot;
          ESPUI.updateControl( control );
          webChangeNeedsReboot();
        } );
    }
}
