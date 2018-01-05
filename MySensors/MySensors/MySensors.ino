/*
 Name:		MySensors.ino
 Created:	04-Jan-18 15:37:07
 Author:	Bastiaan
*/

#include <SPI.h>
#include <Ethernet.h>

// the media access control (ethernet hardware) address for the shield:
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xAA, 0xAA, 0x01 };
IPAddress ip(192, 168, 10, 202);
IPAddress ipGateway(192, 168, 10, 1);
IPAddress ipDNS(192, 168, 10, 1);
IPAddress ipSubnet(255, 255, 255, 0);

void setup() {
  Serial.begin(9600);
  Ethernet.maintain();
  Ethernet.begin(mac,ip,ipDNS,ipGateway, ipSubnet);
}

// the loop function runs over and over again until power down or reset
void loop() {
  
}
