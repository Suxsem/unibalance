#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#define NAME "Unibalance"

/* WIFI */
#define DEFAULTssid "Domo"
#define DEFAULTpsw "domopassword"
WiFiUDP udp;

void setup() {
  
  /* WIFI */
  ArduinoOTA.setHostname(NAME);
  WiFi.mode(WIFI_STA);
  WiFi.begin(DEFAULTssid, DEFAULTpsw);

  delay(2000);

  ArduinoOTA.begin();
  udp.begin(89);
  Serial.begin(115200);

}

char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet,

void loop() {

  ArduinoOTA.handle();

  int packetSize = udp.parsePacket();
  if (packetSize) {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);

    // read the packet into packetBufffer
    udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    Serial.println("Contents:");
    Serial.println(packetBuffer);

  }

}
