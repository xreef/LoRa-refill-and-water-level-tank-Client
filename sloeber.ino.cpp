#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2023-08-17 15:49:36

#include "Arduino.h"
#include "Arduino.h"
#include "LoRa_E32.h"
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include "include/states.h"
#define ACTIVATE_OTA
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

void callback() ;
void setup() ;
void loop() ;
void printParameters(struct Configuration configuration) ;
ResponseStatus setModeNormal();
ResponseStatus setModeWake();
ResponseStatus setModeReceive();
ResponseStatus setModeProgram();
ResponseStatus setModeSleep();
ResponseStatus sendUpdate(PACKET_TYPE packetType, bool needAckParam );
void setMin();
void setMax();
void IRAM_ATTR minCallack();
void IRAM_ATTR maxCallack();
float getBatteryVoltage();


#include "LoRa-refill-and-water-level-tank-Client.ino"

#endif
