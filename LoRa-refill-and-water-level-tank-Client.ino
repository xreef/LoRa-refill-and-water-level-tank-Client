#include "Arduino.h"

#include "LoRa_E32.h"
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include "include/states.h"

#define ACTIVATE_OTA
#ifdef ACTIVATE_OTA
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

bool connected = false;

#endif

void printParameters(struct Configuration configuration);
ResponseStatus sendUpdate(PACKET_TYPE packetType, bool needAckParam = false);

#define SERVER_ADDH 0x00
#define SERVER_ADDL 0x03
#define SERVER_CHANNEL 0x04

#define CLIENT_ADDH 0x00
#define CLIENT_ADDL 0x04
#define CLIENT_CHANNEL 0x04

#define AUX_PIN D5

// Battery voltage resistance
#define BAT_RES_VALUE_GND 20.0
#define BAT_RES_VALUE_VCC 10.0


#define TANK_MAX D7
#define TANK_MIN D6

LoRa_E32 e32ttl(&Serial, AUX_PIN, D0, D8);

#define TANK_DEBUG
// Define where debug output will be printed.
#define DEBUG_PRINTER Serial1

// Setup debug printing macros.
#ifdef TANK_DEBUG
	#define DEBUG_PRINT(...) { DEBUG_PRINTER.print(__VA_ARGS__); }
	#define DEBUG_PRINTF(...) { DEBUG_PRINTER.printf(__VA_ARGS__); }
	#define DEBUG_PRINTLN(...) { DEBUG_PRINTER.println(__VA_ARGS__); }
#else
	#define DEBUG_PRINT(...) {}
	#define DEBUG_PRINTF(...) {}
	#define DEBUG_PRINTLN(...) {}
#endif

#define FPM_SLEEP_MAX_TIME           0xFFFFFFF

void callback() {
  DEBUG_PRINTLN("WAKE THE DEVICE");
#ifdef TANK_DEBUG
	DEBUG_PRINTER.flush();
#endif
}

bool changed = false;

void setMin();
void setMax();

void IRAM_ATTR minCallack();
void IRAM_ATTR maxCallack();

unsigned long packetNumber = 0;


bool pumpIsActive = false;

bool minLevel = false;
bool maxLevel = false;

float batteryLevel = 0;

bool needAck = false;
unsigned long ackStartTime = 0;

float getBatteryVoltage();

void setup()
{
#ifdef TANK_DEBUG
	DEBUG_PRINTER.begin(115200);
#endif
	  // while (!SERIAL_DEBUG) {
	  //   ; // wait for serial port to connect. Needed for native USB
	  // }		// encoder pins


	  DEBUG_PRINTLN();
	  DEBUG_PRINTLN("--------------------------------------------");

	  pinMode(TANK_MIN, INPUT);
	  pinMode(TANK_MAX, INPUT);

	  attachInterrupt(digitalPinToInterrupt(TANK_MIN), minCallack, CHANGE );
	  attachInterrupt(digitalPinToInterrupt(TANK_MAX), maxCallack, CHANGE );

		delay(100);

		e32ttl.begin();

		delay(100);


	//	e32ttl.resetModule();
		// After set configuration comment set M0 and M1 to low
		// and reboot if you directly set HIGH M0 and M1 to program
		ResponseStructContainer c;
		c = e32ttl.getConfiguration();
		Configuration configuration = *(Configuration*) c.data;
		configuration.ADDL = CLIENT_ADDL;
		configuration.ADDH = CLIENT_ADDH;
		configuration.CHAN = CLIENT_CHANNEL;
		configuration.OPTION.fixedTransmission = FT_FIXED_TRANSMISSION;
		configuration.OPTION.wirelessWakeupTime = WAKE_UP_250;

		configuration.OPTION.fec = FEC_1_ON;
		configuration.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;
		configuration.OPTION.transmissionPower = POWER_20;

		configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;
		configuration.SPED.uartBaudRate = UART_BPS_9600;
		configuration.SPED.uartParity = MODE_00_8N1;

		ResponseStatus rs = e32ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
		DEBUG_PRINTLN(rs.getResponseDescription());

		c = e32ttl.getConfiguration();
		Configuration configurationNew = *(Configuration*) c.data;

		printParameters(configurationNew);

    c.close();
		// ---------------------------
		delay(2000);
		e32ttl.setMode(MODE_2_POWER_SAVING);

		DEBUG_PRINTLN();

		batteryLevel = getBatteryVoltage();


}
DynamicJsonDocument doc(512);
unsigned long timePassed = millis();
unsigned long interval = 25000;

unsigned long batteryIntervalHigh = 360000;

unsigned long batteryTimePassed = 0;
unsigned long batteryIntervalLow = 60000;

ResponseStatus setModeReceive();
ResponseStatus setModeSleep();

OPERATION_MODE operationalSelected = OPERATION_NORMAL;

bool btSended = false;

// The loop function is called in an endless loop
void loop()
{
	if (pumpIsActive==false){
		e32ttl.setMode(MODE_2_POWER_SAVING);

		DEBUG_PRINTLN("Check AUX!");

		while (digitalRead(AUX_PIN) == LOW){delay(100);};

		DEBUG_PRINTLN("Start sleep!");
		DEBUG_PRINTLN(e32ttl.getMode());

		//wifi_station_disconnect(); //not needed
		gpio_pin_wakeup_enable(GPIO_ID_PIN(AUX_PIN), GPIO_PIN_INTR_LOLEVEL);
		wifi_set_opmode(NULL_MODE);
		wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
		wifi_fpm_open();
		wifi_fpm_set_wakeup_cb(callback);
		wifi_fpm_do_sleep(FPM_SLEEP_MAX_TIME);
		delay(1000);

		DEBUG_PRINTLN();
		DEBUG_PRINTLN("Start listening!");

#ifdef ACTIVATE_OTA
	const char* ssid = "reef-casa-sopra";
	const char* password = "aabbccdd77";

	delay(100);
#ifdef TANK_DEBUG
	DEBUG_PRINTER.flush();
#endif
	DEBUG_PRINTLN("Booting");
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
	if (WiFi.waitForConnectResult() != WL_CONNECTED) {
		DEBUG_PRINTLN("Connection Failed! Rebooting...");
	  delay(5000);
	//   ESP.restart();
	} else {
	delay(1000);
	DEBUG_PRINTLN("Ready");
	DEBUG_PRINT("IP address: ");
	DEBUG_PRINTLN(WiFi.localIP());

	ArduinoOTA.onStart([]() {
		String type;
		if (ArduinoOTA.getCommand() == U_FLASH) {
		  type = "sketch";
		} else { // U_FS
		  type = "filesystem";
		}

		// NOTE: if updating FS this would be the place to unmount FS using FS.end()
		DEBUG_PRINTLN("Start updating " + type);
	  });
	  ArduinoOTA.onEnd([]() {
		DEBUG_PRINTLN("\nEnd");
	  });
	  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
		DEBUG_PRINTF("Progress: %u%%\r\n", (progress / (total / 100)));
	  });
	  ArduinoOTA.onError([](ota_error_t error) {
		DEBUG_PRINTF("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR) {
		  DEBUG_PRINTLN("Auth Failed");
		} else if (error == OTA_BEGIN_ERROR) {
		  DEBUG_PRINTLN("Begin Failed");
		} else if (error == OTA_CONNECT_ERROR) {
		  DEBUG_PRINTLN("Connect Failed");
		} else if (error == OTA_RECEIVE_ERROR) {
		  DEBUG_PRINTLN("Receive Failed");
		} else if (error == OTA_END_ERROR) {
		  DEBUG_PRINTLN("End Failed");
		}
	  });
	  ArduinoOTA.begin();
	  connected = true;
	}
#endif


		  setMin();
		  setMax();

		  DEBUG_PRINTLN("MIN MAX");
		  DEBUG_PRINT(minLevel);
		  DEBUG_PRINTLN(maxLevel);

	}
	if (e32ttl.available()){
		DEBUG_PRINTLN("Start reading!");

		ResponseContainer rs = e32ttl.receiveMessage();
		String message = rs.data;

		DEBUG_PRINTLN(rs.status.getResponseDescription());

		DEBUG_PRINTLN(message);
		deserializeJson(doc, message);

		String type = doc["type"];
		DEBUG_PRINT("type --> ");
		DEBUG_PRINTLN(type);

		operationalSelected = static_cast<OPERATION_MODE>((int)doc["mode"]);

		ResponseStatus rsW;

		if (type=="start"){
			pumpIsActive = true;

			DEBUG_PRINT(rsW.getResponseDescription());
			DEBUG_PRINTLN("Operation complete!!");

			if (batteryLevel>1){
				batteryTimePassed = 0;
				btSended = false;
			}

		}else if(type=="stopp"){
			batteryTimePassed = 0;

			pumpIsActive = false;
			ResponseStatus rsUpdate = sendUpdate(PACKET_PUMP_LEVEL);
			DEBUG_PRINTLN(rsUpdate.getResponseDescription());
			rsW = setModeSleep();
			DEBUG_PRINT(rsW.getResponseDescription());
			DEBUG_PRINTLN("Operation complete, go to sleep!!");
		}else if (type=="ackpa"){
			needAck = false;
		}

		ResponseStatus rsUpdate = sendUpdate(PACKET_PUMP_LEVEL);
		DEBUG_PRINTLN(rsUpdate.getResponseDescription());

		DEBUG_PRINTLN("Update complete!!");

		timePassed = millis();
	}
	if (
			(batteryLevel>1) &&
			(
					(
							batteryLevel>4 && millis()-batteryTimePassed>batteryIntervalHigh
					)
					||
					(
							batteryLevel<=4 && millis()-batteryTimePassed>batteryIntervalLow
					)
					||
					batteryTimePassed == 0
					||
					!btSended
			)
			){
		ResponseStatus rsUpdate = sendUpdate(BATTERY_LEVEL, false);
		DEBUG_PRINTLN(rsUpdate.getResponseDescription());
		DEBUG_PRINTLN("Update battery complete!!");

		btSended = true;
		batteryTimePassed = millis();
	}

	if ((operationalSelected!=OPERATION_DISABLED) && ((operationalSelected==OPERATION_PING && millis()-timePassed>interval) || changed || (needAck && millis()-ackStartTime>interval))){
		DEBUG_PRINT("Changed ----> ");
		DEBUG_PRINTLN(changed);
		DEBUG_PRINTLN(ackStartTime);
		DEBUG_PRINTLN(interval);

		if (needAck){
			ackStartTime = millis();
		}

		ResponseStatus rsUpdate = sendUpdate(PACKET_PUMP_LEVEL, needAck || changed);
		DEBUG_PRINTLN(rsUpdate.getResponseDescription());
		DEBUG_PRINTLN("Update complete!!");

		changed = false;

		timePassed = millis();
	}
#ifdef ACTIVATE_OTA
	  if (connected) ArduinoOTA.handle();
#endif

}

void printParameters(struct Configuration configuration) {
	DEBUG_PRINTLN("----------------------------------------");

	DEBUG_PRINT(F("HEAD BIN: "));  DEBUG_PRINT(configuration.HEAD, BIN);DEBUG_PRINT(" ");DEBUG_PRINT(configuration.HEAD, DEC);DEBUG_PRINT(" ");DEBUG_PRINTLN(configuration.HEAD, HEX);
	DEBUG_PRINTLN(F(" "));
	DEBUG_PRINT(F("AddH BIN: "));  DEBUG_PRINTLN(configuration.ADDH, DEC);
	DEBUG_PRINT(F("AddL BIN: "));  DEBUG_PRINTLN(configuration.ADDL, DEC);
	DEBUG_PRINT(F("Chan BIN: "));  DEBUG_PRINT(configuration.CHAN, DEC); DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.getChannelDescription());
	DEBUG_PRINTLN(F(" "));
	DEBUG_PRINT(F("SpeedParityBit BIN    : "));  DEBUG_PRINT(configuration.SPED.uartParity, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.SPED.getUARTParityDescription());
	DEBUG_PRINT(F("SpeedUARTDataRate BIN : "));  DEBUG_PRINT(configuration.SPED.uartBaudRate, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.SPED.getUARTBaudRate());
	DEBUG_PRINT(F("SpeedAirDataRate BIN  : "));  DEBUG_PRINT(configuration.SPED.airDataRate, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.SPED.getAirDataRate());

	DEBUG_PRINT(F("OptionTrans BIN       : "));  DEBUG_PRINT(configuration.OPTION.fixedTransmission, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.OPTION.getFixedTransmissionDescription());
	DEBUG_PRINT(F("OptionPullup BIN      : "));  DEBUG_PRINT(configuration.OPTION.ioDriveMode, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.OPTION.getIODroveModeDescription());
	DEBUG_PRINT(F("OptionWakeup BIN      : "));  DEBUG_PRINT(configuration.OPTION.wirelessWakeupTime, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.OPTION.getWirelessWakeUPTimeDescription());
	DEBUG_PRINT(F("OptionFEC BIN         : "));  DEBUG_PRINT(configuration.OPTION.fec, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.OPTION.getFECDescription());
	DEBUG_PRINT(F("OptionPower BIN       : "));  DEBUG_PRINT(configuration.OPTION.transmissionPower, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.OPTION.getTransmissionPowerDescription());

	DEBUG_PRINTLN("----------------------------------------");

}


ResponseStatus setModeNormal(){
	ResponseStatus rs;
	rs.code = e32ttl.setMode(MODE_0_NORMAL);
	return rs;
}
ResponseStatus setModeWake(){
	ResponseStatus rs;
	rs.code = e32ttl.setMode(MODE_1_WAKE_UP);
	return rs;
}

ResponseStatus setModeReceive(){
	ResponseStatus rs;
	rs.code = e32ttl.setMode(MODE_0_NORMAL);
	return rs;
}
ResponseStatus setModeProgram(){
	ResponseStatus rs;
	rs.code = e32ttl.setMode(MODE_3_SLEEP);
	return rs;
}
ResponseStatus setModeSleep(){
	ResponseStatus rs;
	rs.code = e32ttl.setMode(MODE_2_POWER_SAVING);
	return rs;
}


ResponseStatus sendUpdate(PACKET_TYPE packetType, bool needAckParam ){
	DEBUG_PRINTLN(" ------------ START ---------------");
	DEBUG_PRINTLN(packetType);
	delay(500);

	JsonObject root = doc.to<JsonObject>(); // get the root object

	switch (packetType) {
		case BATTERY_LEVEL: {
			batteryLevel = getBatteryVoltage();
			DEBUG_PRINT(F(" BATTERY --> "));
			DEBUG_PRINTLN(batteryLevel);
			root["ty"] = "bl";
			root["pn"] = packetNumber++;
			root["battLev"] = batteryLevel;
			break;
		}
		case PACKET_PUMP_LEVEL: {
			setMin();
			setMax();

			root["ty"] = "ppl";
			root["pn"] = packetNumber++;
			root["maxL"] = (maxLevel?1:0);
			root["minL"] = (minLevel?1:0);
			root["ack"] = needAckParam?1:0;
			break;
		}
		default:
			break;
	}

	int size = measureJson(doc)+1;

	char buf[size];
	serializeJson(doc, buf, size);
	DEBUG_PRINTLN(buf);
	DEBUG_PRINTLN(measureJson(doc));

	DEBUG_PRINT("Send message to server ");
	DEBUG_PRINT(SERVER_ADDH, DEC);
	DEBUG_PRINT(" ");
	DEBUG_PRINT(SERVER_ADDL, DEC);
	DEBUG_PRINT(" ");
	DEBUG_PRINT(SERVER_CHANNEL, HEX);
	DEBUG_PRINTLN(" ");

	DEBUG_PRINTLN("Check mode ");
	DEBUG_PRINTLN(e32ttl.getMode());

	ResponseStatus rsW = setModeNormal();

	DEBUG_PRINTLN(rsW.getResponseDescription());

	if (rsW.code!=SUCCESS) return rsW;

	ResponseStatus rsSend = e32ttl.sendFixedMessage(SERVER_ADDH, SERVER_ADDL, SERVER_CHANNEL, buf, size);
	DEBUG_PRINTLN(rsSend.getResponseDescription());

	if (rsSend.code==SUCCESS && needAckParam){
		ackStartTime = millis();
		needAck = true;
	}

	return rsSend;
}

void setMin(){
	uint8_t valMin = digitalRead(TANK_MIN);
	minLevel = valMin==HIGH;
}
void setMax(){
	uint8_t valMax = digitalRead(TANK_MAX);
	maxLevel = valMax==LOW;
}

//void ICACHE_RAM_ATTR minCallack(){
void IRAM_ATTR minCallack(){
	bool minLevelPrec = minLevel;
	setMin();
	if (minLevelPrec!=minLevel) changed = true;
}
//void ICACHE_RAM_ATTR maxCallack(){
void IRAM_ATTR maxCallack(){
	bool maxLevelPrec = maxLevel;
	setMax();
	if (maxLevelPrec!=maxLevel) changed = true;
}

float getBatteryVoltage(){
	//************ Measuring Battery Voltage ***********
	float sample1 = 0;

	for (int i = 0; i < 100; i++) {
		sample1 = sample1 + analogRead(A0); //read the voltage from the divider circuit
		delay(2);
	}
	sample1 = sample1 / 100;
	DEBUG_PRINT(F("AnalogRead..."));
	DEBUG_PRINTLN(sample1);
	float batVolt = (sample1 * 3.3 * (BAT_RES_VALUE_VCC + BAT_RES_VALUE_GND) / BAT_RES_VALUE_GND) / 1023;


	int bvI = batVolt * 100;

	batVolt = (float)bvI/100;

	return batVolt;
}

