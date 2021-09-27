#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <util.h>

#define DEBUG_ESP_PORT Serial

#ifdef DEBUG_ESP_PORT
	#define DEBUG_MSG(args...) DEBUG_ESP_PORT.printf( args )
#else
	#define DEBUG_MSG(...)
#endif

//                               |   0 |   1  |    2   |   3  |   4  |   5  |    6   |  7  |   8  |   9  |  10   |  11 |  12  |  13  |   14   |  15 |
//                               |Norte|      |Noroeste|      | Oeste|      |Sudoeste|     |  Sul |      |Sudeste|     | Leste|      |Nordeste|     |
const float constDirection[16] = {0.411, 1.377, 1.202,   2.774, 2.721, 0.331, 2.248,  2.540, 1.803, 0.824, 0.749, 2.879, 0.124, 1.980, 0.223,  0.584};
// st float constDirection[16] = {0.411, 0.584, 0.223,   0.331, 0.124, 0.824, 0.749,  1.980, 1.803, 2.540, 2.248, 2.879, 2.721, 2.774, 1.202,  1.377};

ADC_MODE(ADC_VCC);

WiFiClient espWifi;

Adafruit_ADS1115 ads;

TinyGPSPlus gps;

SoftwareSerial SerialGPS(RX, TX);

/*********************************** NTPTime ************************************/

WiFiUDP Udp;
unsigned int localPort = 8888;
const int timeZone = 0;
time_t getNtpTime();

/***************************** Tópicos MQTT *************************************/

const char* topic_Rain = "estacao/invalidTopic/data_rain";
const char* topic_Location = "estacao/invalidTopic/data_location";
const char* topic_Wind = "estacao/invalidTopic/data_wind";
const char* topic_Air = "estacao/invalidTopic/data_air";
const char* topic_Sun = "estacao/invalidTopic/data_sun";
const char* topic_Internal = "estacao/invalidTopic/data_internal";
const char* topic_firmware = "estacao/invalidTopic/firmware";

Adafruit_MQTT_Client mqtt(&espWifi, MQTTSERVER, MQTTPORT, MQTTUSER, MQTTPASS);
Adafruit_MQTT_Publish estacao = Adafruit_MQTT_Publish(&mqtt, topic_Rain);
Adafruit_MQTT_Publish estacao_loc = Adafruit_MQTT_Publish(&mqtt, topic_Location);
Adafruit_MQTT_Publish estacao_wind = Adafruit_MQTT_Publish(&mqtt, topic_Wind);
Adafruit_MQTT_Publish estacao_air = Adafruit_MQTT_Publish(&mqtt, topic_Air);
Adafruit_MQTT_Publish estacao_sun = Adafruit_MQTT_Publish(&mqtt, topic_Sun);
Adafruit_MQTT_Publish estacao_internal = Adafruit_MQTT_Publish(&mqtt, topic_Internal);
Adafruit_MQTT_Subscribe firmwareupdate = Adafruit_MQTT_Subscribe(&mqtt, topic_firmware);

Adafruit_BME280 bme;

volatile int countSpeed = 0;
volatile int countRain = 0;
volatile uint32_t lastcount = 0;
volatile uint32_t lastcount2 = 0;

bool failed = true;

byte modaDirection[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

struct sensores
{
	float temperature;//----|
	float humidity;//-------|
	float pressure;//-------|BME680
	uint16_t iaq;//---------|
	float altitude;//-------|
	float rain;//-------------|
	float windSpeed;//--------|Sensores mecanicos
	uint8_t windDirection;//--|
	float lat;//------------|
	float lng;//------------|GY-NEO6MV2
	// float altitude;//-------|
	uint8_t uva;//--------------|
	float irra;//-------------|
	uint32_t unixtime;//----|
} sensor, auxSensor;//chegará a 41Bytes

struct selfData
{
	float guidance;
	float battery;
	float power;
	float panel;
	uint32_t up_time;
	uint32_t unixtime;
} internal;

uint8_t Calcula_nivel_UV(float tensao)
{
  // Compara com valores tabela UV_Index
  if (tensao > 0 && tensao < 50)
  {
    return 0;
  }
  else if (tensao > 50 && tensao <= 227)
  {
    return 0;
  }
  else if (tensao > 227 && tensao <= 318)
  {
    return 1;
  }
  else if (tensao > 318 && tensao <= 408)
  {
    return 2;
  }
  else if (tensao > 408 && tensao <= 503)
  {
    return 3;
  }
  else if (tensao > 503 && tensao <= 606)
  {
    return 4;
  }
  else if (tensao > 606 && tensao <= 696)
  {
    return 5;
  }
  else if (tensao > 696 && tensao <= 795)
  {
    return 6;
  }
  else if (tensao > 795 && tensao <= 881)
  {
    return 7;
  }
  else if (tensao > 881 && tensao <= 976)
  {
    return 8;
  }
  else if (tensao > 976 && tensao <= 1079)
  {
    return 9;
  }
  else if (tensao > 1079 && tensao <= 1170)
  {
    return 10;
  }
  else if (tensao > 1170)
  {
    return 11;
  }
}

ICACHE_RAM_ATTR void anemometro()
{
	if(millis() - lastcount2 > HYSTERESIS_DELAY_ANE)
	{
		countSpeed++;
	}

	lastcount2 = millis();
}

ICACHE_RAM_ATTR void pluviometro()
{
	if(millis() - lastcount > HYSTERESIS_DELAY_PLU)
	{
		countRain++;
	}

	lastcount = millis();
}

void windDirection()
{
	uint8_t x;

	float AIN2 = ads.readADC_SingleEnded(2)*0.000125;

	uint8_t temporaryDirection = 0;
	float dirTemp = 10.0;

	for ( x=0; x < 16; x++)
	{
		if (fabs(constDirection[x]-AIN2)<dirTemp)
		{
			dirTemp = fabs(constDirection[x]-AIN2);
			temporaryDirection = x;
		}
	}
	modaDirection[temporaryDirection]++;
}

char* toJSON(struct sensores *dataJson)
{
	String dataBuffer = //"{\"temperature\": "   + String(dataJson->temperature) +
						//", \"humidity\": "     + String(dataJson->humidity)    +
						//", \"pressure\": "     + String(dataJson->pressure)    +
						//", \"air_quality\": "  + String(dataJson->iaq)         +
						//", \"windspeed\": "    + String(dataJson->windSpeed)   +
						//", \"winddirection\": "    + String(dataJson->windDirection)   +
						"{\"rain\": " + String(dataJson->rain) +
						", \"count_lighting\": 0" +
						", \"unix_time\": " + String(dataJson->unixtime)    + "}";
						//", \"altitude\": "     + String(dataJson->altitude)    +
						//", IS: " + String(dataJson->lat, 6) +
						//", UVA: " + String(dataJson->lat, 6) +
						//", lat: " + String(dataJson->lat, 6) +
						//", long: " + String(dataJson->lng, 6) +
	int dataLength = dataBuffer.length()+1;
	char* data = new char[dataLength];
	strcpy(data, dataBuffer.c_str());
	//data = &dataBuffer[0u];

	Serial.print("Dados em readSensor: ");
	for(int i=0; i<dataLength; i++)
	{
		Serial.print(data[i]);
	}
	Serial.println("");
	//char* data = (char*)"0";
	return data;
}

char* readSensor()
{
	//BME280
	// if(bme.isReadingCalibration()){
		sensor.temperature = bme.readTemperature();
		sensor.pressure = bme.readPressure();
		sensor.humidity = bme.readHumidity();
		sensor.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
		sensor.iaq = 0;
	// } else {
		//Codigo de erro
	// }

	//PLUVIOMETRO
	sensor.rain = countRain * SCALERAIN;
	countRain = 0;

	//ANEMOMETRO
	sensor.windSpeed = (countSpeed * SCALEWSPEED) / (INTERVAL/1000);
	countSpeed = 0;

	//BIRUTA
	uint8_t x, xt=0;
	for(x=0; x<16; x++){
		if(modaDirection[x] > xt)
		{
			xt = modaDirection[x];
			sensor.windDirection = x;
		}
		modaDirection[x] = 0;
	}

	//IRRADIAÇÃO
	sensor.irra = ads.readADC_SingleEnded(1) / 10.95;

	//UVA
	float valorUV = ads.readADC_SingleEnded(0) * 0.125;
	sensor.uva = Calcula_nivel_UV(valorUV);

	//SELF DATA
	//Battery level
	internal.battery = (ads.readADC_SingleEnded(3)*0.03571428)-500;

	DEBUG_MSG("Battery: %f\n", internal.battery);
	if (internal.battery > 100.0)
	{
		internal.battery = 100.0;
	} else if (internal.battery < 0.0) {
		internal.battery = 0.0;
	}

	//3.3V Power
	internal.power = ((float)ESP.getVcc()*0.9322)/1023.0;

	//EPOCH TIME
	sensor.unixtime = now();

	return toJSON(&sensor);
}

void MQTT_connect()
{
	int8_t ret;

	if (mqtt.connected())
	{
		return;
	}

	if(WiFi.status() != WL_CONNECTED){
		return;
	}


	uint8_t retries = 3;
	Serial.println("Connecting to MQTT... ");//delay
	while ((ret = mqtt.connect()) != 0)
	{
		Serial.println("Retrying MQTT connection in 5 seconds...");
		Serial.println(mqtt.connectErrorString(ret));
		mqtt.disconnect();
		delay(5000);
		retries--;
		
		if (retries == 0)
		{
			failed = false;
			return;
		}
	}

	failed = true;
	Serial.println("MQTT Connected!");
}

void displayInfo()
{
  Serial.println(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

//Teste
// const int NTP_PACKET_SIZE = 48;
// byte packetBuffer[NTP_PACKET_SIZE];
// bool requestNTP = false;

// void sendNTPpacket(IPAddress &address)
// {
//   memset(packetBuffer, 0, NTP_PACKET_SIZE);
//   packetBuffer[0] = 0b11100011;
//   packetBuffer[1] = 0;
//   packetBuffer[2] = 6;
//   packetBuffer[3] = 0xEC;
//   packetBuffer[12] = 49;
//   packetBuffer[13] = 0x4E;
//   packetBuffer[14] = 49;
//   packetBuffer[15] = 52;
//   Udp.beginPacket(address, 123);
//   Udp.write(packetBuffer, NTP_PACKET_SIZE);
//   Udp.endPacket();
// }

void setup()
{
	#ifdef DEBUG_ESP_PORT
		DEBUG_ESP_PORT.begin(9600);
		DEBUG_MSG("\n> Start System\nInitializing...\n\n");
		DEBUG_MSG("\n-------- Serial Debug --------\nEstação  Meteorologica  Online\n");
		DEBUG_MSG("Firmware Version:        %s\n", FW_VERSION);
		DEBUG_MSG("Chip ID:              %d\n", ESP.getChipId());
		DEBUG_MSG("MAC Address: %s\n", WiFi.macAddress().c_str());
		DEBUG_MSG("Free Memory:             %d\n", ESP.getFreeHeap());
		DEBUG_MSG("By:               Agritech, CA\n------------------------------\n");
	#endif

	//Building topic
	uint8_t mac[6];
	WiFi.macAddress(mac);
	char idBuffer[13];

	sprintf(idBuffer, "%02x%02x%02x%02x%02x%02x",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

	sprintf((char*)topic_Rain,"estacao/%s/data_rain", idBuffer);
	sprintf((char*)topic_Location,"estacao/%s/data_location", idBuffer);
	sprintf((char*)topic_Wind,"estacao/%s/data_wind", idBuffer);
	sprintf((char*)topic_Air,"estacao/%s/data_air", idBuffer);
	sprintf((char*)topic_Sun,"estacao/%s/data_sun", idBuffer);
	sprintf((char*)topic_Internal,"estacao/%s/data_internal", idBuffer);
	sprintf((char*)topic_firmware,"estacao/%s/firmware", idBuffer);

	DEBUG_MSG("Topic is: %s\n", topic_Rain);//

	attachInterrupt(SPEEDPIN, anemometro, FALLING);
	attachInterrupt(PLUVPIN, pluviometro, FALLING);

	WiFi.mode(WIFI_STA);
	WiFiManager wifiManager;
	wifiManager.autoConnect("Agritech EMON");

	//Hardware I2C initializing
	Wire.begin(SDA,SCL);

	//Serial GPS
	DEBUG_MSG("Inicializando GPS");
	SerialGPS.begin(GPSBAUND);

	pinMode(ENABLE_GPS_PIN, OUTPUT);
	digitalWrite(ENABLE_GPS_PIN, HIGH);

	uint32_t waitGPS = 180000;

	while (!gps.location.isValid() || !gps.altitude.isValid() || !gps.date.isValid() || !gps.time.isValid())
	{
		while (SerialGPS.available() > 0)
		{
			gps.encode(SerialGPS.read());
		}
		if(millis()>waitGPS)
		{
			displayInfo();
			waitGPS+= 180000;
			digitalWrite(ENABLE_GPS_PIN, LOW);
			delay(2000);
			digitalWrite(ENABLE_GPS_PIN, HIGH);
		}
	}

	/*******Teste inicio******/
	/*Udp.begin(localPort);

	IPAddress ntpServerIP;
	while (Udp.parsePacket() > 0) ;

	while(!requestNTP & (WiFi.status() == WL_CONNECTED)){
		Serial.println("Transmit NTP Request");
		WiFi.hostByName(ntpServerName, ntpServerIP);
		Serial.print(ntpServerName);
		Serial.print(": ");
		Serial.println(ntpServerIP);

		sendNTPpacket(ntpServerIP);

		uint32_t beginWait = millis();
		while (millis() - beginWait < 1500) {
			requestNTP = false;
			int size = Udp.parsePacket();
			if (size >= NTP_PACKET_SIZE) {
				Serial.println("Receive NTP Response");
				Udp.read(packetBuffer, NTP_PACKET_SIZE);
				unsigned long secsSince1900;
				secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
				secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
				secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
				secsSince1900 |= (unsigned long)packetBuffer[43];
				setTime(secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR);
				requestNTP = true;
				break;
			}
		}
	}*/
	/********Teste fim*******/

	setTime(gps.time.hour(),
			gps.time.minute(),
			gps.time.second(),
			gps.date.day(),
			gps.date.month(),
			gps.date.year());
	//adjustTime(time_offset);u
	sensor.lat = gps.location.lat(); //-6.16881343;
	sensor.lng = gps.location.lng(); //-38.4888910;
	sensor.altitude = gps.altitude.meters();

	displayInfo();
	digitalWrite(ENABLE_GPS_PIN, LOW);

	//ADS1115 initializing
	ads.setGain(GAIN_ONE); // 1x gain   +/- 4.096V  1 bit = 0.125mV
	ads.begin();

	//BME680 initializing
	if(!bme.begin(0x76))
	{
		DEBUG_MSG("[Erro] BME680 does not initialize\n");
	} else {
		// Set up oversampling and filter initialization
		bme.setSampling(
			Adafruit_BME280::MODE_NORMAL,
			Adafruit_BME280::SAMPLING_X2,
			Adafruit_BME280::SAMPLING_X16,
			Adafruit_BME280::SAMPLING_X2,
			Adafruit_BME280::FILTER_X16,
			Adafruit_BME280::STANDBY_MS_20
		);
		// bme.setTemperatureOversampling(BME680_OS_8X);
		// bme.setHumidityOversampling(BME680_OS_2X);
		// bme.setPressureOversampling(BME680_OS_4X);
		// bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
		// bme.setGasHeater(320, 150);
	}

	mqtt.subscribe(&firmwareupdate);

	delay(2000);

	//while (WiFi.status() == WL_CONNECTED)
	//{
	//	DEBUG_MSG(".");
	//	delay(500);
	//}
}

void loop()
{
	static uint32_t millisec = 0, millisec2 = 0;//, millisec3 = 0;
	static uint8_t countPush = 0;

	/************************* Listen Subscription *********************************/

	Adafruit_MQTT_Subscribe *subscription;
	while ((subscription = mqtt.readSubscription(5000))) {
		// Check firmware version
		if (subscription == &firmwareupdate){
			DEBUG_MSG("New Version: %s\n", firmwareupdate.lastread);
			if (strcmp((char *)firmwareupdate.lastread, FW_VERSION) == 0)
			{
				DEBUG_MSG("Já atualizado");
			} else {
				t_httpUpdate_return ret = ESPhttpUpdate.update(espWifi, SERVER_UPDATE);
				switch(ret) {
					case HTTP_UPDATE_FAILED:
						DEBUG_MSG("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
						break;
					case HTTP_UPDATE_NO_UPDATES:
						DEBUG_MSG("[update] Update no Update.\n");
						break;
					case HTTP_UPDATE_OK:
						DEBUG_MSG("[update] Update ok.\n");// may not be called since we reboot the ESP
						break;
				}
			}
    	}
	}

	windDirection();

	if(!failed){
		if((millis() - millisec2) > 30000){
			millisec2 = millis();
			MQTT_connect();
		}
	} else {
		MQTT_connect();
	}


	if((millis() - millisec) > INTERVAL)
	{
		millisec += INTERVAL;

		char* dado = readSensor();

		uint8_t x = 0;

		while(!estacao.publish(dado))
		{
			if(x > 3)
			{
				countPush++;
				if(countPush > 10){
					countPush = 0;
					//buffer.push(sensor);
				}
				#ifdef DEBUG
					Serial << "Falha ao publicar: 1" << endl;
				#endif
				break;
			}
			x++;
		}
		delete[] dado;

		String data_loc = "{\"lat\": " + String(sensor.lat, 8) +
						", \"long\": " + String(sensor.lng, 8) +
						", \"altitude\": " + String(sensor.altitude) +
						", \"unix_time\": " + String(sensor.unixtime) + "}";
		int dataLength = data_loc.length()+1;
		char* dado2 = new char[dataLength];
		strcpy(dado2, data_loc.c_str());
		x = 0;
		while(!estacao_loc.publish(dado2))
		{
			if(x > 3)
			{
				#ifdef DEBUG
					Serial << "Falha ao publicar: 2" << endl;
				#endif
				break;
			}
			x++;
		}
		delete[] dado2;

		String data_wind = "{\"wind_speed\": " + String(sensor.windSpeed, 2) +
						", \"wind_direction\": " + String(sensor.windDirection) +
						", \"unix_time\": " + String(sensor.unixtime) + "}";
		Serial.println(data_wind);
		int dataLength2 = data_wind.length()+1;
		char* dado3 = new char[dataLength2];
		strcpy(dado3, data_wind.c_str());
		x = 0;
		while(!estacao_wind.publish(dado3))
		{
			if(x > 3)
			{
				#ifdef DEBUG
					Serial << "Falha ao publicar: 3" << endl;
				#endif
				break;
			}
			x++;
		}
		delete[] dado3;

		String data_air = "{\"temperature\": " + String(sensor.temperature, 2) +
						", \"humidity\": " + String(sensor.humidity, 2) +
						", \"pressure\": " + String(sensor.pressure, 2) +
						", \"air_quality\": " + String(sensor.iaq) +
						", \"unix_time\": " + String(sensor.unixtime) + "}";
		Serial.println(data_air);
		int dataLength3 = data_air.length()+1;
		char* dado4 = new char[dataLength3];
		strcpy(dado4, data_air.c_str());
		x = 0;
		while(!estacao_air.publish(dado4))
		{
			if(x > 3)
			{
				#ifdef DEBUG
					Serial << "Falha ao publicar: 4" << endl;
				#endif
				break;
			}
			x++;
		}
		delete[] dado4;

		String data_sun = "{\"solar_irra\": " + String(sensor.irra, 2) +
						", \"uva\": " + String(sensor.uva) +
						", \"unix_time\": " + String(sensor.unixtime) + "}";
		Serial.println(data_sun);
		int dataLength4 = data_sun.length()+1;
		char* dado5 = new char[dataLength4];
		strcpy(dado5, data_sun.c_str());
		x = 0;
		while(!estacao_sun.publish(dado5))
		{
			if(x > 3)
			{
				#ifdef DEBUG
					Serial << "Falha ao publicar: 5" << endl;
				#endif
				break;
			}
			x++;
		}
		delete[] dado5;

		String data_internal = "{\"direction\": " + String(0, 2) +
						", \"battery_level\": " + String(internal.battery, 2) +
						", \"battery_voltage\": " + String(internal.power, 2) +
						", \"panel_voltage\": " + String(0, 2) +
						", \"up_time\": " + String(millis()/1000) +
						", \"unix_time\": " + String(sensor.unixtime) + "}";
		Serial.println(data_internal);
		int dataLength5 = data_internal.length()+1;
		char* dado6 = new char[dataLength5];
		strcpy(dado6, data_internal.c_str());
		x = 0;
		while(!estacao_internal.publish(dado6))
		{
			if(x > 3)
			{
				#ifdef DEBUG
					Serial << "Falha ao publicar: 6" << endl;
				#endif
				break;
			}
			x++;
		}
		delete[] dado6;
	}
}