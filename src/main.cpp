#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <util.h>
#include <CircularBuffer.h>
#include "PubSubClient.h" // Biblioteca MQTT


#define DEBUG_ESP_PORT Serial

#ifdef DEBUG_ESP_PORT
#define DEBUG_MSG(args...) DEBUG_ESP_PORT.printf(args)
#else
#define DEBUG_MSG(...)
#endif

//                               |   0 |   1  |    2   |   3  |   4  |   5  |    6   |  7  |   8  |   9  |  10   |  11 |  12  |  13  |   14   |  15 |
//                               |Norte|      |Noroeste|      | Oeste|      |Sudoeste|     |  Sul |      |Sudeste|     | Leste|      |Nordeste|     |
const float constDirection[16] = {0.411, 1.377, 1.202, 2.774, 2.721, 0.331, 2.248, 2.540, 1.803, 0.824, 0.749, 2.879, 0.124, 1.980, 0.223, 0.584};
// st float constDirection[16] = {0.411, 0.584, 0.223,   0.331, 0.124, 0.824, 0.749,  1.980, 1.803, 2.540, 2.248, 2.879, 2.721, 2.774, 1.202,  1.377};

const char* mqtt_server = "demo.thingsboard.io"; // Endereço do servidor AWS
#define TOKEN "NXaPGNp0ZCHkwgBbhcWM" // Token do dispositivo
#define MSG_BUFFER_SIZE (150) // Tamanho de buffer
char msg[MSG_BUFFER_SIZE]; // String msg

ADC_MODE(ADC_VCC);

//WiFiClient espWifi; // Objeto para lib adafruit
WiFiClient espClient; // Objeto para lib PubSubClient
PubSubClient client(espClient); // Conexão MQTT

void reconnect_mqtt() {
  int try_reconnect_mqtt=0;
  // Enquanto não tiver conectado
  while (!client.connected()) {
    try_reconnect_mqtt++;
    Serial.print("try_reconnect_mqtt: ");
    Serial.println(try_reconnect_mqtt);
    //portYIELD_FROM_ISR();
    Serial.print("Tentando conexão MQTT...");
    // Cria ID randomico
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Tentando conexão MQTT
    if (client.connect(clientId.c_str(), TOKEN, NULL)) {
      Serial.println("Conectado");
      try_reconnect_mqtt = 0;
    } else {
      Serial.print("Erro de codigo = ");
      Serial.println(client.state());
      Serial.println("Tentanto novamente em 5 segundos");
      delay(5000);
    }
	if(try_reconnect_mqtt > 15) {
		//reinicia o dispositivo	
		//ESP.restart();
	}
  }
}

Adafruit_ADS1115 ads;

TinyGPSPlus gps;

SoftwareSerial SerialGPS(RX, TX);

/**** NTPTime *****/

WiFiUDP Udp;
unsigned int localPort = 8888;
const int timeZone = 0;
time_t getNtpTime();

/**** Tópicos MQTT ******/

/* const char *topic_Rain = "estacao/invalidTopic/data_rain";
const char *topic_Location = "estacao/invalidTopic/data_location";
const char *topic_Wind = "estacao/invalidTopic/data_wind";
const char *topic_Air = "estacao/invalidTopic/data_air";
const char *topic_Sun = "estacao/invalidTopic/data_sun";
const char *topic_Internal = "estacao/invalidTopic/data_internal";
const char *topic_firmware = "estacao/invalidTopic/firmware";*/

//Adafruit_MQTT_Client mqtt(&espWifi, MQTTSERVER, MQTTPORT, MQTTUSER, MQTTPASS);
/*Adafruit_MQTT_Publish estacao = Adafruit_MQTT_Publish(&mqtt, topic_Rain);
Adafruit_MQTT_Publish estacao_loc = Adafruit_MQTT_Publish(&mqtt, topic_Location);
Adafruit_MQTT_Publish estacao_wind = Adafruit_MQTT_Publish(&mqtt, topic_Wind);
Adafruit_MQTT_Publish estacao_air = Adafruit_MQTT_Publish(&mqtt, topic_Air);
Adafruit_MQTT_Publish estacao_sun = Adafruit_MQTT_Publish(&mqtt, topic_Sun);
Adafruit_MQTT_Publish estacao_internal = Adafruit_MQTT_Publish(&mqtt, topic_Internal);
Adafruit_MQTT_Subscribe firmwareupdate = Adafruit_MQTT_Subscribe(&mqtt, topic_firmware); */

Adafruit_BME280 bme;

volatile int countSpeed = 0;
volatile int countRain = 0;
volatile uint32_t lastcount = 0;
volatile uint32_t lastcount2 = 0;

bool failed = true;

byte modaDirection[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

struct sensores
{
	float temperature;	   //----|
	float humidity;		   //-------|
	float pressure;		   //-------|BME680
	uint16_t iaq;		   //---------|
	float altitude;		   //-------|
	float rain;			   //-------------|
	float windSpeed;	   //--------|Sensores mecanicos
	uint8_t windDirection; //--|
	float lat;			   //------------|
	float lng;			   //------------|GY-NEO6MV2
	// float altitude;//-------|
	uint8_t uva;		  //--------------|
	float irra;			  //-------------|
	uint32_t unixtime;	  //----|
} sensor, auxSensor; //chegará a 41Bytes

struct data_rain_struct
{
	float rain;
	uint16_t count_lighing;
	uint32_t unixtime;
} data_rain_obj, data_rain_obj_aux;

struct data_loc_struct
{
	float lat;
	float lng;
	float altitude;
	uint32_t unixtime;
} data_loc_obj, data_loc_obj_aux;

struct data_wind_struct
{
	float windSpeed;
	uint8_t windDirection;
	uint32_t unixtime;
} data_wind_obj, data_wind_obj_aux;

struct data_air_struct
{
	float temperature;
	float humidity;
	float pressure;
	uint16_t air_quality;
	uint32_t unixtime;
} data_air_obj, data_air_obj_aux;

struct data_sun_struct
{
	float solar_irra;
	uint8_t uva;
	uint32_t unixtime;
} data_sun_obj, data_sun_obj_aux;

struct data_internal_struct
{
	char sentido;
	float bateria;
	float fonte;
	char painel;
	uint32_t up_time;
	uint32_t unixtime;
}data_internal_obj, data_internal_obj_aux;

CircularBuffer<data_rain_struct, 5> my_buf_sensores;
CircularBuffer<data_loc_struct, 5> data_loc_buffer;
CircularBuffer<data_wind_struct, 5> data_wind_buffer;
CircularBuffer<data_air_struct, 5> data_air_buffer;
CircularBuffer<data_sun_struct, 5> data_sun_buffer;
CircularBuffer<data_internal_struct, 5> data_internal_buffer;

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
	else
	{
		return 0;
	}
}

ICACHE_RAM_ATTR void anemometro()
{
	if (millis() - lastcount2 > HYSTERESIS_DELAY_ANE)
	{
		countSpeed++;
	}

	lastcount2 = millis();
}

ICACHE_RAM_ATTR void pluviometro()
{
	if (millis() - lastcount > HYSTERESIS_DELAY_PLU)
	{
		countRain++;
	}

	lastcount = millis();
}

void windDirection()
{
	uint8_t x;

	float AIN2 = ads.readADC_SingleEnded(2) * 0.000125;

	uint8_t temporaryDirection = 0;
	float dirTemp = 10.0;

	for (x = 0; x < 16; x++)
	{
		if (fabs(constDirection[x] - AIN2) < dirTemp)
		{
			dirTemp = fabs(constDirection[x] - AIN2);
			temporaryDirection = x;
		}
	}
	modaDirection[temporaryDirection]++;
}

char *toJSON(struct sensores *dataJson)
{
	String dataBuffer = //"{\"temperature\": "   + String(dataJson->temperature) +
		//", \"humidity\": "     + String(dataJson->humidity)    +
		//", \"pressure\": "     + String(dataJson->pressure)    +
		//", \"air_quality\": "  + String(dataJson->iaq)         +
		//", \"windspeed\": "    + String(dataJson->windSpeed)   +
		//", \"winddirection\": "    + String(dataJson->windDirection)   +
		"{\"rain\": " + String(dataJson->rain) +
		", \"count_lighing\": 0" +
		", \"unixtime\": " + String(dataJson->unixtime) + "}";
	//", \"altitude\": "     + String(dataJson->altitude)    +
	//", IS: " + String(dataJson->lat, 6) +
	//", UVA: " + String(dataJson->lat, 6) +
	//", lat: " + String(dataJson->lat, 6) +
	//", long: " + String(dataJson->lng, 6) +
	int dataLength = dataBuffer.length() + 1;
	char *data = new char[dataLength];
	strcpy(data, dataBuffer.c_str());
	//data = &dataBuffer[0u];

	Serial.print("Dados em readSensor: ");
	for (int i = 0; i < dataLength; i++)
	{
		Serial.print(data[i]);
	}
	Serial.println("");
	//char* data = (char*)"0";
	return data;
}

char *readSensor()
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
	sensor.windSpeed = (countSpeed * SCALEWSPEED) / (INTERVAL / 1000);
	countSpeed = 0;

	//BIRUTA
	uint8_t x, xt = 0;
	for (x = 0; x < 16; x++)
	{
		if (modaDirection[x] > xt)
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
	internal.battery = (ads.readADC_SingleEnded(3) * 0.03571428) - 500;

	DEBUG_MSG("Battery: %f\n", internal.battery);
	if (internal.battery > 100.0)
	{
		internal.battery = 100.0;
	}
	else if (internal.battery < 0.0)
	{
		internal.battery = 0.0;
	}

	//3.3V Power
	internal.power = ((float)ESP.getVcc() * 0.9322) / 1023.0;

	//EPOCH TIME
	sensor.unixtime = now();

	return toJSON(&sensor);
}

/* void MQTT_connect()
{
	int8_t ret;

	if (mqtt.connected())
	{
		return;
	}

	if (WiFi.status() != WL_CONNECTED)
	{
		return;
	}

	uint8_t retries = 3;
	Serial.println("Connecting to MQTT... "); //delay
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
 */
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
		if (gps.time.hour() < 10)
			Serial.print(F("0"));
		Serial.print(gps.time.hour());
		Serial.print(F(":"));
		if (gps.time.minute() < 10)
			Serial.print(F("0"));
		Serial.print(gps.time.minute());
		Serial.print(F(":"));
		if (gps.time.second() < 10)
			Serial.print(F("0"));
		Serial.print(gps.time.second());
		Serial.print(F("."));
		if (gps.time.centisecond() < 10)
			Serial.print(F("0"));
		Serial.print(gps.time.centisecond());
	}
	else
	{
		Serial.print(F("INVALID"));
	}

	Serial.println();
}

//Teste
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];
bool requestNTP = false;

void sendNTPpacket(IPAddress &address)
{
	memset(packetBuffer, 0, NTP_PACKET_SIZE);
	packetBuffer[0] = 0b11100011;
	packetBuffer[1] = 0;
	packetBuffer[2] = 6;
	packetBuffer[3] = 0xEC;
	packetBuffer[12] = 49;
	packetBuffer[13] = 0x4E;
	packetBuffer[14] = 49;
	packetBuffer[15] = 52;
	Udp.beginPacket(address, 123);
	Udp.write(packetBuffer, NTP_PACKET_SIZE);
	Udp.endPacket();
}

// Funçao para verificar conexão WiFi
int reads=0;
void verificaConexao(){
	
    while(WiFi.status() != WL_CONNECTED){
      Serial.println("Sem conexão WiFi!");
      reads++;
      delay(1000);
      if(reads > 30){
        Serial.println("Sem conexão. Resetando!");
        ESP.restart();
      }
    }
}

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

	sprintf(idBuffer, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	/* sprintf((char *)topic_Rain, "estacao/%s/data_rain", idBuffer);
	sprintf((char *)topic_Location, "estacao/%s/data_location", idBuffer);
	sprintf((char *)topic_Wind, "estacao/%s/data_wind", idBuffer);
	sprintf((char *)topic_Air, "estacao/%s/data_air", idBuffer);
	sprintf((char *)topic_Sun, "estacao/%s/data_sun", idBuffer);
	sprintf((char *)topic_Internal, "estacao/%s/data_internal", idBuffer);
	sprintf((char *)topic_firmware, "estacao/%s/firmware", idBuffer); */

	//DEBUG_MSG("Topic is: %s\n", topic_Rain); //

	attachInterrupt(SPEEDPIN, anemometro, FALLING);
	attachInterrupt(PLUVPIN, pluviometro, FALLING);
	pinMode(D3, INPUT_PULLUP);

	WiFi.mode(WIFI_STA);
	WiFiManager wifiManager;

	wifiManager.autoConnect("Agritech EMON");

	//Hardware I2C initializing
	Wire.begin(SDA, SCL);

	//Serial GPS
	DEBUG_MSG("Inicializando GPS");
	SerialGPS.begin(GPSBAUND);

	pinMode(ENABLE_GPS_PIN, OUTPUT);
	// digitalWrite(ENABLE_GPS_PIN, HIGH);

	// uint32_t waitGPS = 180000;

	// while (!gps.location.isValid() || !gps.altitude.isValid() || !gps.date.isValid() || !gps.time.isValid())
	// {
	// 	while (SerialGPS.available() > 0)
	// 	{
	// 		gps.encode(SerialGPS.read());
	// 	}
	// 	if(millis()>waitGPS)
	// 	{
	// 		displayInfo();
	// 		waitGPS+= 180000;
	// 		digitalWrite(ENABLE_GPS_PIN, LOW);
	// 		delay(2000);
	// 		digitalWrite(ENABLE_GPS_PIN, HIGH);
	// 	}
	// }

	/*Teste inicio**/
	Udp.begin(localPort);

	IPAddress ntpServerIP;
	while (Udp.parsePacket() > 0)
		;

	while (!requestNTP & (WiFi.status() == WL_CONNECTED))
	{
		Serial.println("Transmit NTP Request");
		WiFi.hostByName(ntpServerName, ntpServerIP);
		Serial.print(ntpServerName);
		Serial.print(": ");
		Serial.println(ntpServerIP);

		sendNTPpacket(ntpServerIP);

		uint32_t beginWait = millis();
		while (millis() - beginWait < 1500)
		{
			requestNTP = false;
			int size = Udp.parsePacket();
			if (size >= NTP_PACKET_SIZE)
			{
				Serial.println("Receive NTP Response");
				Udp.read(packetBuffer, NTP_PACKET_SIZE);
				unsigned long secsSince1900;
				secsSince1900 = (unsigned long)packetBuffer[40] << 24;
				secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
				secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
				secsSince1900 |= (unsigned long)packetBuffer[43];
				setTime(secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR);
				requestNTP = true;
				break;
			}
		}
	}
	/**Teste fim*/

	// setTime(gps.time.hour(),
	// 		gps.time.minute(),
	// 		gps.time.second(),
	// 		gps.date.day(),
	// 		gps.date.month(),
	// 		gps.date.year());
	//adjustTime(time_offset);u
	sensor.lat = -7.982696;	 // gps.location.lat();               -6.161616, -38.451155
	sensor.lng = -38.289547; // gps.location.lng();
	sensor.altitude = 612;	 // gps.altitude.meters();

	displayInfo();
	digitalWrite(ENABLE_GPS_PIN, LOW);

	//ADS1115 initializing
	ads.setGain(GAIN_ONE); // 1x gain   +/- 4.096V  1 bit = 0.125mV
	ads.begin();

	//BME680 initializing
	if (!bme.begin(0x76))
	{
		DEBUG_MSG("[Erro] BME280 does not initialize\n");
	}
	else
	{
		// Set up oversampling and filter initialization
		bme.setSampling(
			Adafruit_BME280::MODE_NORMAL,
			Adafruit_BME280::SAMPLING_X2,
			Adafruit_BME280::SAMPLING_X16,
			Adafruit_BME280::SAMPLING_X2,
			Adafruit_BME280::FILTER_X16,
			Adafruit_BME280::STANDBY_MS_20);
		// bme.setTemperatureOversampling(BME680_OS_8X);
		// bme.setHumidityOversampling(BME680_OS_2X);
		// bme.setPressureOversampling(BME680_OS_4X);
		// bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
		// bme.setGasHeater(320, 150);
	}

	//mqtt.subscribe(&firmwareupdate);
	client.setServer(mqtt_server, 1883); // Conecta com o servidor MQTT

	delay(2000);

	//while (WiFi.status() == WL_CONNECTED)
	//{
	//	DEBUG_MSG(".");
	//	delay(500);
	//}
}

void loop()
{
	//verificaConexao();
	if (!client.connected()) { // Caso não esteja conectado ao servidor MQTT
		reconnect_mqtt(); // Reconecta
	}
	client.loop();
	static uint32_t millisec = 0, millisec2 = 0; //, millisec3 = 0;
	static uint8_t countPush = 0;

	/*** Listen Subscription *****/

	Adafruit_MQTT_Subscribe *subscription;
	/* while ((subscription = mqtt.readSubscription(5000)))
	{
		// Check firmware version
		if (subscription == &firmwareupdate)
		{
			DEBUG_MSG("New Version: %s\n", firmwareupdate.lastread);
			if (strcmp((char *)firmwareupdate.lastread, FW_VERSION) == 0)
			{
				DEBUG_MSG("Já atualizado");
			}
			else
			{
				t_httpUpdate_return ret = ESPhttpUpdate.update(espWifi, SERVER_UPDATE);
				switch (ret)
				{
				case HTTP_UPDATE_FAILED:
					DEBUG_MSG("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
					break;
				case HTTP_UPDATE_NO_UPDATES:
					DEBUG_MSG("[update] Update no Update.\n");
					break;
				case HTTP_UPDATE_OK:
					DEBUG_MSG("[update] Update ok.\n"); // may not be called since we reboot the ESP
					break;
				}
			}
		}
	}
 */
	windDirection();

	if (!failed)
	{
		if ((millis() - millisec2) > 30000)
		{
			millisec2 = millis();
			//MQTT_connect();
			reconnect_mqtt(); // Reconecta
		}
	}
	else
	{
		//MQTT_connect();
		reconnect_mqtt(); // Reconecta
	}

	if ((millis() - millisec) > INTERVAL)
	{
		millisec += INTERVAL;
		//////////////////////////////////////////////////////////////////////////////////////////////////
		char *dado = readSensor();
		snprintf (msg, MSG_BUFFER_SIZE, "{\"topic_Rain\":%s}", dado);
		uint8_t x = 0;
		bool flag_sensor = !client.publish("v1/devices/me/telemetry", msg);
		
		#ifdef DEBUG
				//Serial << msg << endl;
		#endif
		data_rain_obj.rain = sensor.rain;
		data_rain_obj.unixtime = sensor.unixtime;
		while (flag_sensor)
		{
			if (x > 3)
			{
				my_buf_sensores.push(data_rain_obj);
#ifdef DEBUG
				Serial << "Falha ao publicar: 1" << endl;
#endif
				break;
			}
			x++;
		}
		delete[] dado;
		while (flag_sensor == false && my_buf_sensores.isEmpty() == false)
		{
			data_rain_obj_aux = my_buf_sensores.last();
			String data_sensor = "{\"rain\": " + String(data_rain_obj_aux.rain) +
								 ", \"count_lighing\": 0" +
								 ", \"unixtime\": " + String(data_rain_obj_aux.unixtime) + "}";
								 Serial.println(data_sensor);
			int dataLengthRain = data_sensor.length() + 1;
			char *dado1Copied = new char[dataLengthRain];
			strcpy(dado1Copied, data_sensor.c_str());

			if (client.publish("v1/devices/me/telemetry", dado1Copied))
			{
				my_buf_sensores.pop();
				break;
			}
			else
			{
				while (my_buf_sensores.isEmpty() == false)
				{
					my_buf_sensores.pop();
					if (my_buf_sensores.isEmpty() == true)
					{
						break;
					}
				}
				DEBUG_MSG("Erro na hora de publicar a recaida\n");
				break;
			}
		}
		//////////////////////////////////////////////////////////////////////////////////////////////////

		String data_loc = "{\"loc_lat\": " + String(sensor.lat, 8) +
						  ", \"loc_long\": " + String(sensor.lng, 8) +
						  ", \"loc_altitude\": " + String(sensor.altitude) +
						  ", \"loc_unixtime\": " + String(sensor.unixtime) + "}";
		int dataLength = data_loc.length() + 1;
		char *dado2 = new char[dataLength];
		strcpy(dado2, data_loc.c_str());
		x = 0;
		data_loc_obj.lat = sensor.lat;
		data_loc_obj.lng = sensor.lng;
		data_loc_obj.altitude = sensor.altitude;
		data_loc_obj.unixtime = sensor.unixtime;
		bool flag_loc = !client.publish("v1/devices/me/telemetry", dado2);
		while (flag_loc)
		{
			if (x > 3)
			{
				data_loc_buffer.push(data_loc_obj);
#ifdef DEBUG
				Serial << "Falha ao publicar: 2" << endl;
#endif
				break;
			}
			x++;
		}
		delete[] dado2;
		while (flag_loc == false && data_loc_buffer.isEmpty() == false)
		{
			data_loc_obj_aux = data_loc_buffer.last();
			String data_loc_sensor = "{\"lat\": " + String(data_loc_obj_aux.lat, 8) +
									 ", \"long\": " + String(data_loc_obj_aux.lng, 8) +
									 ", \"altitude\": " + String(data_loc_obj_aux.altitude) +
									 ", \"unixtime\": " + String(data_loc_obj_aux.unixtime) + "}";
			int dataLengthLoc = data_loc_sensor.length() + 1;
			char *dado2Copied = new char[dataLengthLoc];
			strcpy(dado2Copied, data_loc_sensor.c_str());
			if (client.publish("v1/devices/me/telemetry", dado2Copied))
			{
				data_loc_buffer.pop();
				break;
			}
			else
			{
				DEBUG_MSG("Erro na hora de publicar a recaida\n");
				break;
			}
		}
		//////////////////////////////////////////////////////////////////////////////////////////////////
		String data_wind = "{\"wind_windspeed\": " + String(sensor.windSpeed, 2) +
						   ", \"wind_winddirection\": " + String(sensor.windDirection) +
						   ", \"wind_unixtime\": " + String(sensor.unixtime) + "}";
		Serial.println(data_wind);
		int dataLength2 = data_wind.length() + 1;
		char *dado3 = new char[dataLength2];
		strcpy(dado3, data_wind.c_str());
		x = 0;
		data_wind_obj.windSpeed = sensor.windSpeed;
		data_wind_obj.windDirection = sensor.windDirection;
		data_wind_obj.unixtime = sensor.unixtime;
		bool flag_winspeed = !client.publish("v1/devices/me/telemetry", dado3);
		while (flag_winspeed)
		{
			if (x > 3)
			{
				data_wind_buffer.push(data_wind_obj);
#ifdef DEBUG
				Serial << "Falha ao publicar: 3" << endl;
#endif
				break;
			}
			x++;
		}
		delete[] dado3;
		while (flag_winspeed == false && data_wind_buffer.isEmpty() == false)
		{
			data_wind_obj_aux = data_wind_buffer.last();
			String data_winspeed_sensor = "{\"windspeed\": " + String(data_wind_obj_aux.windSpeed, 2) +
										  ", \"winddirection\": " + String(data_wind_obj_aux.windDirection) +
										  ", \"unixtime\": " + String(data_wind_obj_aux.unixtime) + "}";
			int dataLengthWinSpeed = data_winspeed_sensor.length() + 1;
			char *dado3Copied = new char[dataLengthWinSpeed];
			strcpy(dado3Copied, data_winspeed_sensor.c_str());
			if (client.publish("v1/devices/me/telemetry", dado3Copied))
			{
				data_wind_buffer.pop();
				break;
			}
			else
			{
				DEBUG_MSG("Erro na hora de publicar a recaida\n");
				break;
			}
		}
		//////////////////////////////////////////////////////////////////////////////////////////////////
		String data_air = "{\"air_temperature\": " + String(sensor.temperature, 2) +
						  ", \"air_humidity\": " + String(sensor.humidity, 2) +
						  ", \"air_pressure\": " + String(sensor.pressure, 2) +
						  ", \"air_air_quality\": " + String(sensor.iaq) +
						  ", \"air_unixtime\": " + String(sensor.unixtime) + "}";
		Serial.println(data_air);
		int dataLength3 = data_air.length() + 1;
		char *dado4 = new char[dataLength3];
		strcpy(dado4, data_air.c_str());
		x = 0;
		data_air_obj.temperature = sensor.temperature;
		data_air_obj.humidity = sensor.humidity;
		data_air_obj.pressure = sensor.pressure;
		data_air_obj.air_quality = sensor.iaq;
		data_air_obj.unixtime = sensor.unixtime;

		bool flag_air = !client.publish("v1/devices/me/telemetry", dado4);
		while (flag_air)
		{
			if (x > 3)
			{
				data_air_buffer.push(data_air_obj);
#ifdef DEBUG
				Serial << "Falha ao publicar: 4" << endl;
#endif
				break;
			}
			x++;
		}
		delete[] dado4;
		while (flag_air == false && data_air_buffer.isEmpty() == false)
		{
			data_air_obj_aux = data_air_buffer.last();
			String data_air_sensor ="{\"temperature\": " + String(data_air_obj_aux.temperature, 2) +
						  ", \"humidity\": " + String(data_air_obj_aux.humidity, 2) +
						  ", \"pressure\": " + String(data_air_obj_aux.pressure, 2) +
						  ", \"air_quality\": " + String(data_air_obj_aux.air_quality) +
						  ", \"unixtime\": " + String(data_air_obj_aux.unixtime) + "}";
			int dataLengthAir = data_air_sensor.length() + 1;
			char *dado4Copied = new char[dataLengthAir];
			strcpy(dado4Copied, data_air_sensor.c_str());
			if (client.publish("v1/devices/me/telemetry", dado4Copied))
			{
				data_air_buffer.pop();
				break;
			}
			else
			{
				DEBUG_MSG("Erro na hora de publicar a recaida\n");
				break;
			}
		}
		//////////////////////////////////////////////////////////////////////////////////////////////////
		String data_sun = "{\"sun_solar_irra\": " + String(sensor.irra, 2) +
						  ", \"sun_uva\": " + String(sensor.uva) +
						  ", \"sun_unixtime\": " + String(sensor.unixtime) + "}";
		Serial.println(data_sun);
		int dataLength4 = data_sun.length() + 1;
		char *dado5 = new char[dataLength4];
		strcpy(dado5, data_sun.c_str());
		x = 0;
		data_sun_obj.solar_irra = sensor.irra;
		data_sun_obj.uva = sensor.uva;
		data_sun_obj.unixtime = sensor.unixtime;
		bool flag_sun = !client.publish("v1/devices/me/telemetry", dado5);
		while (flag_sun)
		{
			if (x > 3)
			{
				data_sun_buffer.push(data_sun_obj);
#ifdef DEBUG
				Serial << "Falha ao publicar: 5" << endl;
#endif
				break;
			}
			x++;
		}
		delete[] dado5;
		while (flag_sun == false && data_sun_buffer.isEmpty() == false)
		{
			data_sun_obj_aux = data_sun_buffer.last();
			String data_sun_sensor = "{\"solar_irra\": " + String(data_sun_obj_aux.solar_irra, 2) +
							  ", \"uva\": " + String(data_sun_obj_aux.uva) +
							  ", \"unixtime\": " + String(data_sun_obj_aux.unixtime) + "}";
			int dataLengthSun = data_sun_sensor.length() + 1;
			char *dado5Copied = new char[dataLengthSun];
			strcpy(dado5Copied, data_sun_sensor.c_str());
			if (client.publish("v1/devices/me/telemetry", dado5Copied))
			{
				data_sun_buffer.pop();
				break;
			}
			else
			{
				DEBUG_MSG("Erro na hora de publicar a recaida\n");
				break;
			}
		}
		//////////////////////////////////////////////////////////////////////////////////////////////////
		String data_internal = "{\"Internal_sentido\": " + String(0, 2) +
							   ", \"Internal_bateria\": " + String(internal.battery, 2) +
							   ", \"Internal_fonte\": " + String(internal.power, 2) +
							   ", \"Internal_painel\": " + String(0, 2) +
							   ", \"Internal_up_time\": " + String(millis() / 1000) +
							   ", \"Internal_unixtime\": " + String(sensor.unixtime) + "}";
		Serial.println(data_internal);
		int dataLength5 = data_internal.length() + 1;
		char *dado6 = new char[dataLength5];
		strcpy(dado6, data_internal.c_str());
		x = 0;
		data_internal_obj.bateria = internal.battery;
		data_internal_obj.fonte = internal.power;
		data_internal_obj.unixtime = sensor.unixtime;
		data_internal_obj.up_time = millis() / 1000;
		bool flag_internal = !client.publish("v1/devices/me/telemetry", dado6);
		while (flag_internal)
		{
			if (x > 3)
			{
				data_internal_buffer.push(data_internal_obj);
#ifdef DEBUG
				Serial << "Falha ao publicar: 6" << endl;
#endif
				break;
			}
			x++;
		}
		delete[] dado6;
		while (flag_internal == false && data_internal_buffer.isEmpty() == false)
		{
			data_internal_obj_aux = data_internal_buffer.last();
			String data_internal_sensor = "{\"sentido\": " + String(0, 2) +
							   ", \"bateria\": " + String(data_internal_obj_aux.bateria, 2) +
							   ", \"fonte\": " + String(data_internal_obj_aux.fonte, 2) +
							   ", \"painel\": " + String(0, 2) +
							   ", \"up_time\": " + String(data_internal_obj_aux.up_time) +
							   ", \"unixtime\": " + String(data_internal_obj_aux.unixtime) + "}";
			Serial.println(data_internal_sensor);
			int dataLengthInternal = data_internal_sensor.length() + 1;
			char *dado6Copied = new char[dataLengthInternal];
			strcpy(dado6Copied, data_internal_sensor.c_str());
			if (client.publish("v1/devices/me/telemetry", dado6Copied))
			{
				data_internal_buffer.pop();
				break;
			}
			else
			{
				DEBUG_MSG("Erro na hora de publicar a recaida\n");
				break;
			}
		}
	}
}