// Emon estação meteorológica baseada em ESP8266
// Desenvolvimente 2018-2019

/* Definições de pinos e constantes */

#include <Streaming.h>
#include <Adafruit_MQTT_Client.h>
#include <Adafruit_MQTT.h>
#include <Wire.h>
#include "Adafruit_BME680.h"
#include "Adafruit_BME280.h"
#include <Time.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ESP8266httpUpdate.h>
//#include <FS.h>
#include <Adafruit_ADS1015.h>
//#include <ESPBuffer.cpp>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const char ntpServerName[] = "us.pool.ntp.org";
#include <WiFiUdp.h>

#define FW_VERSION "1.0.2"
#define SERVER_UPDATE "192.168.1.111/firmware.bin"

//#define MQTTSERVER "io.adafruit.com"
//#define MQTTSERVER "cosmos.agritechsemiarido.com.br"
#define MQTTSERVER "tcp://demo.thingsboard.io"
#define MQTTPORT 1883
#define MQTTUSER "guilhermenunes_asp"
#define MQTTPASS ""

#define DEBUG //Comentar para desabilitar o debug
#define MAXRETRIES 3
#define WIFI_TIMEOUT 60000
#define AP_TIMEOUT 120000
#define SPEEDPIN 13//D5
#define PLUVPIN 2//D4
#define SDA 4
#define SCL 5
#define RX 14
#define TX 12
#define ENABLE_GPS_PIN 15
#define GPSBAUND 9600

//D=107.55mm A=53.775^2*pi=9084.7025mm^2 V100mm=90847.025mm^3
#define SCALERAIN 0.4042//0.6915//0.2391//0.27397//1//0.1787 //Escala calculada experimentalmente
#define SCALEWSPEED 0.571769744 // 0.439822972 // escala de medição da velocidade do vento
#define INTERVAL 120000 //milissegundos

#define SEALEVELPRESSURE_HPA (1013.25)

#define HYSTERESIS_DELAY_ANE 5 //ms
#define HYSTERESIS_DELAY_PLU 300 //ms