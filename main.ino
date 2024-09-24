#include "NMEA.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

// Configurações de WiFi e MQTT
const char *SSID = "Wokwi-GUEST";
const char *PASSWORD = "";
const char *BROKER_MQTT = "broker.hivemq.com";
const int BROKER_PORT = 1883;
const char *ID_MQTT = "leandro";
const char *TOPIC_PUB = "clirv/edge/challenge";

#define PUBLISH_DELAY 2000

WiFiClient espClient;
PubSubClient MQTT(espClient);
unsigned long publishUpdate = 0;

#define LEN(arr) ((int)(sizeof(arr) / sizeof(arr)[0]))

union {
  char bytes[4];
  float valor;
} velocidadeGPS;

float latitude;
float longitude;

// Instancia a leitura de dados GPS com o tipo de sentença GPRMC
NMEA gps(GPRMC);

const int analogPin = 34;  // Alterado para um pino ADC compatível no ESP32
const int ledCount = 10;

int ledPins[] = {
  19, 18, 32, 33, 25, 26, 27, 14, 12, 13
};

// Configuração do Display OLED
const byte SCREEN_WIDTH = 128;
const byte SCREEN_HEIGHT = 64;

const int OLED_RESET = -1;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void initWiFi() {
  Serial.printf("Conectando com a rede: %s\n", SSID);
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) delay(100);
  Serial.printf("Conectado com sucesso: %s\nIP: %s\n", SSID, WiFi.localIP().toString().c_str());
}

void initMQTT() {
  MQTT.setServer(BROKER_MQTT, BROKER_PORT);
}

void reconnectMQTT() {
  while (!MQTT.connected()) {
    Serial.printf("Tentando conectar com o Broker MQTT: %s\n", BROKER_MQTT);
    if (MQTT.connect(ID_MQTT)) {
      Serial.println("Conectado ao broker MQTT!");
    } else delay(2000);
  }
}

void setup() {
  Serial.begin(115200);      // Comunicação via monitor serial
  Serial2.begin(9600, SERIAL_8N1, 16, 17);       // Comunicação com o módulo GPS via Serial2

  // Inicializando display OLED
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Definindo todos os LEDs como saída
  for (int thisLed = 0; thisLed < ledCount; thisLed++) {
    pinMode(ledPins[thisLed], OUTPUT);
  }

  // Iniciando Wifi e MQTT
  initWiFi();
  initMQTT();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) initWiFi();
  if (!MQTT.connected()) reconnectMQTT();

  // Leitura da bateria simulada (potenciômetro)
  int sensorReading = analogRead(analogPin);               // Leitura do nível da bateria
  int ledLevel = map(sensorReading, 0, 4095, 0, ledCount); // Mapeia para a quantidade de LEDs
  int porcentLevel = map(sensorReading, 0, 4095, 0, 100);  // Mapeia para porcentagem

  // Controle de LEDs baseado na leitura analógica
  for (int thisLed = 0; thisLed < ledCount; thisLed++) {
    if (thisLed < ledLevel) {
      digitalWrite(ledPins[thisLed], HIGH);
    } else {
      digitalWrite(ledPins[thisLed], LOW);
    }
  }

  // Exibe nível de bateria no display OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Bateria: ");
  display.print(porcentLevel);
  display.println("%");

  // Leitura dos dados do GPS via Serial2
  while (Serial2.available()) {             // Verifica se há dados disponíveis na serial
    char serialData = Serial2.read();       // Lê dados da serial do GPS

    if (gps.decode(serialData)) {           // Decodifica os dados do GPS
      if (gps.gprmc_status() == 'A') {      // Verifica se o status do GPS é 'A' (ativo)
        velocidadeGPS.valor = gps.gprmc_speed(KMPH); // Obtém a velocidade em km/h
      } else {
        velocidadeGPS.valor = 0;           // Se status for 'V', define velocidade como 0
      }

      // Obtém latitude e longitude
      latitude  = gps.gprmc_latitude();
      longitude = gps.gprmc_longitude();
    }
  }

  // Exibe os dados no display OLED
  display.setCursor(0, 12);
  display.println("Latitude: ");
  display.setCursor(0, 21);
  display.println(latitude, 8);

  display.setCursor(0, 33);
  display.println("Longitude: ");
  display.setCursor(0, 42);
  display.println(longitude, 8);

  display.setCursor(0, 54);
  display.print("Velocidade: ");
  display.print(velocidadeGPS.valor, 2);
  display.println("Km/h");
  display.display();

  // Chamada do loop do MQTT
  MQTT.loop();

  // Publica os dados via MQTT a cada intervalo definido
  if ((millis() - publishUpdate) >= PUBLISH_DELAY) {
    publishUpdate = millis();
    StaticJsonDocument<200> json;
    json["Bateria"] = porcentLevel;
    json["Latitude"] = latitude;
    json["Longitude"] = longitude;
    json["Velocidade"] = velocidadeGPS.valor;
    char buffer[200];
    serializeJson(json, buffer);
    MQTT.publish(TOPIC_PUB, buffer);
    Serial.println(buffer);
  }
}
