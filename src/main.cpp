#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include "Adafruit_TCS34725.h"

// --- Константы ---
#define MODULE_TYPE "sorting-center"
#define DISTANCE_THRESHOLD_MM 25

#define MOTOR_IN1 3       // Пины подключения к L298N
#define MOTOR_IN2 2
#define XSHUT 10          // Пин для отключения vl53l0x

#define MQTT_SERVER "192.168.8.100"
#define WIFI_SSID "Beeline_MF"
#define WIFI_PASS "$@ndr0nix"

// --- MQTT и сеть ---
WiFiClient espClient;
PubSubClient client(espClient);

// --- Датчики ---
Adafruit_VL53L0X vl53 = Adafruit_VL53L0X();
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

String mac_id = "";

// --- Функции ---
void sendIdentity() {
  String ip = WiFi.localIP().toString();
  String mac = WiFi.macAddress();

  client.publish(("module/" + mac + "/identity/type").c_str(), MODULE_TYPE);
  client.publish(("module/" + mac + "/identity/ip").c_str(), ip.c_str());
}

void setup_wifi() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  mac_id = WiFi.macAddress();
  Serial.println("\nWiFi connected, MAC: " + mac_id);
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect(mac_id.c_str())) {
      Serial.println("Connected to MQTT");
      client.subscribe(("module/" + mac_id + "/command").c_str());
      sendIdentity();
    } else {
      delay(1000);
    }
  }
}

void sendVL53() {
  VL53L0X_RangingMeasurementData_t measure;
  vl53.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {
    char topic[64];
    snprintf(topic, sizeof(topic), "module/%s/sensor/vl53l0x", mac_id.c_str());

    char msg[16];
    snprintf(msg, sizeof(msg), "%d", measure.RangeMilliMeter);
    client.publish(topic, msg);
  }
}

void sendColor() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  char topic[64];
  snprintf(topic, sizeof(topic), "module/%s/sensor/tcs3472", mac_id.c_str());

  char msg[64];
  snprintf(msg, sizeof(msg), "%u,%u,%u,%u", r, g, b, c);
  client.publish(topic, msg);
}

void lowerSensor() {
  /*            UP      DOWN
    MOTOR_IN1   HIGH    LOW
    MOTOR_IN2   LOW     HIGH
  */

  while (true) {
    VL53L0X_RangingMeasurementData_t measure;
    vl53.rangingTest(&measure, false);
    if (measure.RangeStatus == 0 && measure.RangeMilliMeter < DISTANCE_THRESHOLD_MM) {
      digitalWrite(MOTOR_IN1, HIGH);
      digitalWrite(MOTOR_IN2, LOW);
    } else if (measure.RangeStatus == 0 && measure.RangeMilliMeter > DISTANCE_THRESHOLD_MM){
      digitalWrite(MOTOR_IN1, LOW);
      digitalWrite(MOTOR_IN2, HIGH);
    } else {
      break;
    }
    delay(50);
  }

  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

void onMessage(char* topic, byte* payload, unsigned int length) {
  String cmd;
  for (unsigned int i = 0; i < length; i++) cmd += (char)payload[i];

  if (cmd == "analyze") {
    lowerSensor();
    delay(200); // стабилизация
    sendColor();
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);

  setup_wifi();
  client.setServer(MQTT_SERVER, 1883);
  client.setCallback(onMessage);
  
  vl53.begin(0x30, false);
  delay(500);
  tcs.begin(0x29);
  delay(500);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  sendVL53();
  delay(1000);
}
