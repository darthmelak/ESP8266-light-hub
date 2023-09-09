#include <Arduino.h>
#include <WifiConfig.hpp>
#include <Wire.h>
#include <SHT2x.h>
#include <arduino-timer.h>
#include <OneButton.h>
#include <HAswitchHelper.hpp>
#include <HAlightHelper.hpp>
#include <HAfanHelper.hpp>
#include <SerialHandler.hpp>
#include "secrets.h"

#define LEFT_LIGHT D6
#define LEFT_BUTTON D4
#define RIGHT_LIGHT D7
#define RIGHT_BUTTON D3
#define RELAY_PIN D0
#define PIR_PIN D5
#define FAN_PIN D8
#define FAN_BASE 210
#define FAN_STEPS 8

bool debug = false;
SHT2x sensor;
Timer<1> timer;
WifiConfig wifiConfig(WIFI_SSID, WIFI_PASSWORD, "ESP8266 Galeria-Hub", "galeria-hub", AUTH_USER, AUTH_PASS, true, true, debug);
OneButton rightButton(RIGHT_BUTTON);
OneButton leftButton(LEFT_BUTTON);
HAswitchHelper relay(wifiConfig, "relay", RELAY_PIN, false, debug);
HAlightHelper rightLight(wifiConfig, "rightLight", RIGHT_LIGHT, 10, 0, 0, true, debug);
HAlightHelper leftLight(wifiConfig, "leftLight", LEFT_LIGHT, 10, 0, 0, true, debug);
HAfanHelper fan(wifiConfig, "fan", FAN_PIN, FAN_STEPS, FAN_BASE, 0, false, debug);
bool hasSensor = false;
IntConfig PIR("PIR", 0);

void setupPeripherals();
void readSensor();
void lightButtonCb(HAswitchHelper&);
void serialCb(String);

void setup() {
  if (debug) {
    Serial.begin(115200);
    delay(10);
  }

  setupPeripherals();

  PIR.setCb([](int value) {
    wifiConfig.publish("binary_sensor/{sensorId}_PIR/state", value ? "ON" : "OFF", true);
  });
  leftButton.attachClick([]() {
    lightButtonCb(leftLight);
  });
  rightButton.attachClick([]() {
    lightButtonCb(rightLight);
  });

  // read sensor every minute
  if (hasSensor) {
    timer.every(60000, [](void*) -> bool { readSensor(); return true; });
  }

  wifiConfig.beginMQTT(
    MQTT_SERVER,
    1883,
    MQTT_USER,
    MQTT_PASS,
    "homeassistant/",
    MQTTConnectProps([]() {
      relay.onMqttConnect();
      leftLight.onMqttConnect();
      rightLight.onMqttConnect();
      fan.onMqttConnect();
      wifiConfig.publish("binary_sensor/{sensorId}_PIR/config", wifiConfig.binarySensorConfigPayload("PIR", "motion"), true);
      if (hasSensor) {
        wifiConfig.publish("sensor/{sensorId}_temperature/config", wifiConfig.sensorConfigPayload("temperature", "temperature", "°C"), true);
        wifiConfig.publish("sensor/{sensorId}_humidity/config", wifiConfig.sensorConfigPayload("humidity", "humidity", "%"), true);
        readSensor();
      }
    }, [](const String& topic, const String& data) {
      relay.onMqttMessage(topic, data);
      leftLight.onMqttMessage(topic, data);
      rightLight.onMqttMessage(topic, data);
      fan.onMqttMessage(topic, data);
    })
  );
  relay.begin();
  leftLight.begin();
  rightLight.begin();
  fan.begin();
}

void loop() {
  wifiConfig.loop();
  handleSerial(debug, serialCb);
  PIR.setValue(digitalRead(PIR_PIN));
  leftButton.tick();
  rightButton.tick();
  timer.tick();
  fan.tick();
  delay(1);
}

void setupPeripherals() {
  analogWriteFreq(20000);
  pinMode(A0, INPUT);
  pinMode(PIR_PIN, INPUT);

  Wire.begin();
  hasSensor = sensor.begin(&Wire);
  if (debug) {
    uint8_t stat = sensor.getStatus();
    Serial.print("Sensor status: ");
    Serial.print(stat, HEX);
    Serial.println();
  }

  randomSeed(analogRead(A0));
}

void readSensor() {
  if (debug) Serial.println("Reading sensor");

  sensor.read();
  float temp = sensor.getTemperature();
  float humi = sensor.getHumidity();
  wifiConfig.publish("sensor/{sensorId}_temperature/state", String(temp), true);
  wifiConfig.publish("sensor/{sensorId}_humidity/state", String(humi), true);
}

void lightButtonCb(HAswitchHelper& light) {
  IntConfig *state = light.getConfig().getInt("state");
  IntConfig *level = light.getConfig().getInt("level");
  if (state->getIntVal()) {
    if (level->getIntVal() < 255) {
      level->setValue(255);
    } else {
      state->setValue(0);
    }
  } else {
    level->setValue(255);
    state->setValue(1);
  }
}

void serialCb(String cmd) {
  if (cmd.equals("reset")) {
    SavedConfiguration wc = wifiConfig.getConfig();
    wc.get("ssid")->setValue(WIFI_SSID);
    wc.get("password")->setValue(WIFI_PASSWORD);
    wc.get("auth_user")->setValue(AUTH_USER);
    wc.get("auth_pass")->setValue(AUTH_PASS);
    Serial.println("Resetting wifi settings");
  }
}
