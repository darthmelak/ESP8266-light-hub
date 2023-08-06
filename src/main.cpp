#include <Arduino.h>

#include <WifiConfig.hpp>
#include <Wire.h>
#include <SHT2x.h>
#include <arduino-timer.h>
#include <OneButton.h>
#include <SerialHandler.hpp>
#include "secrets.h"

#define LEFT_LIGHT D6
#define LEFT_BUTTON D4
#define RIGHT_LIGHT D7
#define RIGHT_BUTTON D3
#define RELAY_PIN D0
#define PIR_PIN D5
#define FAN_PIN D8
#define FAN_BASE 192
#define FAN_STEPS 8

bool debug = false;
TwoWire wire;
SHT2x sensor;
Timer<1> timer;
Configuration outputs("/outputs", debug);
Configuration inputs("/inputs", debug);
WifiConfig wifiConfig(WIFI_SSID, WIFI_PASSWORD, "ESP8266 Galeria-Hub", "galeria-hub", true, true, debug);

IntConfig *leftLight;
IntConfig *rightLight;
IntConfig *PIR;
OneButton leftButton(LEFT_BUTTON);
OneButton rightButton(RIGHT_BUTTON);
int fanSpd = 0;
bool hasSensor = false;

void serialCb(String);

void setup() {
  if (debug) {
    Serial.begin(115200);
    delay(10);
  }

  analogWriteFreq(20000);
  pinMode(PIR_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LEFT_LIGHT, OUTPUT);
  pinMode(RIGHT_LIGHT, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  wire.begin();
  hasSensor = sensor.begin(&wire);
  if (debug) {
    uint8_t stat = sensor.getStatus();
    Serial.print("Sensor status: ");
    Serial.print(stat, HEX);
    Serial.println();
  }

  inputs
    .add("PIR", 0, [](int value) {
      String state = value ? "ON" : "OFF";
      wifiConfig.publish("binary_sensor/{sensorId}_PIR/state", state, true);
    });
  PIR = inputs.getInt("PIR");

  outputs
    .add("relay", LOW, [](int value) {
      digitalWrite(RELAY_PIN, value);
      String state = value ? "ON" : "OFF";
      wifiConfig.publish("switch/{sensorId}_relay/state", state, true);
    })
    .add("leftLight", LOW, [](int value) {
      digitalWrite(LEFT_LIGHT, value);
      String state = value ? "ON" : "OFF";
      wifiConfig.publish("switch/{sensorId}_leftLight/state", state, true);
    })
    .add("rightLight", LOW, [](int value) {
      digitalWrite(RIGHT_LIGHT, value);
      String state = value ? "ON" : "OFF";
      wifiConfig.publish("switch/{sensorId}_rightLight/state", state, true);
    })
    .add("fanState", LOW, [](int value) {
      if (value == 0) {
        fanSpd = 0;
        analogWrite(FAN_PIN, 0);
      } else {
        fanSpd = outputs.getInt("fanSpeed")->getIntVal();
        analogWrite(FAN_PIN, FAN_BASE + fanSpd * FAN_STEPS);
      }
      String state = value ? "ON" : "OFF";
      wifiConfig.publish("fan/{sensorId}_fan/status/state", state, true);
    })
    .add("fanSpeed", 1, [](int value) {
      if (fanSpd != 0) {
        fanSpd = value;
        analogWrite(FAN_PIN, FAN_BASE + fanSpd * FAN_STEPS);
      }
      wifiConfig.publish("fan/{sensorId}_fan/status/speed", String(value), true);
    });

  leftLight = outputs.getInt("leftLight");
  rightLight = outputs.getInt("rightLight");
  leftButton.attachClick([]() {
    leftLight->setValue(leftLight->getIntVal() == HIGH ? LOW : HIGH);
  });
  rightButton.attachClick([]() {
    rightLight->setValue(rightLight->getIntVal() == HIGH ? LOW : HIGH);
  });

  // read sensor every minute
  timer.every(60000, [](void*) -> bool {
    if (!hasSensor) return false;
    if (debug) Serial.println("Reading sensor");

    sensor.read();
    float temp = sensor.getTemperature();
    float hum = sensor.getHumidity();
    wifiConfig.publish("sensor/{sensorId}_temperature/state", String(temp), true);
    wifiConfig.publish("sensor/{sensorId}_humidity/state", String(hum), true);

    return true;
  });

  wifiConfig.registerConfigApi(outputs);

  wifiConfig.setupMQTT(
    MQTT_SERVER,
    1883,
    MQTT_USER,
    MQTT_PASS,
    "homeassistant/",
    MQTTConnectProps([]() {
      wifiConfig.publish("binary_sensor/{sensorId}_PIR/config", wifiConfig.binarySensorConfigPayload("PIR", "motion"), true);

      wifiConfig.publish("switch/{sensorId}_relay/config", wifiConfig.switchConfigPayload("relay"), true);
      wifiConfig.subscribe("switch/{sensorId}_relay/cmd");
      wifiConfig.publish("switch/{sensorId}_relay/state", "OFF", true);

      wifiConfig.publish("switch/{sensorId}_leftLight/config", wifiConfig.switchConfigPayload("leftLight"), true);
      wifiConfig.subscribe("switch/{sensorId}_leftLight/cmd");
      wifiConfig.publish("switch/{sensorId}_leftLight/state", "OFF", true);

      wifiConfig.publish("switch/{sensorId}_rightLight/config", wifiConfig.switchConfigPayload("rightLight"), true);
      wifiConfig.subscribe("switch/{sensorId}_rightLight/cmd");
      wifiConfig.publish("switch/{sensorId}_rightLight/state", "OFF", true);

      wifiConfig.publish("fan/{sensorId}_fan/config", wifiConfig.fanConfigPayload("fan", true, false), true);
      wifiConfig.subscribe("fan/{sensorId}_fan/cmd/state");
      wifiConfig.publish("fan/{sensorId}_fan/status/state", "OFF", true);
      wifiConfig.subscribe("fan/{sensorId}_fan/cmd/speed");
      wifiConfig.publish("fan/{sensorId}_fan/status/speed", "1", true);

      if (hasSensor) {
        wifiConfig.publish("sensor/{sensorId}_temperature/config", wifiConfig.sensorConfigPayload("temperature", "temperature", "°C"), true);
        wifiConfig.publish("sensor/{sensorId}_humidity/config", wifiConfig.sensorConfigPayload("humidity", "humidity", "%"), true);
      }
    }, [](String topic, String data) {
      if (topic == wifiConfig.getPrefixedTopic("switch/{sensorId}_relay/cmd")) {
        outputs.getInt("relay")->setValue(data == "ON" ? HIGH : LOW);
      } else if (topic == wifiConfig.getPrefixedTopic("switch/{sensorId}_leftLight/cmd")) {
        outputs.getInt("leftLight")->setValue(data == "ON" ? HIGH : LOW);
      } else if (topic == wifiConfig.getPrefixedTopic("switch/{sensorId}_rightLight/cmd")) {
        outputs.getInt("rightLight")->setValue(data == "ON" ? HIGH : LOW);
      } else if (topic == wifiConfig.getPrefixedTopic("fan/{sensorId}_fan/cmd/state")) {
        outputs.getInt("fanState")->setValue(data == "ON" ? HIGH : LOW);
      } else if (topic == wifiConfig.getPrefixedTopic("fan/{sensorId}_fan/cmd/speed")) {
        outputs.getInt("fanSpeed")->setValue(data.toInt());
      }
    })
  );
}

void loop() {
  wifiConfig.loop();
  handleSerial(debug, serialCb);
  PIR->setValue(digitalRead(PIR_PIN));
  leftButton.tick();
  rightButton.tick();
  timer.tick();
  delay(1);
}

void serialCb(String cmd) {
  if (cmd.equals("reset")) {
    SavedConfiguration wc = wifiConfig.getConfig();
    wc.get("ssid")->setValue(WIFI_SSID);
    wc.get("password")->setValue(WIFI_PASSWORD);
    Serial.println("Resetting wifi settings");
  }
}
