#include <Arduino.h>
#include <WiFi.h>
#include "Update.h"
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include <secret.h>

#define pinSensor 36
#define pinPump 2

uint32_t sensorValue;
const uint32_t sensorThreshold = 2000;
constexpr char THINGSBOARD_SERVER[] = "thingsboard.ucll.cloud";
constexpr uint32_t THINGSBOARD_PORT = 31364U;
constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 9600U;
constexpr uint32_t telemetrySendInterval = 2000U;
uint32_t previousDataSend = 0;
WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

void InitWiFi() {
    Serial.println("Connecting to WiFi ...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected to WiFi");
}

bool reconnect() {
    if (WiFi.status() != WL_CONNECTED) {
        InitWiFi();
    }
    return WiFi.status() == WL_CONNECTED;
}

RPC_Response giveWater(RPC_Data &data) {
    Serial.println("Received give water function");
    digitalWrite(pinPump, HIGH);
    delay(1000);
    digitalWrite(pinPump, LOW);
    return RPC_Response("giveWater", "success");
}

const std::array<RPC_Callback, 1U> cb = {
    RPC_Callback("giveWater", giveWater),
};

void setup() {
    Serial.begin(SERIAL_DEBUG_BAUD);
    Serial.println();

    pinMode(pinSensor, INPUT);
    pinMode(pinPump, OUTPUT);

    InitWiFi();
}

void loop() {
    if (!reconnect()) {
        return;
    }

    if (!tb.connected()) {
        Serial.print("Connecting to: ");
        Serial.print(THINGSBOARD_SERVER);
        Serial.print(" with token ");
        Serial.println(TOKEN);
        if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
            Serial.println("Failed to connect");
            return;
        } else {
            if (!tb.RPC_Subscribe(cb.cbegin(), cb.cend())) {
                Serial.println("Failed to subscribe RPC");
                return;
            } else {
                Serial.println("RPC Subscribed successfully");
            }
        }
        tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());
    }

    sensorValue = analogRead(pinSensor);

    if (millis() - previousDataSend > telemetrySendInterval) {
        previousDataSend = millis();
        tb.sendTelemetryData("sensorValue", sensorValue);
        tb.sendAttributeData("rssi", WiFi.RSSI());
        tb.sendAttributeData("channel", WiFi.channel());
        tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
        tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
        tb.sendAttributeData("ssid", WiFi.SSID().c_str());
    }

    tb.loop();
    delay(100);
}
