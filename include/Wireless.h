#include <Arduino.h>
#include <stdint.h>

#include <PubSubClient.h>

// declare globals
extern PubSubClient m_MqttClient;

// declare functions
void setupWifi();
void setupMqtt();

void handleWiFi();
void handleMqtt();

void connectWiFi();

void sendMeasurement(uint8_t port);
void debugWiFi();