#include <Arduino.h>
#include <Wireless.h>
#include <Lcd.h>
#include <Power.h>
#include <Rotary.h>

// ################# Main stuff #################

void setup()
{
  // setup Serial for debugging
  Serial.begin(9600);
  while (!Serial)
    delay(10);

  // setup LCD
  setupLcd();

  // setup and connect WiFi
  setupWifi();
  connectWiFi();

  setupMqtt();

  // setup power measurement
  setupPower();

  // setup the rotary ecoder
  setupRotary();
}

void loop()
{
  handleWiFi();
  handleMqtt();
  handlePower();
  handleLcd();
}