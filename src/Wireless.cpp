#include <Wireless.h>
#include <Lcd.h>
#include <Power.h>

// WiFi includes
#include <AltSoftSerial.h>
#include "../lib/WiFiEspAT/src/WiFi.h"
#include <WiFiSecret.h>

// MQTT includes
#include <PubSubClient.h>

// #define DEBUG // toggle Debug Output

// ######## WiFi / networking
bool m_NetClientIsConnected = false; // the client's connection status
AltSoftSerial m_WifiPort(8, 9);      // must be 8 and 9 (interrupt used in library)
WiFiClient m_NetClient;              // WiFi client

// ######## MQTT client
PubSubClient m_MqttClient(m_NetClient); // the WiFi client
unsigned long m_MqttLastReconnect = 0;  // last reconnect time of MQTT client

// ################# WiFi stuff #################

void handleWiFi()
{
  // WiFi must be read or link is blocked forever
  while (m_NetClient.available())
    m_NetClient.read();

  // if disconnected, stop the client
  if (m_NetClientIsConnected && !m_NetClient.connected())
  {
#ifdef DEBUG
    Serial.println(F("disconnecting from server."));
#endif
    m_NetClient.stop();
    m_NetClientIsConnected = false;
  }
}

void setupWifi()
{
  // setup WiFi
  m_WifiPort.begin(9600);

  WiFi.init(m_WifiPort);

  m_Lcd.clear();

  if (WiFi.status() == WL_NO_MODULE)
  {
    m_Lcd.print(F("No WiFi chip, stop."));
    while (true)
      ;
  }
  else
    m_Lcd.print(F("WiFi chip found."));
}

void connectWiFi()
{
  m_Lcd.clear();
  m_Lcd.print(F("Connecting: "));
  m_Lcd.print(m_Ssid);

  // Connect to network
  WiFi.begin(m_Ssid, m_Pass);

  m_Lcd.clear();
  m_Lcd.print(F("WiFi connected"));

  // print the received signal strength
  long rssi = WiFi.RSSI();
  m_Lcd.setCursor(0, 1);
  m_Lcd.print(F("Signal: "));
  m_Lcd.print(rssi);
  m_Lcd.print(F(" dBm"));
  Serial.print("RSSI: ");
  Serial.println(WiFi.RSSI());
  delay(2000);
}

void debugWiFi()
{
  if (m_NetClientIsConnected)
  {
    m_Lcd.print(F("Client running"));
    m_Lcd.setCursor(0, 1);
    m_Lcd.print(F("Can't debug WiFi"));
    return;
  }

  long rssi = WiFi.RSSI();
  m_Lcd.print(F("RSSI:"));
  m_Lcd.print(rssi);

  bool stat = WiFi.status();
  m_Lcd.print(F(" Stat:"));
  m_Lcd.print(stat);

  m_Lcd.setCursor(0, 1);
  IPAddress ip = WiFi.localIP();
  m_Lcd.print(ip);
}

// make a HTTP connection to a server
void sendHttpRequest(const char *server)
{
  Serial.println();

  if (m_NetClient.connected())
  {
    Serial.println(F("Can't send request, still connected"));
    return;
  }

  if (m_NetClient.connect(server, 80))
  {
    Serial.println(F("Connected..."));
    m_NetClientIsConnected = true;

    // send the HTTP request
    m_NetClient.println(F("GET / HTTP/1.1"));
    m_NetClient.print(F("Host: "));
    m_NetClient.println(server);
    m_NetClient.println(F("Connection: Close"));
    m_NetClient.println();
    m_NetClient.flush();
  }
  else
  {
    Serial.println(F("Connection failed"));
  }
}

/**
 * MQTT topics for autodiscovery by Home Assistant:
 *   disc/sensor/Stromdings_P16/config -> 
 *          {
 *              "dev_cla": "power",
 *              "stat_t": "disc/sensor/Stromdings_P16/state",
 *              "unit_of_meas": "W",
 *              "uniq_id": "Stromdings_P16",
 *              "name": "Stromdings P6",
 *              "dev": {
 *                  "ids": [ "Stromdings" ],
 *                  "name": "Stromdings"
 *              }
 *          }
 *   disc/sensor/Stromdings_P16/state -> <measured_watts>
 */
void setupMqtt()
{
  m_Lcd.clear();
  m_Lcd.print(F("MQTT starting..."));
  m_Lcd.setCursor(0, 1);

  m_MqttClient.setServer("homeassistant", 1883);
  m_Lcd.print(F("MQTT server set"));
}

void handleMqtt()
{
  if (m_MqttClient.connected() || (millis() - m_MqttLastReconnect) < 5000)
    return;

  m_MqttLastReconnect = millis();
  m_MqttClient.loop();

  m_Lcd.clear();
  m_Lcd.print(F("MQTT connecting..."));
  m_Lcd.setCursor(0, 1);

  if (m_MqttClient.connect("Stromdings", "mosquitto", "mosquitto"))
  {
    m_Lcd.clear();
    m_Lcd.print(F("MQTT connected"));
    m_Lcd.setCursor(0, 1);

    // 33+1 chars: disc/sensor/Stromdings_P16/config
    // 182+1 chars: {"dev_cla":"power","stat_t":"disc/sensor/Stromdings_P16/state","unit_of_meas":"W","uniq_id":"Stromdings_Pxx","name":"Stromdings Pxx","dev":{"ids":["Stromdings"],"name":"Stromdings"}}
    char portnum[5];
    char topic[35];
    char msg[185];

    for (uint8_t i = 1; i <= m_NumPorts; i++)
    {
      itoa(i, portnum, 10);

      m_Lcd.print(F("Startup pub P"));
      m_Lcd.print(portnum);
      m_Lcd.setCursor(0, 1);

      strcpy(topic, "disc/sensor/Stromdings_P");
      strcat(topic, portnum);
      strcat(topic, "/config");

      strcpy(msg, "{");
      strcat(msg, "\"dev_cla\":\"power\",");
      strcat(msg, "\"stat_t\":\"disc/sensor/Stromdings_P");
      strcat(msg, portnum);
      strcat(msg, "/state\",");
      strcat(msg, "\"unit_of_meas\":\"W\",");
      strcat(msg, "\"uniq_id\":\"Stromdings_P");
      strcat(msg, portnum);
      strcat(msg, "\",");
      strcat(msg, "\"name\":\"Stromdings P");
      strcat(msg, portnum);
      strcat(msg, "\",");
      strcat(msg, "\"dev\":{\"ids\":[\"Stromdings\"],\"name\":\"Stromdings\"}");
      strcat(msg, "}");

      if (!m_MqttClient.publish(topic, msg, true))
      {
        m_LastPageChange = millis(); // delay next page change
        m_Lcd.print(F("Startup pub err!"));
      }
    }

    m_LastPageChange = millis(); // delay next page change
    m_Lcd.print(F("Startup pub ok  "));
  }
  else
  {
    m_LastPageChange = millis(); // delay next page change
    m_Lcd.clear();
    m_Lcd.print(F("MQTT start error"));
    m_Lcd.setCursor(0, 1);
    m_Lcd.print(F("Code: "));
    m_Lcd.print(m_MqttClient.state());
  }
}

// transmit a power measurement via MQTT
void sendMeasurement(uint8_t port)
{
  if (!m_MqttClient.connected())
    return;

  char portnum[5];
  itoa(port + 1, portnum, 10);

  // 32+1 chars: disc/sensor/Stromdings_P16/state
  char topic[35];
  strcpy(topic, "disc/sensor/Stromdings_P");
  strcat(topic, portnum);
  strcat(topic, "/state");

  char msg[7];
  itoa(m_LastMeasurements[port], msg, 10);

  if (!m_MqttClient.publish(topic, msg, false))
  {
    m_LastPageChange = millis(); // delay next page change
    m_Lcd.clear();
    m_Lcd.print(F("MQTT pub error!"));
    m_Lcd.setCursor(0, 1);
    m_Lcd.print(F("Port: P"));
    m_Lcd.print(port + 1);
  }
}
