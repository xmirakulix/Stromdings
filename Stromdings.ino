#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <WiFiEsp.h>
#include <SPI.h>

// #define DEBUG                                   // toggle Debug Output

// ######## Power measurement
const int m_MeasureInterval = 1000;             // Intervall der Messungen
int m_LastMeasureTime;                          // letzte Messung
const unsigned long m_MeasureDuration = 20000;  // 20k µs, 50Hz = 20ms pro Welle
const int m_CsPins[] = {10, 9, 8, 7};           // list of CS pins
const int m_CntCsPins = sizeof(m_CsPins) / sizeof(int); // Anzahl CS Pins
const int m_PsuCalibration = 228.20;             // calibration factor for used 9VAC PSU

// ######## LCD display
LiquidCrystal_I2C m_Lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
String m_LcdLine1, m_LcdLine2 = "";
bool m_LcdLineComplete = false;

// ######## WiFi / networking
SoftwareSerial m_WifiPort(2, 3); 
char m_Ssid[] = "Troubadix";            // your network SSID (name)
char m_Pass[] = "PsWlanKey";        // your network password
int m_NetStatus = WL_IDLE_STATUS;     // the Wifi radio's status
WiFiEspClient m_NetClient;

void setup()  
{
  // setup Serial for debugging
  Serial.begin(9600); //same speed as ESP chip to send commands between ESP and Serial Monitor
  while (!Serial) 
    delay(10);

  // setup WiFi
  m_WifiPort.begin(9600); // or 115200 if your ESP can only communicate at that speed
  while (!m_WifiPort) 
    delay(10);

  WiFi.init(&m_WifiPort);
  
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi chip not present, not continuing");
    while (true);
  }
  else
    Serial.println("WiFi chip found.");

  connectWiFi();
    
  // setup power measurement
  setupSPIandADCs(m_CsPins, m_CntCsPins);
  
  // setup LCD
  m_Lcd.begin(16, 2); 
  m_Lcd.home();
  m_Lcd.noBlink();

  sendHttpRequest("home.parnigoni.net", true);
}

void loop() 
{

/*  
  if (m_WifiPort.available()) 
  {
    char c = m_WifiPort.read();
    Serial.write(c);
  }
*/
  
  if (Serial.available())
  {
    char c = Serial.read();
    m_WifiPort.write(c);
  }

  if (m_NetClient.available()) 
  {
    Serial.write(m_NetClient.read());
  }
  

  if ((micros() - m_LastMeasureTime) < m_MeasureInterval)
  {
    // do power measurements
    float voltage;
    int power1, power2, power3;
  
    voltage = getAdcVoltage(10, 6, m_MeasureDuration, m_PsuCalibration);
    power1 = getAdcPower(9, 2, m_MeasureDuration, 100, 2000, voltage);
    power2 = getAdcPower(8, 4, m_MeasureDuration, 220, 2000, voltage);
    power3 = getAdcPower(7, 0, m_MeasureDuration, 220, 2000, voltage);
  
    Serial.print("V: ");
    Serial.print(voltage, 2);
    Serial.print(" W1: ");
    Serial.print(power1);
    Serial.print(" W2: ");
    Serial.print(power2);
    Serial.print(" W3: ");
    Serial.print(power3);
    Serial.println();
  }

}

// ################# WiFi stuff #################

void connectWiFi() 
{
  // attempt to connect to WiFi network
  while ( m_NetStatus != WL_CONNECTED) {
    Serial.print(F("Attempting to connect to WPA SSID: "));
    Serial.println(m_Ssid);
    
    // Connect to WPA/WPA2 network
    m_NetStatus = WiFi.begin(m_Ssid, m_Pass);
    
    Serial.print(F("connectWiFi(): Connection status: "));
    Serial.println(m_NetStatus);
  }

  // print the received signal strength
  long rssi = WiFi.RSSI();  
  Serial.print(F("You're connected to the network, signal strength: "));
  Serial.println(rssi); 
}

// this method makes a HTTP connection to the server
void sendHttpRequest(char server[], bool ssl)
{
  Serial.println();
    
  // close any connection before send a new request
  // this will free the socket on the WiFi shield
  m_NetClient.stop();

  // if there's a successful connection
  bool result;
  if (ssl)
    result = m_NetClient.connectSSL(server, 443);
  else
    result = m_NetClient.connect(server, 80);

  if (result)
  {
    Serial.println("Connected...");
    
    // send the HTTP PUT request
    m_NetClient.println(F("GET / HTTP/1.1"));
    m_NetClient.print(F("Host: "));
    m_NetClient.println(server);
    m_NetClient.println(F("Connection: close"));
    m_NetClient.println();

  }
  else 
  {
    // if you couldn't make a connection
    Serial.println(F("Connection failed"));
  }
}

/*
 * AT commands for ESP8266
 * Quelle: https://docs.espressif.com/projects/esp-at/en/latest/AT_Command_Set/TCP-IP_AT_Commands.html
 * ATE0/ATE1 Enable/Disable echo
 * AT+CWMODE=1 WiFi Mode=Station
 * AT+CWJAP="Troubadix","PsWlanKey" Connect to AP
 * AT+CWDHCP=1,0 Enable DHCP (en=0/1)
 * AT+CIPSTA=ip Set IP
 * AT+CIFSR Get IP
 * AT+CIPSTATUS Connection Status
 * AT+CIPDOMAIN="home.parnigoni.net" DNS resolve:
 * AT+PING="8.8.8.8" Ping:
 * AT+CIPSTART=id,TCP,addr,port Connection aufbauen
 * AT+RST Reset
 * AT+CWAUTOCONN=0 Disable autoconnect, automatic connection can create problems during initialization phase at next boot
 */

// ################# POWER stuff #################
   
 /*
 * setup the SPI bus and the CS pins
 *  csPins: array containing the cs pin numbers on the arduino
 * 
 * Return: none
 */
void setupSPIandADCs(const int csPins[], int countCsPins)
{
  SPI.begin();              // init the SPI bus
  for (int i = 0; i < countCsPins; i++)
  {
    pinMode(csPins[i], OUTPUT);    // set CS pin to output
    digitalWrite(csPins[i], HIGH); // pull CS pin high to disable the ADCs
  }
  delay(1);
}


/*
 * Get Reading from ADC and convert to Voltage
 *  csPin: Arduino chipselect pin for this ADC
 *  adcPin: which input pin to use on ADC (MCP3304: 0,2,4,6)
 *  measureDuration: measure interval in usec
 *  corrector: correction value specific to used AC/AC power supply
 *  
 * Return: Voltage in V
 */
float getAdcVoltage(int csPin, int adcPin, unsigned long measureDuration, float calibration)
{
  unsigned long lsbValue = 610351;      // 1 bit in nV (=5V/2^13)
  unsigned long voltage = 0;
  float primVoltage = 0;
  bool debug = false;                     // print debug output

  #ifdef DEBUG
    debug = true;
  #endif
  
  int adcVal = readAdcRms(csPin, adcPin, measureDuration);

  // strange order to maximize accuracy (max_val = 2^32)
  voltage = adcVal* lsbValue;           // reading in nV
  voltage /= 1000000;                   // conv to mV
  primVoltage = voltage * calibration;  // upscale to primary
  primVoltage /= 1000;                  // conv to V
  
  if (debug)
  {
    Serial.print("getAdcVoltage() Res: ");
    Serial.print(adcVal);
    Serial.print(" V: ");
    Serial.print(primVoltage);
    Serial.println();    
  }
  
  return primVoltage;
}

/*
 * Get Reading from ADC and convert to Power (W)
 *  csPin: Arduino chipselect pin for this ADC
 *  adcPin: which input pin to use on ADC (MCP3304: 0,2,4,6)
 *  measureDuration: measure interval in usec
 *  shuntOhms: size of measure shunt in Ohm
 *  turns: count of turns in current transformer (e.g. 2000 for 1:2000)
 *  voltage: voltage to use for power calculation
 *  
 * Return: Power in W 
 */
int getAdcPower(int csPin, int adcPin, unsigned long measureDuration, int shuntOhms, int turns, int voltage) 
{
  unsigned long power = 0;
  unsigned long lsbValue = 610351;      // 1 bit in nV (=5V/2^13)
  bool debug = false;                   // print debug output

  #ifdef DEBUG
    debug = true;
  #endif

  if (shuntOhms == 0)
  {
    if (debug)
      Serial.println(F("##### ERROR getAdcPower(): shuntOhms=0, div/0!"));
    return 0;
  }

  int adcVal = readAdcRms(csPin, adcPin, measureDuration);

  // strange order to maximize accuracy (max_val = 2^32)
                                  //                  approx. calc                        exact calc
  power = (adcVal * lsbValue);    // convert to nV;   46 * 610.351 = 28.076.146           46 * 610.351 = 28.076.146 
  power /= shuntOhms;             // conv to sek A    28.076.146 / 220 = 127.618,8454     28.076.146 / 220,8 = 127.156,4583333333
  power /= 1000;                  //                  127.618 / 1000 = 127                127.156,4583333333 / 1000 = 127,1564583333
  power *= turns;                 // conv to prim A   127 * 2000 = 254.000                127,1564583333 * 2000 = 254.312,9166666
  power /= 1000;                  //                  254.000 / 1000 = 254                254.312,9166666 / 1000 = 254,3129166666
  power *= voltage;               // conv to W(rms)   254 * 229 = 58.166                  254,3129166666 * 229,11 = 58.265,6323374847
  power /= 1000;                  //                  58.166 / 1000 = 58  <-- 99,5% -->   58.265,6323374847 / 1000 = 58,26
                                  //                                      absolute Error is between 0 and 3W
  
  if (debug)
  {
    Serial.print("getAdcPower() Res: ");
    Serial.print(adcVal);
    Serial.print(" Power: ");
    Serial.print(power);
    Serial.println();    
  }

  return int(power);              // Power in W
}


/*
 * Get Reading from ADC
 *  csPin: Arduino chipselect pin for this ADC
 *  adcPin: which input pin to use on ADC (MCP3304: 0,2,4,6)
 *  
 * Return: 13-bit value from ADC 
 */
int readAdc(int csPin, int adcPin)
{
  int adcValue = 0;                       // nimmt den Messwert vom ADC auf
  byte hi, lo, sign = 0;                  // results of ADC reads

  digitalWrite(csPin, LOW);             // turn on chipselect
  SPI.transfer(0x08 | (adcPin >> 1));   // send: 4x null, startbit, DIFF = 0, 2 Channelbits (D2, D1); return: uninteressant
  hi = SPI.transfer(adcPin << 7);       // send: lowest Channelbit (D0), rest dont care; return: 2 undef, nullbit, Signbit, 4 highest databits
  lo = SPI.transfer(0x00);              // send: dont care; return: 8 lowest databits
  digitalWrite(csPin, HIGH);            // turn off chipselect
  sign = hi & 0x10;                     // extract Sign Bit for DIFF
  adcValue = ((hi & 0x0F) << 8) + lo;   // combine the 2 return Values

  if (sign) 
    adcValue -= 4096;                   // if signbit is set, range is 0xFFF = -1 to 0x000 = -4096

  return adcValue;
}

/*
 * Get RMS value from ADC over given period (typically 20ms)
 *  csPin: Arduino chipselect pin for this ADC
 *  adcPin: which input pin to use on ADC (MCP3304: 0,2,4,6)
 *  measureDuration: measure interval in usec
 *  
 * Return: RMS value of reading series on given pin 
 */
int readAdcRms(int csPin, int adcPin, unsigned long measureDuration)
{
  unsigned long startTime = micros();
  int count = 0;                          // Anzahl der Messungen
  int adcValue = 0;                       // nimmt den Messwert vom ADC auf
  unsigned long adcRms = 0;               // nimmt die quadrierten Summen der gesamten Periode auf
  bool debug = false;                     // print debug output

  #ifdef DEBUG
    debug = true;
  #endif

  SPI.beginTransaction(SPISettings(1700000, MSBFIRST, SPI_MODE0)); // max 1,7MHz for MCP3304
  do 
  {
    adcValue = readAdc(csPin, adcPin);
    adcRms += (unsigned long)adcValue * (unsigned long)adcValue;  // calculate square
    count++;                                                      
  } 
  while ((micros() - startTime) < measureDuration);

  SPI.endTransaction();

  if (count == 0)
  {
    if (debug)
      Serial.println(F("##### ERROR readAdcRms(): count=0, div/0!"));
    return 0;
  }
  
  adcRms = sqrt(adcRms / count);          // -> quadratischer Mittelwert

  int ret = adcRms;
  if (ret < 4)
  {
    ret = 0;

    if (debug)
      Serial.println(F("####### ATTENTION, readAdcRms() return value set to 0 because low val read (probably noise)!"));
  }
  
  if (debug)
  {
    Serial.print("readAdcRms(");
    Serial.print(csPin);
    Serial.print(",");
    Serial.print(adcPin);
    Serial.print(") Cnt: ");
    Serial.print(count);
    Serial.print(" Res: ");
    Serial.print(adcRms);
    Serial.print(" Return: ");
    Serial.println(ret);
  }

  return ret;
}

// ################# LCD stuff #################

void writeLCD(char c)
{
  if (c > 31)
  {
    if (m_LcdLineComplete)
    {
      m_LcdLine1 = m_LcdLine2;
      m_LcdLine2 = "";
      m_LcdLineComplete = false;
      m_Lcd.clear();
      Serial.println(F("LCD cleared..."));
      m_Lcd.home();
      m_Lcd.print(m_LcdLine1);
      m_Lcd.setCursor(0,1); 
    }
    m_Lcd.print(c);
    m_LcdLine2 += c;
  }
  if (c == 10)
     m_LcdLineComplete = true;  
}
