#include "./src/AltSoftSerial/AltSoftSerial.h"
#include "./src/WiFiEspAT/src/WiFiEspAT.h"

#include <SPI.h>

#include <Wire.h>
#include "src/hd44780/hd44780.h"                       // main hd44780 header
#include "src/hd44780/hd44780ioClass/hd44780_I2Cexp.h" // i2c expander i/o class header

// #define DEBUG // toggle Debug Output

// ######## Power measurement
const int m_CsPins[] = {4, 5, 6, 7};                   // list of CS pins
const int m_CntCsPins = sizeof(m_CsPins) / sizeof(int); // Anzahl CS Pins
const int m_PsuCalibration = 228.20;                    // calibration factor for used 9VAC PSU
const unsigned long m_MeasureInterval = 1 * 1000;       // ms, Intervall der Messungen
unsigned long m_LastMeasureTime = 0;                    // letzte Messung
unsigned int m_curMeasurePort = 1;                      // zuletzt gemessener Pin
const unsigned long m_MeasureDuration = 20 * 1000;      // 20k µs, 50Hz = 20ms pro Welle

// ######## LCD display
hd44780_I2Cexp m_Lcd; // declare lcd object: auto locate & auto config expander chip

// ######## WiFi / networking
AltSoftSerial m_WifiPort(8,9);        // must be 8 and 9 (ISR in library)
char m_Ssid[] = "Troubadix";          // your network SSID (name)
char m_Pass[] = "PsWlanKey";          // your network password
int m_NetStatus = WL_IDLE_STATUS;     // the Wifi radio's status
WiFiClient m_NetClient;               // WiFi client
bool m_NetClientIsConnected = false;  // the client's connection status

// ######## rotary encoder
const int m_RotInput = 0; // Pin A0 is used to actually read the encoder values
int m_RotValue = 0;       // Variable to store the analog values read from the encoder
char m_RotResult = (' '); // contains the interpreted result

// ################# Main stuff #################

void setup()
{
  // setup Serial for debugging
  Serial.begin(9600); //same speed as ESP chip to send commands between ESP and Serial Monitor
  while (!Serial)
    delay(10);

  // setup and connect WiFi
  setupWifi();
  connectWiFi();

  // setup power measurement
  setupPower(m_CsPins, m_CntCsPins);

  // setup LCD
  setupLcd();

  // setup the rotary ecoder
  setupRotary();

  // const char *server = "home.parnigoni.net";
  // sendHttpRequest(server);
}

void loop()
{
  // handleWiFi();

  handlePower();

  handleEncoder();
}

// ################# WiFi stuff #################

void handleWiFi()
{
  while (m_NetClient.available())
  {
    char c = m_NetClient.read();
    Serial.write(c);
  }

  // if the server's disconnected, stop the client
  if (m_NetClientIsConnected && !m_NetClient.connected())
  {
    Serial.println();
    Serial.println("disconnecting from server.");
    m_NetClient.stop();
    m_NetClientIsConnected = false;
  }
}

void setupWifi()
{
  // setup WiFi
  m_WifiPort.begin(9600); // or 115200 if your ESP can only communicate at that speed

  WiFi.init(m_WifiPort);

  if (WiFi.status() == WL_NO_MODULE)
  {
    Serial.println("WiFi chip not present, not continuing");
    while (true)
      ;
  }
  else
    Serial.println("WiFi chip found.");
}

void connectWiFi()
{
  Serial.print(F("connectWiFi(): Attempting to connect to WPA SSID: "));
  Serial.print(m_Ssid);

  // Connect to network
  m_NetStatus = WiFi.begin(m_Ssid, m_Pass);
  while (m_NetStatus != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.print(F("connectWiFi(): Connected, status: "));
  Serial.println(m_NetStatus);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print(F("connectWiFi(): connected to the network, signal strength: "));
  Serial.print(rssi);
  Serial.println(" dBm");
}

// this method makes a HTTP connection to the server
void sendHttpRequest(const char *server)
{
  Serial.println();

  // close any connection before send a new request
  // this will free the socket on the WiFi shield
  m_NetClient.stop();
  m_NetClientIsConnected = false;

  // if there's a successful connection
  if (m_NetClient.connect(server, 80))
  {
    Serial.println("Connected...");
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

// ################# POWER stuff #################

void handlePower()
{
  // do power measurements
  if ((millis() - m_LastMeasureTime) > m_MeasureInterval)
  {
    m_LastMeasureTime = millis();

    // 0x1 (00|01) to 0xF (11|11) -> 4x chip | 4x pin
    int csPin = m_curMeasurePort >> 2; // extraxt chip
    int adcPin = m_curMeasurePort & 3; // extract pin

    float voltage;
    int power;

    Serial.print("Measuring: Chip: ");
    Serial.print(m_CsPins[csPin]);
    Serial.print(", Pin: ");
    Serial.print(adcPin);

    voltage = getAdcVoltage(m_CsPins[0], 0, m_MeasureDuration, m_PsuCalibration);
    power = getAdcPower(m_CsPins[csPin], adcPin, m_MeasureDuration, 220, 2000, voltage);

    Serial.print(" V: ");
    Serial.print(voltage, 2);
    Serial.print(" W1: ");
    Serial.print(power);
    Serial.println();

    m_curMeasurePort++;
    if (m_curMeasurePort & 16)
      m_curMeasurePort = 1; // start over at port 1 (port 0 = voltage)
  }
}

/*
 * setup the SPI bus and the CS pins
 *  csPins: array containing the cs pin numbers on the arduino
 * 
 * Return: none
 */
void setupPower(const int csPins[], int countCsPins)
{
  SPI.begin(); // init the SPI bus
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
  unsigned long lsbValue = 610351; // 1 bit in nV (=5V/2^13)
  unsigned long voltage = 0;
  float primVoltage = 0;

  int adcVal = readAdcRms(csPin, adcPin, measureDuration);

  // strange order to maximize accuracy (max_val = 2^32)
  voltage = adcVal * lsbValue;         // reading in nV
  voltage /= 1000000;                  // conv to mV
  primVoltage = voltage * calibration; // upscale to primary
  primVoltage /= 1000;                 // conv to V

#ifdef DEBUG
  Serial.print("getAdcVoltage() Res: ");
  Serial.print(adcVal);
  Serial.print(" V: ");
  Serial.print(primVoltage);
  Serial.println();
#endif

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
  unsigned long lsbValue = 610351; // 1 bit in nV (=5V/2^13)

  if (shuntOhms == 0)
  {
#ifdef DEBUG
    Serial.println(F("##### ERROR getAdcPower(): shuntOhms=0, div/0!"));
#endif
    return 0;
  }

  int adcVal = readAdcRms(csPin, adcPin, measureDuration);

  // strange order to maximize accuracy (max_val = 2^32)
  //                  approx. calc                        exact calc
  power = (adcVal * lsbValue); // convert to nV;   46 * 610.351 = 28.076.146           46 * 610.351 = 28.076.146
  power /= shuntOhms;          // conv to sek A    28.076.146 / 220 = 127.618,8454     28.076.146 / 220,8 = 127.156,4583333333
  power /= 1000;               //                  127.618 / 1000 = 127                127.156,4583333333 / 1000 = 127,1564583333
  power *= turns;              // conv to prim A   127 * 2000 = 254.000                127,1564583333 * 2000 = 254.312,9166666
  power /= 1000;               //                  254.000 / 1000 = 254                254.312,9166666 / 1000 = 254,3129166666
  power *= voltage;            // conv to W(rms)   254 * 229 = 58.166                  254,3129166666 * 229,11 = 58.265,6323374847
  power /= 1000;               //                  58.166 / 1000 = 58  <-- 99,5% -->   58.265,6323374847 / 1000 = 58,26
                               //                                      absolute Error is between 0 and 3W

#ifdef DEBUG
  Serial.print("getAdcPower() Res: ");
  Serial.print(adcVal);
  Serial.print(" Power: ");
  Serial.print(power);
  Serial.println();
#endif

  return int(power); // Power in W
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
  int adcValue = 0;      // nimmt den Messwert vom ADC auf
  byte hi, lo, sign = 0; // results of ADC reads

  adcPin *= 2;                        // 4 adcPins are 0, 2, 4, 6
  digitalWrite(csPin, LOW);           // turn on chipselect
  SPI.transfer(0x08 | (adcPin >> 1)); // send: 4x null, startbit, DIFF = 0, 2 Channelbits (D2, D1); return: uninteressant
  hi = SPI.transfer(adcPin << 7);     // send: lowest Channelbit (D0), rest dont care; return: 2 undef, nullbit, Signbit, 4 highest databits
  lo = SPI.transfer(0x00);            // send: dont care; return: 8 lowest databits
  digitalWrite(csPin, HIGH);          // turn off chipselect
  sign = hi & 0x10;                   // extract Sign Bit for DIFF
  adcValue = ((hi & 0x0F) << 8) + lo; // combine the 2 return Values

  if (sign)
    adcValue -= 4096; // if signbit is set, range is 0xFFF = -1 to 0x000 = -4096

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
  int count = 0;            // Anzahl der Messungen
  int adcValue = 0;         // nimmt den Messwert vom ADC auf
  unsigned long adcRms = 0; // nimmt die quadrierten Summen der gesamten Periode auf

  SPI.beginTransaction(SPISettings(1700000, MSBFIRST, SPI_MODE0)); // max 1,7MHz for MCP3304
  do
  {
    adcValue = readAdc(csPin, adcPin);
    adcRms += (unsigned long)adcValue * (unsigned long)adcValue; // calculate square
    count++;
  } while ((micros() - startTime) < measureDuration);

  SPI.endTransaction();

  if (count == 0)
  {
#ifdef DEBUG
    Serial.println(F("##### ERROR readAdcRms(): count=0, div/0!"));
#endif

    return 0;
  }

  adcRms = sqrt(adcRms / count); // -> quadratischer Mittelwert

  int ret = adcRms;
  if (ret < 4)
  {
    ret = 0;

#ifdef DEBUG
    Serial.println(F("####### ATTENTION, readAdcRms() return value set to 0 because low val read (probably noise)!"));
#endif
  }

#ifdef DEBUG
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
#endif

  return ret;
}

// ################# LCD stuff #################

void setupLcd()
{
  int status = m_Lcd.begin(16, 2);
  if (status) // non zero status means it was unsuccesful
  {
    // hd44780 has a fatalError() routine that blinks an led if possible
    // begin() failed so blink error code using the onboard LED if possible
    Serial.println(F("setupLcd(): Failed to initialize LCD"));
    hd44780::fatalError(status); // does not return
  }

#ifdef DEBUG
    Serial.print(F("setupLcd(): LCD initialized...")
#endif

  m_Lcd.lineWrap();
  m_Lcd.clear();
  m_Lcd.print("Very very very very long text");
  m_Lcd.noCursor();
}


// ################# rotary encoder stuff #################

void setupRotary()
{

  pinMode(m_RotInput, INPUT); // Define A0 as Analog input

  unsigned int intr_state = SREG;
  cli();
  PCICR |= (1 << PCIE1);   // PCIE1: Pin Change Interrupt Enable group 1
  PCMSK1 |= (1 << PCINT8); // Enable Pin Change Interrupt for A0
	SREG = intr_state;
}


// show_encoder() Subroutine to read the Encoder values
void handleEncoder()
{
  if (m_RotResult != ' ')
  {
    Serial.println(m_RotResult);
    m_RotResult = ' ';
  }
}

// Interrupt Service Routine to read the Encoder values
// Using an interrupt routine to avoid slowing down the main program, since no polling is neccesary
ISR(PCINT1_vect)
{
  m_RotResult = (' ');

  // read average value
  m_RotValue = (analogRead(m_RotInput) + analogRead(m_RotInput) + analogRead(m_RotInput) + analogRead(m_RotInput)) / 4;

  // gaps between value-windows to avoid mis-reading
  if (m_RotValue > 590 && m_RotValue < 630)
    m_RotResult = ('B'); // press button
  if (m_RotValue > 670 && m_RotValue < 710)
    m_RotResult = ('R'); // turn right
  if (m_RotValue > 780 && m_RotValue < 820)
    m_RotResult = ('L'); // turn left
}
