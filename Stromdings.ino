// WiFi includes
#include "./src/AltSoftSerial/AltSoftSerial.h"
#include "./src/WiFiEspAT/src/WiFiEspAT.h"

// power includes
#include <SPI.h>

// LCD includes
#include <Wire.h>
#include "src/hd44780/hd44780.h"                        // main hd44780 header
#include "src/hd44780/hd44780ioClass/hd44780_I2Cexp.h"  // i2c expander i/o class header

// #define DEBUG // toggle Debug Output
#ifdef DEBUG
unsigned long m_PowerHandlerDuration = 0;  // store the duation of last handler run
unsigned long m_LcdHandlerDuration = 0;    // store the duation of last handler run
#endif

// ######## Power measurement
const int m_CsPins[] = {4, 5, 6, 7};                  // list of ADC Chip-Select pins
const int m_AdcCnt = sizeof(m_CsPins) / sizeof(int);  // number of ADCs
const int m_PsuCalibration = 228.20;                  // calibration factor for used 9VAC PSU
const unsigned long m_MeasureInterval = 1 * 1000;     // ms, interval between measurements
unsigned long m_LastMeasureTime = 0;                  // time of last measurement
unsigned int m_curMeasurePort = 1;                    // zuletzt gemessener Pin
const unsigned long m_MeasureDuration = 20 * 1000;    // 20k µs, 50Hz = 20ms pro Welle
int m_LastMeasurements[(m_AdcCnt * 4) - 1];           // last measurement per port, 4*num of ADCs minus voltage
float m_LastVoltage;                                  // last measured voltage

// ######## LCD display
hd44780_I2Cexp m_Lcd;                 // declare lcd object: auto locate & auto config expander chip
unsigned int m_LcdCurrentPage = 0;    // which page to display
bool m_LcdShowInfoPage = false;       // should we display the info page?
unsigned long m_LastPageChange = 0;   // time of last page change
unsigned long m_LastBacklightOn = 0;  // time of last backlight on
bool m_IsLcdBacklight = true;         // is the backlight on?

// ######## WiFi / networking
AltSoftSerial m_WifiPort(8, 9);       // must be 8 and 9 (ISR in library)
char m_Ssid[] = "Troubadix";          // your network SSID (name)
char m_Pass[] = "PsWlanKey";          // your network password
int m_NetStatus = WL_IDLE_STATUS;     // the Wifi radio's status
WiFiClient m_NetClient;               // WiFi client
bool m_NetClientIsConnected = false;  // the client's connection status

// ######## rotary encoder
const int m_RotInput = 0;  // Pin A0 is used
int m_RotValue = 0;        // analog values read from the encoder
char m_RotResult = (' ');  // contains the interpreted result

// ################# Main stuff #################

void setup()
{
  // setup Serial for debugging
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  // setup LCD
  setupLcd();

  // setup and connect WiFi
  setupWifi();
  connectWiFi();

  // setup power measurement
  setupPower();

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
  handleLcd();
}

// ################# WiFi stuff #################

void handleWiFi()
{
  while (m_NetClient.available())
  {
    char c = m_NetClient.read();
    Serial.write(c);
  }

  // if disconnected, stop the client
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
  m_WifiPort.begin(9600);

  WiFi.init(m_WifiPort);

  m_Lcd.clear();

  if (WiFi.status() == WL_NO_MODULE)
  {
    m_Lcd.print("No WiFi chip, not continuing");
    while (true)
      ;
  }
  else
    m_Lcd.print("WiFi chip found.");
}

void connectWiFi()
{
  m_Lcd.clear();
  m_Lcd.print("Connecting: ");
  m_Lcd.print(m_Ssid);

  // Connect to network
  m_NetStatus = WiFi.begin(m_Ssid, m_Pass);
  while (m_NetStatus != WL_CONNECTED)
    ;

  m_Lcd.clear();
  m_Lcd.print("WiFi connected");

  // print the received signal strength
  long rssi = WiFi.RSSI();
  m_Lcd.setCursor(0, 1);
  m_Lcd.print("Signal: ");
  m_Lcd.print(rssi);
  m_Lcd.print(" dBm");

  delay(2000);
}

// this method makes a HTTP connection to the server
void sendHttpRequest(const char* server)
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
#ifdef DEBUG
  m_PowerHandlerDuration = millis();
#endif

  // do power measurements
  if ((millis() - m_LastMeasureTime) > m_MeasureInterval)
  {
    m_LastMeasureTime = millis();

    // 0x1 (00|01) to 0xF (11|11) -> 4x chip | 4x pin
    int csPin = m_curMeasurePort >> 2;  // extraxt chip
    int adcPin = m_curMeasurePort & 3;  // extract pin

#ifdef DEBUG
    Serial.print("Measuring: Chip: ");
    Serial.print(m_CsPins[csPin]);
    Serial.print(", Pin: ");
    Serial.print(adcPin);
#endif

    m_LastVoltage = getAdcVoltage(m_CsPins[0], 0, m_MeasureDuration, m_PsuCalibration);
    m_LastMeasurements[m_curMeasurePort - 1] = getAdcPower(m_CsPins[csPin], adcPin, m_MeasureDuration, 220, 2000, m_LastVoltage);

#ifdef DEBUG
    Serial.print(" V: ");
    Serial.print(voltage, 2);
    Serial.print(" W1: ");
    Serial.print(power);
    Serial.println();
#endif

    m_curMeasurePort++;
    if (m_curMeasurePort & (m_AdcCnt * 4))
      m_curMeasurePort = 1;  // start over at port 1 (port 0 = voltage)
  }

#ifdef DEBUG
  Serial.print("Power handler took: ");
  Serial.print(millis() - m_PowerHandlerDuration);
  Serial.println("ms");
#endif
}

/*
 * setup the SPI bus and the CS pins
 *	csPins: array containing the cs pin numbers on the arduino
 *
 * Return: none
 */
void setupPower()
{
  SPI.begin();  // init the SPI bus
  for (int i = 0; i < m_AdcCnt; i++)
  {
    pinMode(m_CsPins[i], OUTPUT);     // set CS pin to output
    digitalWrite(m_CsPins[i], HIGH);  // pull CS pin high to disable the ADCs
  }
  delay(1);
}

/*
 * Get Reading from ADC and convert to Voltage
 *	csPin: Arduino chipselect pin for this ADC
 *	adcPin: which input pin to use on ADC (MCP3304: 0,2,4,6)
 *	measureDuration: measure interval in usec
 *	corrector: correction value specific to used AC/AC power supply
 *
 * Return: Voltage in V
 */
float getAdcVoltage(int csPin, int adcPin, unsigned long measureDuration, float calibration)
{
  unsigned long lsbValue = 610351;  // 1 bit in nV (=5V/2^13)
  unsigned long voltage = 0;
  float primVoltage = 0;

  int adcVal = readAdcRms(csPin, adcPin, measureDuration);

  // strange order to maximize accuracy (max_val = 2^32)
  voltage = adcVal * lsbValue;          // reading in nV
  voltage /= 1000000;                   // conv to mV
  primVoltage = voltage * calibration;  // upscale to primary
  primVoltage /= 1000;                  // conv to V

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
 *	csPin: Arduino chipselect pin for this ADC
 *	adcPin: which input pin to use on ADC (MCP3304: 0,2,4,6)
 *	measureDuration: measure interval in usec
 *	shuntOhms: size of measure shunt in Ohm
 *	turns: count of turns in current transformer (e.g. 2000 for 1:2000)
 *	voltage: voltage to use for power calculation
 *
 * Return: Power in W
 */
int getAdcPower(int csPin, int adcPin, unsigned long measureDuration, int shuntOhms, int turns, int voltage)
{
  unsigned long power = 0;
  unsigned long lsbValue = 610351;  // 1 bit in nV (=5V/2^13)

  if (shuntOhms == 0)
  {
#ifdef DEBUG
    Serial.println(F("##### ERROR getAdcPower(): shuntOhms=0, div/0!"));
#endif
    return 0;
  }

  int adcVal = readAdcRms(csPin, adcPin, measureDuration);

  // strange order to maximize accuracy (max_val = 2^32)
  power = (adcVal * lsbValue);  // convert to nV;	46 * 610.351 = 28.076.146
  power /= shuntOhms;           // conv to sek A	28.076.146 / 220 = 127.618,8454
  power /= 1000;                //					127.618 / 1000 = 127
  power *= turns;               // conv to prim A	127 * 2000 = 254.000
  power /= 1000;                //					254.000 / 1000 = 254
  power *= voltage;             // conv to W(rms)	254 * 229 = 58.166
  power /= 1000;                //					58.166 / 1000 = 58

#ifdef DEBUG
  Serial.print("getAdcPower() Res: ");
  Serial.print(adcVal);
  Serial.print(" Power: ");
  Serial.print(power);
  Serial.println();
#endif

  return int(power);  // Power in W
}

/*
 * Get Reading from ADC
 *	csPin: Arduino chipselect pin for this ADC
 *	adcPin: which input pin to use on ADC (MCP3304: 0,2,4,6)
 *
 * Return: 13-bit value from ADC
 */
int readAdc(int csPin, int adcPin)
{
  int adcValue = 0;       // nimmt den Messwert vom ADC auf
  byte hi, lo, sign = 0;  // results of ADC reads

  adcPin *= 2;                         // 4 adcPins are 0, 2, 4, 6
  digitalWrite(csPin, LOW);            // turn on chipselect
  SPI.transfer(0x08 | (adcPin >> 1));  // send: 4x null, startbit, DIFF = 0, 2 Channelbits (D2, D1); return: uninteressant
  hi = SPI.transfer(adcPin << 7);      // send: lowest Channelbit (D0), rest dont care; return: 2 undef, nullbit, Signbit, 4 highest databits
  lo = SPI.transfer(0x00);             // send: dont care; return: 8 lowest databits
  digitalWrite(csPin, HIGH);           // turn off chipselect
  sign = hi & 0x10;                    // extract Sign Bit for DIFF
  adcValue = ((hi & 0x0F) << 8) + lo;  // combine the 2 return Values

  if (sign)
    adcValue -= 4096;  // if signbit is set, range is 0xFFF = -1 to 0x000 = -4096

  return adcValue;
}

/*
 * Get RMS value from ADC over given period (typically 20ms)
 *	csPin: Arduino chipselect pin for this ADC
 *	adcPin: which input pin to use on ADC (MCP3304: 0,2,4,6)
 *	measureDuration: measure interval in usec
 *
 * Return: RMS value of reading series on given pin
 */
int readAdcRms(int csPin, int adcPin, unsigned long measureDuration)
{
  unsigned long startTime = micros();
  int count = 0;     // Anzahl der Messungen
  int adcValue = 0;  // nimmt den Messwert vom ADC auf
  unsigned long adcRms =
      0;  // nimmt die quadrierten Summen der gesamten Periode auf

  SPI.beginTransaction(
      SPISettings(1700000, MSBFIRST, SPI_MODE0));  // max 1,7MHz for MCP3304
  do
  {
    adcValue = readAdc(csPin, adcPin);
    adcRms +=
        (unsigned long)adcValue * (unsigned long)adcValue;  // calculate square
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

  adcRms = sqrt(adcRms / count);  // -> quadratischer Mittelwert

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
  if (status)  // non zero status means it was unsuccesful
  {
    // hd44780 has a fatalError() routine that blinks an led if possible
    // begin() failed so blink error code using the onboard LED if possible
    Serial.println(F("setupLcd(): Failed to initialize LCD"));
    hd44780::fatalError(status);  // does not return
  }

#ifdef DEBUG
    Serial.print(F("setupLcd(): LCD initialized...");
#endif

	m_Lcd.lineWrap();
	m_Lcd.clear();
	m_Lcd.noCursor();
}

void handleLcd()
{
#ifdef DEBUG
  m_LcdHandlerDuration = millis();
#endif

  if (m_RotResult == 'L')  // rotary turned back
  {
    m_LcdCurrentPage = (m_LcdCurrentPage - 2) % m_AdcCnt;  // LCD page back
    m_LastPageChange = 0;                                  // change page immediately
    m_RotResult = ' ';
    enableLcdBacklight();
  }

  if (m_RotResult == 'R')  // rotary turned forward
  {
    m_LastPageChange = 0;  // render next page immediately
    m_RotResult = ' ';
    enableLcdBacklight();
  }

  if (m_RotResult == 'B')  // button pressed
  {
    displayDiagInfo();
    enableLcdBacklight();
  }

  if (m_IsLcdBacklight && (millis() - m_LastBacklightOn) > 10 * 1000)
  {
    disableLcdBacklight();
  }

  if ((millis() - m_LastPageChange) > 5 * 1000)
  {
    m_LcdCurrentPage = (m_LcdCurrentPage + 1) % m_AdcCnt;  // next page
    int position = 0;                                      // 4 positions per page (top left, top right, ...)

    m_Lcd.clear();

    if (m_LcdCurrentPage == 0)
    {
      m_Lcd.print("In:");
      m_Lcd.print((int)m_LastVoltage, DEC);
      m_Lcd.print("V");
      position++;
    }

    for (int i = position; i < 4; i++)
    {
      m_Lcd.setCursor((i << 3) & 8, (i >> 1) & 1);  // col 0+8, row 0+1
      m_Lcd.print("P");
      m_Lcd.print((m_LcdCurrentPage << 2) + i, DEC);
      m_Lcd.print(":");
      m_Lcd.print(m_LastMeasurements[(m_LcdCurrentPage << 2) + i - 1], DEC);
      m_Lcd.print("W");
    }

    m_LastPageChange = millis();
  }
#ifdef DEBUG
  Serial.print("Lcd handler took: ");
  Serial.print(millis() - m_LcdHandlerDuration);
  Serial.println("ms");
#endif
}

void enableLcdBacklight()
{
  m_LastBacklightOn = millis();
  m_Lcd.backlight();
  m_IsLcdBacklight = true;
}

void disableLcdBacklight()
{
  m_Lcd.noBacklight();
  m_IsLcdBacklight = false;
}

void displayDiagInfo()
{
  m_Lcd.clear();
}

// ################# rotary encoder stuff #################

void setupRotary()
{
  pinMode(m_RotInput, INPUT);  // Define A0 as Analog input

  unsigned int intr_state = SREG;
  cli();
  PCICR |= (1 << PCIE1);    // PCIE1: Pin Change Interrupt Enable group 1
  PCMSK1 |= (1 << PCINT8);  // Enable Pin Change Interrupt for A0
  SREG = intr_state;
}

// show_encoder() Subroutine to read the Encoder values
void handleEncoder()
{
}

// Interrupt Service Routine to read the Encoder values
// Using an interrupt routine to avoid slowing down the main program, since no
// polling is neccesary
ISR(PCINT1_vect)
{
  m_RotResult = (' ');

  // read average value
  m_RotValue = (analogRead(m_RotInput) + analogRead(m_RotInput) + analogRead(m_RotInput) + analogRead(m_RotInput)) / 4;

  // gaps between value-windows to avoid mis-reading
  if (m_RotValue > 590 && m_RotValue < 630)
    m_RotResult = ('B');  // press button
  if (m_RotValue > 670 && m_RotValue < 710)
    m_RotResult = ('R');  // turn right
  if (m_RotValue > 780 && m_RotValue < 820)
    m_RotResult = ('L');  // turn left
}
