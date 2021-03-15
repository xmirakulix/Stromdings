#include <Power.h>
#include <Wireless.h>
#include <SPI.h>

// #define DEBUG  // toggle Debug Output

#ifdef DEBUG
unsigned long m_PowerHandlerDuration = 0; // store the duation of last handler run
#endif

// ######## Power measurement
const uint8_t m_CsPins[] = {4, 5, 6, 7};                     // list of ADC Chip-Select pins
const uint8_t m_AdcCnt = sizeof(m_CsPins) / sizeof(uint8_t); // number of ADCs
const uint8_t m_NumPorts = m_AdcCnt * 4;                     // number of available ports
uint16_t m_LastMeasurements[m_NumPorts];                     // last measurement per port
unsigned long m_LastMeasureTime = 0;                         // time of last measurement
uint8_t m_curMeasurePort = 0;                                // last measured Pin

// const int m_PsuCalibration = 228.20;               // calibration factor for used 9VAC PSU

// ################# POWER stuff #################

void handlePower()
{
#ifdef DEBUG
  m_PowerHandlerDuration = millis();
#endif

  unsigned int measureInterval = 1 * 1000; // in ms
  // do power measurements
  if ((millis() - m_LastMeasureTime) > measureInterval)
  {
    m_LastMeasureTime = millis();

    // 0x0 (00|00) to 0xF (11|11) -> 4x chip | 4x pin
    int csPin = m_curMeasurePort >> 2; // extraxt chip
    int adcPin = m_curMeasurePort & 3; // extract pin

#ifdef DEBUG
    Serial.print(F("Measuring: Port:"));
    Serial.print(m_curMeasurePort);
    Serial.print(F(", Chip: "));
    Serial.print(m_CsPins[csPin]);
    Serial.print(F(", Pin: "));
    Serial.println(adcPin);
#endif

    int measureDuration = 20 * 1000; // 20k µs, 50Hz = 20ms per wave
    m_LastMeasurements[m_curMeasurePort] = getAdcPower(m_CsPins[csPin], adcPin, measureDuration, 220, 2000, 230);

#ifdef DEBUG
    Serial.print(F("W["));
    Serial.print(m_curMeasurePort);
    Serial.print(F("]: "));
    Serial.print(m_LastMeasurements[m_curMeasurePort]);
    Serial.println();
#endif

    sendMeasurement(m_curMeasurePort);
    m_curMeasurePort++;
    if (m_curMeasurePort == m_NumPorts)
      m_curMeasurePort = 0;
  }

#ifdef DEBUG
  Serial.print(F("Power handler took: "));
  Serial.print(millis() - m_PowerHandlerDuration);
  Serial.println(F("ms"));
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
  SPI.begin(); // init the SPI bus
  for (int i = 0; i < m_AdcCnt; i++)
  {
    pinMode(m_CsPins[i], OUTPUT);    // set CS pin to output
    digitalWrite(m_CsPins[i], HIGH); // pull CS pin high to disable the ADCs
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
  Serial.print(F("getAdc: "));
  Serial.print(adcVal);
  Serial.print(F(" V: "));
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
  power = (adcVal * lsbValue); // convert to nV;	46 * 610.351 = 28.076.146
  power /= shuntOhms;          // conv to sek A	28.076.146 / 220 = 127.618,8454
  power /= 1000;               //					127.618 / 1000 = 127
  power *= turns;              // conv to prim A	127 * 2000 = 254.000
  power /= 1000;               //					254.000 / 1000 = 254
  power *= voltage;            // conv to W(rms)	254 * 229 = 58.166
  power /= 1000;               //					58.166 / 1000 = 58

#ifdef DEBUG
  Serial.print(F("getAdcPower() Res: "));
  Serial.print(adcVal);
  Serial.print(F(" Power: "));
  Serial.print(power);
  Serial.println();
#endif

  return int(power); // Power in W
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
 *	csPin: Arduino chipselect pin for this ADC
 *	adcPin: which input pin to use on ADC (MCP3304: 0,2,4,6)
 *	measureDuration: measure interval in usec
 *
 * Return: RMS value of reading series on given pin
 */
int readAdcRms(int csPin, int adcPin, unsigned long measureDuration)
{
  unsigned long startTime = micros();
  int count = 0;    // Anzahl der Messungen
  int adcValue = 0; // nimmt den Messwert vom ADC auf
  unsigned long adcRms =
      0; // nimmt die quadrierten Summen der gesamten Periode auf

  SPI.beginTransaction(
      SPISettings(1700000, MSBFIRST, SPI_MODE0)); // max 1,7MHz for MCP3304
  do
  {
    adcValue = readAdc(csPin, adcPin);
    adcRms +=
        (unsigned long)adcValue * (unsigned long)adcValue; // calculate square
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
  Serial.print(F("readAdcRms("));
  Serial.print(csPin);
  Serial.print(",");
  Serial.print(adcPin);
  Serial.print(F(") Cnt: "));
  Serial.print(count);
  Serial.print(F(" Res: "));
  Serial.print(adcRms);
  Serial.print(F(" Return: "));
  Serial.println(ret);
#endif

  return ret;
}