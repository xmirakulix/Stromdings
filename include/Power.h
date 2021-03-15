#include <Arduino.h>
#include <stdint.h>

// declare globals
extern uint16_t m_LastMeasurements[];
extern const uint8_t m_AdcCnt;
extern const uint8_t m_NumPorts;

// declare functions
void setupPower();

void handlePower();

int getAdcPower(int csPin, int adcPin, unsigned long measureDuration, int shuntOhms, int turns, int voltage);
int readAdcRms(int csPin, int adcPin, unsigned long measureDuration);