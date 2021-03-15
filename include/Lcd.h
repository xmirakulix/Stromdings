#include <Arduino.h>

#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include "hd44780ioClass/hd44780_I2Cexp.h" // i2c expander i/o class header

// define globals
extern hd44780_I2Cexp m_Lcd;
extern unsigned long m_LastPageChange;

// define functions
void setupLcd();
void handleLcd();

void enableLcdBacklight();
void disableLcdBacklight();
void displayDiagInfo();