#include <Lcd.h>
#include <Power.h>
#include <Rotary.h>
#include <Wireless.h>

// #define DEBUG  // toggle Debug Output

#ifdef DEBUG
unsigned long m_LcdHandlerDuration = 0; // store the duation of last handler run
#endif

// ######## LCD display
hd44780_I2Cexp m_Lcd;                // declare lcd object: auto locate & auto config expander chip
unsigned long m_LastPageChange = 0;  // time of last page change
uint8_t m_LcdCurrentPage = 0;        // which page to display
bool m_LcdShowInfoPage = false;      // should we display the info page?
unsigned long m_LastBacklightOn = 0; // time of last backlight on
bool m_IsLcdBacklight = true;        // is the backlight on?

void setupLcd()
{
  int status = m_Lcd.begin(16, 2);
  if (status) // non zero status means it was unsuccesful
  {
    // hd44780 has a fatalError() routine that blinks an led if possible
    // begin() failed so blink error code using the onboard LED if possible
    Serial.println(F("Failed to initialize LCD"));
    hd44780::fatalError(status); // does not return
  }

#ifdef DEBUG
  Serial.print(F("setupLcd(): LCD initialized..."));
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

  if (m_RotResult == 'L') // rotary turned back
  {
    m_LcdCurrentPage = (m_LcdCurrentPage - 2) % m_AdcCnt; // LCD page back
    m_LastPageChange = 0;                                 // change page immediately
    m_RotResult = ' ';
    enableLcdBacklight();
  }

  if (m_RotResult == 'R') // rotary turned forward
  {
    m_LastPageChange = 0; // render next page immediately
    m_RotResult = ' ';
    enableLcdBacklight();
  }

  if (m_RotResult == 'B') // button pressed
  {
    m_LastPageChange = millis(); // delay next page change
    displayDiagInfo();
    enableLcdBacklight();
    m_LastPageChange = millis();
  }

  if (m_IsLcdBacklight && (millis() - m_LastBacklightOn) > 10 * 1000)
  {
    disableLcdBacklight();
  }

  if ((millis() - m_LastPageChange) > 5 * 1000)
  {
    m_LcdCurrentPage = (m_LcdCurrentPage + 1) % m_AdcCnt; // next page
    int position = 0;                                     // 4 positions per page (top left, top right, ...)

    m_Lcd.clear();

    for (int i = position; i < 4; i++)
    {
      m_Lcd.setCursor((i << 3) & 8, (i >> 1) & 1); // col 0+8, row 0+1
      m_Lcd.print("P");
      m_Lcd.print((m_LcdCurrentPage << 2) + i + 1, DEC); // P1 to P16
      m_Lcd.print(":");
      m_Lcd.print(m_LastMeasurements[(m_LcdCurrentPage << 2) + i], DEC); // measurement[0] to [15]
      m_Lcd.print("W");
    }

    m_LastPageChange = millis();
  }
#ifdef DEBUG
  Serial.print(F("Lcd handler took: "));
  Serial.print(millis() - m_LcdHandlerDuration);
  Serial.println(F("ms"));
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
  debugWiFi();
}