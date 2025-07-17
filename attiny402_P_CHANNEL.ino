#include <Wire.h>

#include <avr/pgmspace.h>

#define MOSFET_PIN       0      // PA6 = Pin 0
#define SHUTDOWN_PIN     1      // PA7 = Pin 1
#define SDA_PIN          PIN_PA1  // PA1 = SDA
#define SCL_PIN          PIN_PA2  // PA2 = SCL
#define DS3231_ADDR      0x68

#define OFF               0
#define STARTING          1
#define WAIT_FOR_SHUTDOWN 2
#define SHUTDOWN_DELAY    3

const unsigned long OFF_TIME            = 180000UL;  // 3 Minuten OFF-Zeit
const unsigned long ON_TIME             = 25000UL;   // 25 Sekunden Bootzeit
const unsigned long SHUTDOWN_WAIT_TIME  = 12000UL;   // 12 Sekunden Verzögerung nach Shutdown

// Sonnenauf- und -untergangszeiten (UTC), monatlich (Januar–Dezember) + 2 Std wäre Normalzeit 
const uint8_t sunriseHours[]   PROGMEM = {5, 4, 4, 3, 2, 2, 2, 3, 4, 4, 5, 5};
const uint8_t sunriseMinutes[] PROGMEM = {30,45, 0, 0,30,30,45,30,15,45,15,45};
const uint8_t sunsetHours[]    PROGMEM = {14,15,16,17,18,19,18,18,17,16,14,14};
const uint8_t sunsetMinutes[]  PROGMEM = {30,30,30,30,30, 0,45, 0, 0, 0,30,15};

uint8_t currentState = OFF;
unsigned long timer = 0;

// Hilfsfunktion: BCD zu Dezimal
uint8_t bcdToDec(uint8_t val) {
  return (val >> 4) * 10 + (val & 0x0F);
}

// Stunden korrekt aus BCD (inkl. 12h/24h-Modus) dekodieren
uint8_t decodeHour(uint8_t bcdHour) {
  if (bcdHour & 0x40) {  // 12h-Modus
    bool isPM = bcdHour & 0x20;
    uint8_t hour = bcdToDec(bcdHour & 0x1F);
    if (isPM && hour < 12) hour += 12;
    if (!isPM && hour == 12) hour = 0;
    return hour;
  } else {
    return bcdToDec(bcdHour & 0x3F); // 24h-Modus
  }
}

// RTC lesen: Minuten, Stunden, Monat aus DS3231
bool readRTC(uint8_t &hour, uint8_t &minute, uint8_t &month) {
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(0x01); // Minutenregister
  if (Wire.endTransmission() != 0) return false;

  Wire.requestFrom(DS3231_ADDR, 5);
  if (Wire.available() < 5) return false;

  minute = bcdToDec(Wire.read());
  uint8_t rawHour = Wire.read();
  hour = decodeHour(rawHour);
  Wire.read(); // Wochentag ignorieren
  Wire.read(); // Datum ignorieren
  month = bcdToDec(Wire.read() & 0x1F); // nur untere 5 Bit

  return true;
}

// Zeitvergleich (h1:m1 < h2:m2)
bool timeLess(uint8_t h1, uint8_t m1, uint8_t h2, uint8_t m2) {
  return (h1 < h2) || (h1 == h2 && m1 < m2);
}

// Prüft, ob aktuelle Zeit zwischen Sonnenaufgang und Sonnenuntergang liegt
bool isInActiveTime(uint8_t hour, uint8_t minute, uint8_t month) {
  if (month < 1 || month > 12) return false;
  uint8_t idx = month - 1;

  int srH = pgm_read_byte(&sunriseHours[idx]);


  int srM = pgm_read_byte(&sunriseMinutes[idx]) - 0;  // 30 Minuten früher
  if (srM < 0) { srM += 60; srH--; if (srH < 0) srH = 23; }

  int ssH = pgm_read_byte(&sunsetHours[idx]);
  int ssM = pgm_read_byte(&sunsetMinutes[idx]) + 30;   // 30 Minuten später
  if (ssM >= 60) { ssM -= 60; ssH++; if (ssH > 23) ssH = 0; }

  return !timeLess(hour, minute, srH, srM) && !timeLess(ssH, ssM, hour, minute);
}

void setup() {

  Wire.begin();

  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, HIGH);  // Raspberry aus

  pinMode(SHUTDOWN_PIN, INPUT);   // Shutdown-Eingang vom Pi (kein Pullup)

  timer = millis();
}

void loop() {
  static bool rtcOk = false;
  static unsigned long lastRtcCheck = 0;
  const unsigned long rtcCheckInterval = 10000UL;  // alle 10 Sek. RTC prüfen
  unsigned long now = millis();

  // RTC regelmäßig lesen
  if (now - lastRtcCheck >= rtcCheckInterval) {
    uint8_t hour, minute, month;
    rtcOk = readRTC(hour, minute, month);
    lastRtcCheck = now;

    if (!rtcOk || !isInActiveTime(hour, minute, month)) {
      digitalWrite(MOSFET_PIN, HIGH);
      currentState = OFF;
      timer = now;
      return;
    }
  }

  if (!rtcOk) return;

  // Zustandsautomat
  switch (currentState) {
    case OFF:
      if (now - timer >= OFF_TIME) {
        digitalWrite(MOSFET_PIN, LOW);
        currentState = STARTING;
        timer = now;
      }
      break;

    case STARTING:
      if (now - timer >= ON_TIME) {
        currentState = WAIT_FOR_SHUTDOWN;
      }
      break;

    case WAIT_FOR_SHUTDOWN:
      if (digitalRead(SHUTDOWN_PIN) == HIGH) {
        currentState = SHUTDOWN_DELAY;
        timer = now;
      }
      break;

    case SHUTDOWN_DELAY:
      if (now - timer >= SHUTDOWN_WAIT_TIME) {
        digitalWrite(MOSFET_PIN, HIGH);
        currentState = OFF;
        timer = now;
      }
      break;
  }
}
