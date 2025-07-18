// Version 0.1.2
#include <Wire.h>
#include <avr/pgmspace.h>

// Pinbelegung
#define MOSFET_PIN       0      // PA6 = Pin 0
#define SHUTDOWN_PIN     1      // PA7 = Pin 1
#define SDA_PIN          PIN_PA1
#define SCL_PIN          PIN_PA2
#define DS3231_ADDR      0x68

#define OFF               0
#define STARTING          1
#define WAIT_FOR_SHUTDOWN 2
#define SHUTDOWN_DELAY    3

const unsigned long OFF_TIME            = 180000UL;  // 3 Minuten OFF-Zeit
const unsigned long ON_TIME             = 25000UL;   // 25 Sekunden Bootzeit
const unsigned long SHUTDOWN_WAIT_TIME  = 12000UL;   // 12 Sekunden Verzögerung nach Shutdown

// Sonnenauf- und -untergangszeiten (UTC für Mitteleuropa, Monate 1–12)
const uint8_t sunriseHours[]   PROGMEM = {7, 6, 6, 5, 4, 4, 4, 5, 6, 6, 7, 7};
const uint8_t sunriseMinutes[] PROGMEM = {30,45, 0, 0,30,30,45,30,15,45,15,45};
const uint8_t sunsetHours[]    PROGMEM = {16,17,18,19,20,21,20,20,19,18,16,16};
const uint8_t sunsetMinutes[]  PROGMEM = {30,30,30,30,30, 0,45, 0, 0, 0,30,15};

uint8_t currentState = OFF;
unsigned long timer = 0;

// --- Hilfsfunktionen ---

uint8_t bcdToDec(uint8_t val) {
  return (val >> 4) * 10 + (val & 0x0F);
}

uint8_t decodeHour(uint8_t bcdHour) {
  if (bcdHour & 0x40) {
    bool isPM = bcdHour & 0x20;
    uint8_t hour = bcdToDec(bcdHour & 0x1F);
    if (isPM && hour < 12) hour += 12;
    if (!isPM && hour == 12) hour = 0;
    return hour;
  } else {
    return bcdToDec(bcdHour & 0x3F);
  }
}

// Zeitzone (1 = Winter, 2 = Sommer)
int timezoneOffset(uint8_t month) {
  return (month >= 3 && month <= 10) ? 2 : 1;
}

// RTC auslesen: Minuten, Stunden, Monat
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
  month = bcdToDec(Wire.read() & 0x1F);

  return true;
}

// Zeitvergleich (h1:m1 < h2:m2)
bool timeLess(uint8_t h1, uint8_t m1, uint8_t h2, uint8_t m2) {
  return (h1 < h2) || (h1 == h2 && m1 < m2);
}

// --- Korrigierte Zeitfenster-Funktion ---

bool isInActiveTime(uint8_t hour, uint8_t minute, uint8_t month) {
  if (month < 1 || month > 12) return false;
  uint8_t idx = month - 1;

  int offset = timezoneOffset(month);

  int srH = pgm_read_byte(&sunriseHours[idx]) + offset;
  int srM = pgm_read_byte(&sunriseMinutes[idx]) - 30;  // 30 min früher
  if (srM < 0) { srM += 60; srH--; if (srH < 0) srH = 23; }

  int ssH = pgm_read_byte(&sunsetHours[idx]) + offset;
  int ssM = pgm_read_byte(&sunsetMinutes[idx]) + 30;   // 30 min später
  if (ssM >= 60) { ssM -= 60; ssH++; if (ssH > 23) ssH = 0; }

  // Zeitfenster über Mitternacht erkennen
  if (srH < ssH || (srH == ssH && srM < ssM)) {
    // "Normales" Tagfenster: sunrise < sunset (z.B. 7–18 Uhr)
    return !timeLess(hour, minute, srH, srM) && !timeLess(ssH, ssM, hour, minute);
  } else {
    // Über Mitternacht: sunset < sunrise (z.B. 21–4 Uhr)
    return !timeLess(hour, minute, srH, srM) || !timeLess(ssH, ssM, hour, minute);
  }
}

// --- Hauptprogramm ---

void setup() {
  Wire.begin();

  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, HIGH);  // Raspberry aus (LOW = an, HIGH = aus)

  pinMode(SHUTDOWN_PIN, INPUT);    // Shutdown-Pin vom Raspberry (kein Pullup)

  timer = millis();
}

void loop() {
  static bool rtcOk = false;
  static unsigned long lastRtcCheck = 0;
  const unsigned long rtcCheckInterval = 10000UL;  // alle 10 Sek. RTC prüfen
  unsigned long now = millis();

  if (now - lastRtcCheck >= rtcCheckInterval) {
    uint8_t hour, minute, month;
    rtcOk = readRTC(hour, minute, month);
    lastRtcCheck = now;

    if (!rtcOk || !isInActiveTime(hour, minute, month)) {
      digitalWrite(MOSFET_PIN, HIGH);  // Raspberry aus
      currentState = OFF;
      timer = now;
      return;
    }
  }

  if (!rtcOk) return;

  switch (currentState) {
    case OFF:
      if (now - timer >= OFF_TIME) {
        digitalWrite(MOSFET_PIN, LOW);  // Raspberry ein
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
        digitalWrite(MOSFET_PIN, HIGH);  // Raspberry aus
        currentState = OFF;
        timer = now;
      }
      break;
  }
}
