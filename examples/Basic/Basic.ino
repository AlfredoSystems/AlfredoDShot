/*
  Basic - bidirectional DShot on one pin, printing RPM.

  Wiring: GPIO 8 -> [100 ohm] -> ESC signal, with a 1k pull-up to 3V3 on the ESC
  side of the series resistor. Share ground. See README.md for the diagram.
*/

#include <AlfredoDShot.h>

const int PIN_ESC = 8;
const uint8_t MOTOR_POLES = 14;

AlfredoDShot esc;

void setup() {
  AlfredoDShot::releaseBootloader(PIN_ESC);  // must be first, see AlfredoDShot.h

  Serial.begin(115200);
  esc.begin(PIN_ESC, DSHOT600, true, MOTOR_POLES);
}

void loop() {
  static uint32_t next = micros();
  static uint32_t lastPrint = 0;

  if ((int32_t)(micros() - next) < 0) return;
  next += 1000;  // 1 kHz

  esc.sendThrottle(0.08f);  // 8 %

  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.printf("rpm %8.0f   erpm %6lu   loss %.2f%%\n", esc.rpm(),
                  (unsigned long)esc.erpm(), esc.lossPercent());
  }
}
