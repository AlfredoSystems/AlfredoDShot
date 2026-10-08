/*
  PestoLink_OneMotor - drive one ESC from a gamepad over Bluetooth.

  Needs the PestoLink-Receive library (and ArduinoBLE, which it depends on).
  Open https://pestol.ink in Chrome or Edge, click Connect, pick "DShotBot",
  and the left stick's vertical axis runs the motor. The ESC's RPM, voltage,
  current and temperature show up in PestoLink's terminal.

  The ESC should have "3D mode" ON in the AM32 configurator: stick forward is
  one direction, stick back is the other. With 3D mode off, set THREE_D_MODE
  below to false and only stick forward does anything.

  The motor stops whenever PestoLink disconnects, and the first ~1 s after
  boot is AM32's arming window, during which throttle is ignored.

  Wiring: GPIO 4 -> [100 ohm] -> ESC signal, with a 1k pull-up to 3V3 on the ESC
  side of the series resistor. Share ground. See README.md for the diagram,
  and "ESCs with a series resistor" there if telemetry never arrives.
*/

#include <AlfredoDShot.h>
#include <PestoLink-Receive.h>

const int PIN_ESC = 9;
const uint8_t MOTOR_POLES = 14;    // magnet count, 14 for a 12N14P outrunner
const bool THREE_D_MODE = true;    // match the ESC's "3D mode" setting
const float MAX_THROTTLE = 1.0f;  // 0..1, limit for bench testing
const float DEADBAND = 0.05f;      // stick travel treated as zero
const uint8_t AXIS = 1;            // left stick, vertical

AlfredoDShot esc;

// Stick position (-1..1) to a DShot value. In 3D mode the range is split in
// two halves: 48..1047 is reverse and 1048..2047 is forward, each slowest to
// fastest. Without 3D mode, 48..2047 is forward only.
uint16_t throttleValue(float stick) {
  if (stick > MAX_THROTTLE) stick = MAX_THROTTLE;
  if (stick < -MAX_THROTTLE) stick = -MAX_THROTTLE;
  if (!THREE_D_MODE) return stick > 0 ? 48 + (uint16_t)(stick * 1999.0f + 0.5f) : 0;
  if (!(fabsf(stick) > 0)) return 0;  // also catches NaN
  uint16_t steps = (uint16_t)(fabsf(stick) * 999.0f + 0.5f);
  return (stick > 0 ? 1048 : 48) + steps;
}

void setup() {
  AlfredoDShot::releaseBootloader(PIN_ESC);  // must be first, see AlfredoDShot.h

  Serial.begin(115200);
  PestoLink.begin("DShotBot");
  esc.begin(PIN_ESC, DSHOT300, true, MOTOR_POLES);
}

// Asks the ESC for extended telemetry (voltage, current, temperature). AM32
// ignores commands until it has armed, and the library can't tell when that
// is, so wait until replies are coming back and re-send every second until
// EDT frames arrive. Only with the motor stopped: a command replaces the
// throttle for a few frames.
void enableEdt(bool stopped) {
  static uint32_t lastSentMs = 0;
  if (esc.edtSeen() || !stopped || !esc.telemetryValid() || esc.commandPending()) return;
  if (millis() - lastSentMs < 1000) return;
  lastSentMs = millis();
  esc.command(DSHOT_CMD_EDT_ENABLE);
}

void loop() {
  static uint32_t next = micros();
  static uint32_t lastPrint = 0;

  if ((int32_t)(micros() - next) < 0) return;
  next += 1000;  // 1 kHz

  // Stick up is negative on a gamepad, so flip it: forward = up.
  float stick = 0;
  if (PestoLink.isConnected()) {
    stick = -PestoLink.getAxis(AXIS);
    if (fabsf(stick) < DEADBAND) stick = 0;
  }
  enableEdt(stick == 0);
  esc.send(throttleValue(stick));

  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    if (esc.telemetryValid()) {
      PestoLink.printfTerminal("rpm %.0f  %.1f V  %.0f A  %.0f C", esc.rpm(),
                               esc.voltage(), esc.current(), esc.temperatureC());
      if (!isnan(esc.voltage())) PestoLink.printBatteryVoltage(esc.voltage());
    } else {
      PestoLink.printfTerminal("no telemetry (%s)", esc.isArmed() ? "check wiring" : "arming");
    }
    Serial.printf("stick %+.2f  rpm %8.0f  loss %.1f%%\n", stick, esc.rpm(),
                  esc.lossPercent());
  }
}
