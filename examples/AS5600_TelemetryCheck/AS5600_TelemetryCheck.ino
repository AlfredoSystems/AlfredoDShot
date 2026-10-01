/*
  AS5600_TelemetryCheck - validate bidirectional DShot RPM against a real encoder

  Runs the ESC on one pin and compares the eRPM the ESC reports against the
  shaft speed measured by an AS5600 magnetic encoder on the motor. If the two
  agree, your telemetry decode and pole count are right.

  Wiring (ESP32-S3), see README.md for the full diagram:

                          3V3
                           |
                          [ ] 1k pull-up
                           |
    GPIO 8 ---[ 100 ]------+------------------- ESC signal
                           .
    GND -------------------.------------------- ESC ground

    GPIO 33 -> AS5600 SDA (Qwiic)
    GPIO 34 -> AS5600 SCL (Qwiic)

  Serial commands (115200 baud):
    0-100   set throttle percent          s  stop
    a       auto ramp sweep on/off        r  reset link statistics
    e / E   enable / disable EDT          d  reverse spin direction
    b       beacon (beep)                 p  toggle Serial Plotter output
*/

#include <AlfredoDShot.h>
#include <Wire.h>

// ---- configuration ----------------------------------------------------------
const int PIN_ESC = 8;
const int PIN_I2C_SDA_QWIIC = 33;
const int PIN_I2C_SCL_QWIIC = 34;

const uint8_t MOTOR_POLES = 14;      // 12N14P outrunner. Change to match yours.
const DShotMode DSHOT_RATE = DSHOT600;

const uint32_t LOOP_US = 1000;       // 1 kHz control loop
const uint32_t REPORT_MS = 100;      // print rate
// -----------------------------------------------------------------------------

AlfredoDShot esc;

// AS5600
const uint8_t AS5600_ADDR = 0x36;
const uint8_t AS5600_REG_STATUS = 0x0B;
const uint8_t AS5600_REG_RAW_ANGLE = 0x0C;

float throttlePct = 0.0f;
bool autoSweep = false;
bool plotterMode = false;

// encoder integration state
uint16_t lastAngle = 0;
bool angleValid = false;
int32_t angleAccum = 0;   // signed counts since the last report
uint32_t encReads = 0, encErrors = 0;

// telemetry averaging over the report window
uint64_t erpmSum = 0;
uint32_t erpmCount = 0;

bool as5600Read(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)AS5600_ADDR, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// 12-bit unfiltered angle. Raw (0x0C) rather than filtered (0x0E) so we get the
// least possible lag when comparing against the ESC.
bool as5600Angle(uint16_t &out) {
  uint8_t b[2];
  if (!as5600Read(AS5600_REG_RAW_ANGLE, b, 2)) return false;
  out = (((uint16_t)b[0] << 8) | b[1]) & 0x0FFF;
  return true;
}

void as5600CheckMagnet() {
  uint8_t st;
  if (!as5600Read(AS5600_REG_STATUS, &st, 1)) {
    Serial.println("AS5600: NOT RESPONDING - check Qwiic wiring on GPIO 33/34");
    return;
  }
  Serial.print("AS5600: ");
  if (!(st & 0x20)) Serial.println("no magnet detected!");
  else if (st & 0x08) Serial.println("magnet too strong (move it away)");
  else if (st & 0x10) Serial.println("magnet too weak (move it closer)");
  else Serial.println("magnet OK");
}

void printHelp() {
  Serial.println();
  Serial.println("0-100 throttle % | s stop | a auto sweep | r reset stats");
  Serial.println("e/E EDT on/off | d reverse | b beep | p plotter | h help");
  Serial.println();
}

void handleSerial() {
  static char buf[8];
  static uint8_t len = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c >= '0' && c <= '9') {
      if (len < sizeof(buf) - 1) buf[len++] = c;
      continue;
    }
    if ((c == '\n' || c == '\r') && len) {
      buf[len] = 0;
      len = 0;
      float v = atof(buf);
      throttlePct = constrain(v, 0.0f, 100.0f);
      autoSweep = false;
      Serial.printf("# throttle -> %.0f%%\n", throttlePct);
      continue;
    }
    len = 0;
    switch (c) {
      case 's': throttlePct = 0; autoSweep = false; Serial.println("# stop"); break;
      case 'a': autoSweep = !autoSweep; Serial.printf("# sweep %s\n", autoSweep ? "on" : "off"); break;
      case 'r': esc.resetStats(); Serial.println("# stats reset"); break;
      case 'e': esc.command(DSHOT_CMD_EDT_ENABLE); Serial.println("# EDT enable"); break;
      case 'E': esc.command(DSHOT_CMD_EDT_DISABLE); Serial.println("# EDT disable"); break;
      case 'd': esc.command(DSHOT_CMD_SPIN_DIRECTION_REVERSED); Serial.println("# reverse"); break;
      case 'b': esc.command(DSHOT_CMD_BEACON1, 1); break;
      case 'p': plotterMode = !plotterMode; break;
      case 'h': printHelp(); break;
      default: break;
    }
  }
}

const char *statusName(DShotRxStatus s) {
  switch (s) {
    case DSHOT_RX_OK: return "OK";
    case DSHOT_RX_NO_REPLY: return "NO-REPLY";
    case DSHOT_RX_FRAMING: return "FRAMING";
    case DSHOT_RX_BAD_GCR: return "BAD-GCR";
    case DSHOT_RX_BAD_CRC: return "BAD-CRC";
    default: return "IDLE";
  }
}

void setup() {
  AlfredoDShot::releaseBootloader(PIN_ESC);  // must be first, see AlfredoDShot.h

  Serial.begin(115200);
  Serial.println("\nAlfredoDShot - AS5600 telemetry check");

  Wire.begin(PIN_I2C_SDA_QWIIC, PIN_I2C_SCL_QWIIC, 400000);
  as5600CheckMagnet();

  if (!esc.begin(PIN_ESC, DSHOT_RATE, true, MOTOR_POLES)) {
    Serial.println("esc.begin() failed - out of RMT channels?");
    while (true) delay(100);
  }

  printHelp();
  Serial.println("echo = our own frame read back off the wire; 31 = wiring OK,");
  Serial.println("       0 = nothing on the line, 1-30 = pull-up missing/weak.");
  Serial.println();
  Serial.println("thr%  dshotRPM  encRPM   err%   poles?  loss% echo  noRep fram gcr  crc  status");
}

void loop() {
  static uint32_t nextTick = micros();
  static uint32_t nextReport = millis();
  static uint32_t windowStart = micros();
  static float sweepPhase = 0;

  // Drain serial every pass, not just on tick boundaries.
  handleSerial();

  // ---- fixed-rate control loop ----
  if ((int32_t)(micros() - nextTick) < 0) return;
  nextTick += LOOP_US;

  if (autoSweep) {
    sweepPhase += 0.0004f;
    if (sweepPhase > 1.0f) sweepPhase -= 1.0f;
    throttlePct = 5.0f + 25.0f * (1.0f - cosf(sweepPhase * TWO_PI)) * 0.5f;
  }

  // Send the frame and pick up the previous frame's telemetry.
  if (esc.sendThrottle(throttlePct / 100.0f)) {
    erpmSum += esc.erpm();
    erpmCount++;
  }

  // Sample the encoder and integrate, unwrapping the 12-bit rollover.
  uint16_t a;
  if (as5600Angle(a)) {
    encReads++;
    if (angleValid) {
      int16_t d = (int16_t)((a - lastAngle) & 0x0FFF);
      if (d > 2048) d -= 4096;
      angleAccum += d;
    }
    lastAngle = a;
    angleValid = true;
  } else {
    encErrors++;
  }

  // ---- periodic report ----
  if ((int32_t)(millis() - nextReport) < 0) return;
  nextReport += REPORT_MS;

  uint32_t now = micros();
  float dt = (now - windowStart) * 1e-6f;
  windowStart = now;

  // AS5600 gives shaft revolutions directly; 4096 counts per turn.
  float encRpm = (dt > 0) ? (angleAccum / 4096.0f) / dt * 60.0f : 0.0f;
  angleAccum = 0;

  float dshotErpm = erpmCount ? (float)(erpmSum / erpmCount) : 0.0f;
  erpmSum = 0;
  erpmCount = 0;
  float dshotRpm = dshotErpm * 2.0f / MOTOR_POLES;

  float encAbs = fabsf(encRpm);
  float err = (encAbs > 50.0f) ? (dshotRpm - encAbs) / encAbs * 100.0f : 0.0f;
  // If this lands on a clean even number that is not MOTOR_POLES, that is your
  // actual pole count.
  float impliedPoles = (encAbs > 50.0f) ? 2.0f * dshotErpm / encAbs : 0.0f;

  const AlfredoDShot::Stats &st = esc.stats();

  if (plotterMode) {
    Serial.printf("dshot_rpm:%.0f\tenc_rpm:%.0f\n", dshotRpm, encAbs);
  } else {
    Serial.printf("%4.0f %9.0f %8.0f %6.1f %7.1f %6.2f %4u %7lu %4lu %4lu %4lu  %s",
                  throttlePct, dshotRpm, encRpm, err, impliedPoles,
                  esc.lossPercent(), esc.echoPulses(), (unsigned long)st.noReply,
                  (unsigned long)st.framing, (unsigned long)st.badGcr,
                  (unsigned long)st.badCrc, statusName(esc.status()));
    if (esc.edtSeen()) {
      Serial.printf("  | %.0fC %.2fV %.2fA", esc.temperatureC(), esc.voltage(),
                    esc.current());
    }
    if (encErrors) Serial.printf("  | i2c_err %lu", (unsigned long)encErrors);
    Serial.println();
  }
}
