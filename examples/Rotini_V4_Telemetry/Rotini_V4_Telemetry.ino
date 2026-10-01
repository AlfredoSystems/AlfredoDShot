/*
  Rotini_V4_Telemetry - drive Rotini V4's FOO and BAR ESCs from the
  AlfredoTelemetry web page and plot what the ESCs report back over DShot.

  Needs the AlfredoTelemetry library. Flash its Dongle example to a second
  ESP32 plugged into your computer, open
  AlfredoTelemetry/extras/TelemetryViewer/index.html in Chrome or Edge, click
  Connect, and pair with "Rotini".

  Tunables panel:
    enable        motors only spin while this is checked. It clears itself
                  when the page disconnects, and the throttles are held at 0
                  while it is off, so re-enabling never jumps to an old value.
    foo_throttle  0-100 %
    bar_throttle  0-100 %
    edt           Extended DShot Telemetry (temperature, voltage, current).
                  Sent to each ESC once it is answering and its throttle
                  is 0, and re-sent until the data arrives.

  Plotted per motor (foo_* and bar_*): rpm, throttle, loss_pct, status,
  echo, temp_c, volts, amps. Plus vin (board input voltage) and enabled.
  rpm and the EDT values show as gaps while the ESC isn't replying; status
  and echo (see reportMotor) say why.

  Wiring: each ESC signal needs a 1k pull-up to 3V3 on the ESC side of a
  series resistor, see README.md. Motor direction is set in the AM32
  configurator (AM32_ConfiguratorLink example), not here.
*/

#include <AlfredoDShot.h>
#include <AlfredoTelemetry.h>

// ---- Rotini V4 pins (from SimpleMelt's Rotini-V4 example) -------------------
const int PIN_MOTOR_FOO = 40;
const int PIN_MOTOR_BAR = 41;
const int PIN_STATUS_LED = 4;
const int PIN_SNS_VIN = 10;

// ---- configuration ----------------------------------------------------------
const uint8_t MOTOR_POLES = 14;      // 12N14P outrunner. Change to match yours.
// Rotini V4 signal path: Q1 (BSS138) level shifter with R33 = 10k on the ESP
// side and R36 = 5.1k on the ESC side (R35/R34 for BAR). Weak pull-ups let the
// ESC, which has ~470 ohm in series with its signal pad, pull its reply down
// to ~0.6 V; the stock 1k/1k left it at 1.75 V, which the ESP never saw.
//
// Push-pull drives our rising edges instead of the 5.1k pull-up, which is what
// makes DSHOT600 work (its shortest high is only 0.42 us). With PUSH_PULL off,
// use DSHOT300. Avoid DSHOT150: AM32 replies at the wrong speed for it.
const DShotMode DSHOT_RATE = DSHOT600;
const bool PUSH_PULL = true;
const float MAX_THROTTLE_PCT = 100;  // lower this for bench testing

const uint32_t LOOP_US = 1000;       // 1 kHz control loop
const uint32_t SLOW_MS = 50;         // loss, EDT and vin, 20 times a second
// -----------------------------------------------------------------------------

// Tunables, changed from the page
bool enable = false;
float fooThrottle = 0;  // percent
float barThrottle = 0;
bool edt = true;

// One ESC plus the names of its telemetry channels
struct Motor {
  AlfredoDShot esc;
  const char *rpm, *throttle, *loss, *status, *echo, *temp, *volts, *amps;
  AlfredoDShot::Stats last{};  // stats at the previous slow report
  bool edtOn = false;          // last EDT command sent to this ESC
  uint32_t edtSentMs = 0;
};

Motor foo{{}, "foo_rpm", "foo_throttle", "foo_loss_pct", "foo_status", "foo_echo",
          "foo_temp_c", "foo_volts", "foo_amps"};
Motor bar{{}, "bar_rpm", "bar_throttle", "bar_loss_pct", "bar_status", "bar_echo",
          "bar_temp_c", "bar_volts", "bar_amps"};

// Sends one frame to the ESC and logs the RPM from the reply to the last one.
// Every channel is added from the start, as NaN (a gap in the plot) while
// there is no reading, so a dead link shows up instead of a missing channel.
void runMotor(Motor &m, float throttlePct) {
  m.esc.sendThrottle(throttlePct / 100.0f);
  Telemetry.add(m.throttle, throttlePct);
  Telemetry.add(m.rpm, m.esc.telemetryValid() ? m.esc.rpm() : NAN);
}

// Turns EDT on or off to match the edt tunable. Waits until the ESC is
// answering, because a command sent while it is still booting (battery plugged
// in after the ESP, or AM32 playing its startup tones) is silently lost, and
// re-sends every second until EDT frames arrive. Commands replace the throttle
// for a few frames, so this only runs with the motor stopped.
void updateEdt(Motor &m, float throttlePct, const char *name) {
  if (throttlePct != 0 || !m.esc.telemetryValid() || m.esc.commandPending()) return;
  bool retry = edt && m.edtOn && !m.esc.edtSeen() && millis() - m.edtSentMs >= 1000;
  if (edt == m.edtOn && !retry) return;

  m.esc.command(edt ? DSHOT_CMD_EDT_ENABLE : DSHOT_CMD_EDT_DISABLE);
  if (edt != m.edtOn) Telemetry.printf("%s EDT %s\n", name, edt ? "on" : "off");
  m.edtOn = edt;
  m.edtSentMs = millis();
}

// Loss since the last call, so the plot shows what is happening now rather
// than an average since boot.
//   status: 0 OK, 1 idle, 2 no reply, 3 framing, 4 bad GCR, 5 bad CRC
//   echo:   31 = wiring good, 0 = nothing on the line, 1-30 = weak pull-up
void reportMotor(Motor &m) {
  const AlfredoDShot::Stats &st = m.esc.stats();
  uint32_t sent = st.sent - m.last.sent;
  uint32_t ok = st.ok - m.last.ok;
  m.last = st;
  if (sent) Telemetry.add(m.loss, 100.0f * (sent - ok) / sent);
  Telemetry.add(m.status, m.esc.status());
  Telemetry.add(m.echo, m.esc.echoPulses());

  // NAN until the ESC sends EDT
  Telemetry.add(m.temp, m.esc.temperatureC());
  Telemetry.add(m.volts, m.esc.voltage());
  Telemetry.add(m.amps, m.esc.current());
}

float readVin() {
  return analogReadMilliVolts(PIN_SNS_VIN) * 0.001f * 8.21f;  // Rotini V4 divider
}

void setup() {
  // Must be first, see AlfredoDShot.h. holdMs = 0 leaves FOO low and moves on,
  // so both ESCs share one 2.5 s hold instead of waiting for each in turn.
  AlfredoDShot::releaseBootloader(PIN_MOTOR_FOO, 0);
  AlfredoDShot::releaseBootloader(PIN_MOTOR_BAR);

  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  Serial.begin(115200);

  if (!foo.esc.begin(PIN_MOTOR_FOO, DSHOT_RATE, true, MOTOR_POLES) ||
      !bar.esc.begin(PIN_MOTOR_BAR, DSHOT_RATE, true, MOTOR_POLES)) {
    Serial.println("esc.begin() failed - out of RMT channels?");
    while (true) delay(100);
  }
  foo.esc.setPushPull(PUSH_PULL);
  bar.esc.setPushPull(PUSH_PULL);

  Telemetry.begin("Rotini");
  Telemetry.tune("enable", &enable);
  Telemetry.tune("foo_throttle", &fooThrottle);
  Telemetry.tune("bar_throttle", &barThrottle);
  Telemetry.tune("edt", &edt);
}

void loop() {
  static uint32_t nextTick = micros();
  static uint32_t lastSlowMs = 0;
  static uint32_t lastStatusMs = 0;
  static bool wasConnected = false;

  if ((int32_t)(micros() - nextTick) < 0) return;
  nextTick += LOOP_US;

  // Stop if the page goes away, and make the user re-enable on purpose.
  bool connected = Telemetry.isConnected();
  if (wasConnected && !connected && enable) {
    enable = false;
    Serial.println("page disconnected - motors disabled");
  }
  wasConnected = connected;

  if (!enable) {
    fooThrottle = 0;
    barThrottle = 0;
  }
  fooThrottle = constrain(fooThrottle, 0.0f, MAX_THROTTLE_PCT);
  barThrottle = constrain(barThrottle, 0.0f, MAX_THROTTLE_PCT);

  updateEdt(foo, fooThrottle, "foo");
  updateEdt(bar, barThrottle, "bar");

  runMotor(foo, fooThrottle);
  runMotor(bar, barThrottle);

  if (millis() - lastSlowMs >= SLOW_MS) {
    lastSlowMs = millis();
    reportMotor(foo);
    reportMotor(bar);
    Telemetry.add("vin", readVin());
    Telemetry.add("enabled", enable);
  }
  Telemetry.send();

  digitalWrite(PIN_STATUS_LED, enable);

  // Link diagnostics over USB, once a second
  if (millis() - lastStatusMs >= 1000) {
    lastStatusMs = millis();
    Telemetry.printStatus(Serial);
    printMotor("foo", foo.esc);
    printMotor("bar", bar.esc);
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

// Frame counts are since boot: which column grows says where replies are lost.
void printMotor(const char *name, AlfredoDShot &esc) {
  const AlfredoDShot::Stats &st = esc.stats();
  Serial.printf("%s: %s echo %u %-8s rpm %6.0f loss %5.1f%% | ok %lu noRep %lu fram %lu gcr %lu crc %lu\n",
                name, esc.isArmed() ? "armed " : "arming", esc.echoPulses(),
                statusName(esc.status()), esc.rpm(), esc.lossPercent(),
                (unsigned long)st.ok, (unsigned long)st.noReply,
                (unsigned long)st.framing, (unsigned long)st.badGcr,
                (unsigned long)st.badCrc);
}
