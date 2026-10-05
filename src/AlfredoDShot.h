// AlfredoDShot - bidirectional DShot for ESP32 with RMT
//
// Targets AM32 ESCs. One GPIO per ESC: an RMT TX channel and an RMT RX channel
// share the pad (open-drain + external pull-up) so the ESC can pull the line low
// to answer. See README.md for wiring.

#pragma once

#include <Arduino.h>

#if !defined(ESP_ARDUINO_VERSION) || ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 0, 0)
#error "AlfredoDShot needs ESP32 Arduino core 3.x (ESP-IDF 5 RMT driver)"
#endif

#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>

enum DShotMode : uint8_t {
  DSHOT150,
  DSHOT300,
  DSHOT600,  // default; what AM32 setups normally run
  DSHOT1200
};

// Special commands AM32 acts on. AM32 requires 6 consecutive frames for all of
// these except the beacons, which fire immediately.
enum DShotCommand : uint16_t {
  DSHOT_CMD_MOTOR_STOP = 0,
  DSHOT_CMD_BEACON1 = 1,
  DSHOT_CMD_BEACON2 = 2,
  DSHOT_CMD_BEACON3 = 3,
  DSHOT_CMD_BEACON4 = 4,
  DSHOT_CMD_BEACON5 = 5,
  DSHOT_CMD_ESC_INFO = 6,
  DSHOT_CMD_SPIN_DIRECTION_1 = 7,
  DSHOT_CMD_SPIN_DIRECTION_2 = 8,
  DSHOT_CMD_3D_MODE_OFF = 9,   // AM32: bidirectional (reversible) rotation off
  DSHOT_CMD_3D_MODE_ON = 10,   // AM32: bidirectional (reversible) rotation on
  DSHOT_CMD_SETTINGS_REQUEST = 11,
  DSHOT_CMD_SAVE_SETTINGS = 12,
  DSHOT_CMD_EDT_ENABLE = 13,
  DSHOT_CMD_EDT_DISABLE = 14,
  DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
  DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
};

// Why the last frame did not yield an RPM reading.
enum DShotRxStatus : uint8_t {
  DSHOT_RX_OK,        // valid frame decoded (eRPM or EDT)
  DSHOT_RX_IDLE,      // bidirectional disabled, or no frame sent yet
  DSHOT_RX_NO_REPLY,  // ESC never pulled the line low
  DSHOT_RX_FRAMING,   // reply seen but run lengths made no sense
  DSHOT_RX_BAD_GCR,   // 5-bit quintet not in the GCR table
  DSHOT_RX_BAD_CRC,   // GCR decoded but the 4-bit checksum failed
};

class AlfredoDShot {
 public:
  // Call first in setup(), before begin(). AM32 reboots ~0.5 s after the signal
  // stops, and its bootloader only runs the motor firmware if it sees this line
  // go LOW - the required pull-up beats its internal pull-down, so a booting
  // ESP leaves the ESC stuck in bootloader, deaf until power-cycled. Holding
  // low across that window fixes it; 2500 ms covers both AM32 timeouts.
  static void releaseBootloader(int pin, uint16_t holdMs = 2500);

  // pin        : ESC signal pin (needs an external pull-up to 3V3, see README)
  // motorPoles : magnet count, used to convert eRPM to shaft RPM (12N14P = 14)
  bool begin(int pin, DShotMode mode = DSHOT600, bool bidirectional = true,
             uint8_t motorPoles = 14);
  void end();

  // Send one frame and harvest the reply to the *previous* frame. Call this at
  // your control-loop rate (1-4 kHz is a good range). Returns true when fresh
  // telemetry was decoded on this call.
  //
  // value: 0 = stop, 48..2047 = throttle. 1..47 are commands - use command().
  bool send(uint16_t value);
  bool sendThrottle(float throttle);  // 0.0 .. 1.0 -> 48..2047

  // AM32 needs ~1 s of unbroken zero throttle before it arms, and any non-zero
  // value restarts its timer. send() quietly forces zero until that has passed,
  // so just call sendThrottle() from your first loop - no arming loop needed.
  bool isArmed() const { return _armed; }

  // Queue a special command. It replaces the throttle for the next `repeat`
  // frames. Held back until armed, since AM32 ignores commands before that.
  void command(uint16_t cmd, uint8_t repeat = 6);
  bool commandPending() const { return _cmdRepeat > 0; }

  // ---- eRPM telemetry ----
  DShotRxStatus status() const { return _status; }
  bool telemetryValid() const { return _status == DSHOT_RX_OK; }
  uint32_t erpm() const { return _erpm; }              // electrical RPM
  float rpm() const { return _erpm * 2.0f / _poles; }  // shaft RPM
  uint32_t periodUs() const { return _periodUs; }      // raw commutation period
  uint32_t ageUs() const;                              // since last valid eRPM

  // ---- Extended DShot Telemetry (AM32: send DSHOT_CMD_EDT_ENABLE first) ----
  // AM32 interleaves these with eRPM frames, so they update at a few Hz.
  bool edtSeen() const { return _edtSeen; }
  float temperatureC() const { return _edtTemp; }  // NAN until received, ~5 Hz
  float voltage() const { return _edtVolts; }      // NAN until received, ~5 Hz
  // 1 A steps. Needs AM32 2.21 or newer; older AM32 reads double.
  float current() const { return _edtAmps; }       // NAN until received, ~25 Hz
  // AM32 never emits these two, so they stay 0. Kept for other EDT firmware.
  uint8_t stress() const { return _edtStress; }
  uint8_t escStatus() const { return _edtStatus; }

  // ---- link health ----
  struct Stats {
    uint32_t sent;
    uint32_t ok;
    uint32_t noReply;
    uint32_t framing;
    uint32_t badGcr;
    uint32_t badCrc;
  };
  const Stats &stats() const { return _stats; }
  void resetStats() { _stats = Stats{}; }
  float lossPercent() const;

  // Drive our frames push-pull and release the line to open-drain as soon as
  // each frame is out, the way flight controllers do. The pull-up then only
  // has to hold the idle level, so it can be weak (10k or more), which lets an
  // ESC with a series resistor on its signal pad pull the reply low enough.
  // Off by default. Needs the ESC's reply to start after the TX-done interrupt
  // has run; AM32 waits ~25 us, which leaves a wide margin.
  void setPushPull(bool on) { _pushPull = on; }

  // Time the RMT RX waits for the line to go quiet before ending a capture.
  // Must sit above the ESC's ~30 us turnaround and below your frame interval.
  void setRxIdleTimeoutUs(uint16_t us) { _rxIdleNs = us * 1000u; }

  // Wiring check. The receiver shares the pin with the transmitter, so it reads
  // our own frame back off the wire before the ESC replies. Expect 31.
  //   31    line is driven and released cleanly - wiring is good
  //   0     nothing on the wire: RMT not reaching the pad, or line stuck
  //   1..30 edges are being lost - pull-up missing or too weak
  uint16_t echoPulses() const { return _echoPulses; }

  uint16_t lastFrame() const { return _frame; }  // for scope/debug work

 private:
  static bool IRAM_ATTR onRxDone(rmt_channel_handle_t ch,
                                 const rmt_rx_done_event_data_t *ev, void *ctx);
  static bool IRAM_ATTR onTxDone(rmt_channel_handle_t ch,
                                 const rmt_tx_done_event_data_t *ev, void *ctx);
  void buildFrame(uint16_t value);
  DShotRxStatus decode(size_t nsym);
  DShotRxStatus decodeReply(size_t p, size_t np, uint32_t telemQ8, uint16_t &d16);
  void apply(uint16_t data12);
  void armRx();

  rmt_channel_handle_t _tx = nullptr;
  rmt_channel_handle_t _rx = nullptr;
  rmt_encoder_handle_t _enc = nullptr;

  int _pin = -1;
  bool _bidir = true;
  volatile bool _pushPull = false;
  uint8_t _poles = 14;

  uint16_t _tbit = 0, _t0h = 0, _t1h = 0;  // TX bit timings, RMT ticks
  uint32_t _telemQ8 = 0;                   // telemetry bit period, ticks << 8
  uint32_t _telemNomQ8 = 0;                // what the spec says it should be
  uint16_t _gapMin = 0;                    // turnaround gap threshold, ticks
  uint32_t _rxIdleNs = 60000;
  uint32_t _rxWaitUs = 0;  // worst-case capture length

  rmt_symbol_word_t _txSym[16];
  rmt_symbol_word_t _rxBuf[64];
  volatile size_t _rxCount = 0;
  volatile bool _rxDone = false;
  bool _rxArmed = false;

  uint16_t _frame = 0;
  uint16_t _cmd = 0;
  uint8_t _cmdRepeat = 0;
  uint16_t _echoPulses = 0;
  int64_t _beginUs = 0;
  bool _armed = false;

  DShotRxStatus _status = DSHOT_RX_IDLE;
  uint32_t _erpm = 0;
  uint32_t _periodUs = 0;
  int64_t _lastOkUs = 0;

  bool _edtSeen = false;
  float _edtTemp = NAN, _edtVolts = NAN, _edtAmps = NAN;
  uint8_t _edtStress = 0, _edtStatus = 0;

  Stats _stats{};
};
