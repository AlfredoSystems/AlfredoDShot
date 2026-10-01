#include "AlfredoDShot.h"

#include <driver/gpio.h>
#include <esp_timer.h>
#include <hal/gpio_ll.h>

// 80 MHz = APB, divider 1. 12.5 ns ticks give ~106 ticks per telemetry bit at
// DShot600, which is plenty to resolve 1/2/3-bit GCR run lengths.
static constexpr uint32_t RMT_RES_HZ = 80000000;

// One memory block per channel, so four ESCs still fit on an ESP32-S3
// (4 TX + 4 RX = 8 blocks). 48 symbols = 96 pulses; a capture holds our own
// 16-bit frame (32 pulses) plus at most 21 telemetry pulses.
#ifdef SOC_RMT_MEM_WORDS_PER_CHANNEL
static constexpr size_t RMT_MEM = SOC_RMT_MEM_WORDS_PER_CHANNEL;
#else
static constexpr size_t RMT_MEM = 48;
#endif

static const uint32_t kBitrate[] = {150000, 300000, 600000, 1200000};

// AM32 wants 1.0 s of zeros, plus a little for it to detect the protocol first.
static constexpr int64_t ARM_HOLD_US = 1200000;

// Bidirectional-DShot GCR: 5-bit quintet back to its nibble, 0xFF = invalid.
static const uint8_t kGcrToNibble[32] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  // 0x00-0x07
    0xFF, 0x09, 0x0A, 0x0B, 0xFF, 0x0D, 0x0E, 0x0F,  // 0x08-0x0F
    0xFF, 0xFF, 0x02, 0x03, 0xFF, 0x05, 0x06, 0x07,  // 0x10-0x17
    0xFF, 0x00, 0x08, 0x01, 0xFF, 0x04, 0x0C, 0xFF,  // 0x18-0x1F
};

void AlfredoDShot::releaseBootloader(int pin, uint16_t holdMs) {
  // Open drain so we only pull down, and via the IDF rather than pinMode() so
  // the Arduino peripheral manager does not claim the pad ahead of the RMT
  // driver in begin().
  gpio_config_t cfg = {};
  cfg.pin_bit_mask = 1ULL << pin;
  cfg.mode = GPIO_MODE_OUTPUT_OD;
  cfg.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&cfg);
  gpio_set_level((gpio_num_t)pin, 0);
  delay(holdMs);
}

bool AlfredoDShot::begin(int pin, DShotMode mode, bool bidirectional,
                         uint8_t motorPoles) {
  end();
  _pin = pin;
  _bidir = bidirectional;
  _poles = motorPoles ? motorPoles : 14;

  const uint32_t br = kBitrate[mode];
  _tbit = RMT_RES_HZ / br;
  _t0h = (uint16_t)((uint32_t)_tbit * 3 / 8);
  _t1h = (uint16_t)((uint32_t)_tbit * 3 / 4);
  // Telemetry runs at 5/4 of the DShot bitrate. Kept in 24.8 fixed point so
  // run-length rounding stays exact.
  _telemQ8 = (uint32_t)(((uint64_t)RMT_RES_HZ * 4 * 256) / (5ull * br));
  _telemNomQ8 = _telemQ8;
  // The ESC waits ~30 us before answering; nothing in our own frame stays high
  // for longer than 5/8 of a bit, so 2.5 bits cleanly separates the two.
  _gapMin = (uint16_t)((uint32_t)_tbit * 5 / 2);

  // RX first, then TX: creating a channel reconfigures the pad, so the TX
  // channel with io_od_mode has to go last or open-drain gets cleared. This is
  // the same order ESP-IDF's own 1-Wire RMT driver uses.
  if (_bidir) {
    rmt_rx_channel_config_t rxc = {};
    rxc.gpio_num = (gpio_num_t)pin;
    rxc.clk_src = RMT_CLK_SRC_DEFAULT;
    rxc.resolution_hz = RMT_RES_HZ;
    rxc.mem_block_symbols = RMT_MEM;
    if (rmt_new_rx_channel(&rxc, &_rx) != ESP_OK) return false;

    rmt_rx_event_callbacks_t cbs = {};
    cbs.on_recv_done = onRxDone;
    if (rmt_rx_register_event_callbacks(_rx, &cbs, this) != ESP_OK) {
      end();
      return false;
    }
  }

  rmt_tx_channel_config_t txc = {};
  txc.gpio_num = (gpio_num_t)pin;
  txc.clk_src = RMT_CLK_SRC_DEFAULT;
  txc.resolution_hz = RMT_RES_HZ;
  txc.mem_block_symbols = RMT_MEM;
  txc.trans_queue_depth = 2;
  txc.flags.io_loop_back = _bidir;  // keep the pad readable by the RX channel
  txc.flags.io_od_mode = _bidir;    // open drain, so the ESC can pull it low
  if (rmt_new_tx_channel(&txc, &_tx) != ESP_OK) {
    end();
    return false;
  }

  if (_bidir) {
    rmt_tx_event_callbacks_t tcbs = {};
    tcbs.on_trans_done = onTxDone;
    if (rmt_tx_register_event_callbacks(_tx, &tcbs, this) != ESP_OK) {
      end();
      return false;
    }
  }

  rmt_copy_encoder_config_t enc = {};
  if (rmt_new_copy_encoder(&enc, &_enc) != ESP_OK) {
    end();
    return false;
  }

  if (_bidir) {
    // Only touch the pull resistor here. Do NOT call gpio_set_direction() on
    // this pin: it routes SIG_GPIO_OUT_IDX to the pad and disconnects the RMT
    // output signal, which silently kills transmission. Open drain and input
    // enable are already applied by the TX channel's io_od_mode /
    // io_loop_back flags. The external pull-up does the real work anyway - the
    // internal one is far too weak for telemetry edge rates.
    gpio_pullup_en((gpio_num_t)pin);
  }

  if (rmt_enable(_tx) != ESP_OK) {
    end();
    return false;
  }
  if (_rx && rmt_enable(_rx) != ESP_OK) {
    end();
    return false;
  }

  // Send a stop frame so the line settles at its idle level (high when
  // bidirectional) instead of whatever the RMT channel powered up with.
  _beginUs = esp_timer_get_time();
  _armed = false;
  send(0);
  return true;
}

void AlfredoDShot::end() {
  if (_rx) {
    rmt_disable(_rx);
    rmt_del_channel(_rx);
    _rx = nullptr;
  }
  if (_tx) {
    rmt_disable(_tx);
    rmt_del_channel(_tx);
    _tx = nullptr;
  }
  if (_enc) {
    rmt_del_encoder(_enc);
    _enc = nullptr;
  }
  _rxArmed = false;
  _rxDone = false;
  _status = DSHOT_RX_IDLE;
}

bool AlfredoDShot::onRxDone(rmt_channel_handle_t,
                            const rmt_rx_done_event_data_t *ev, void *ctx) {
  AlfredoDShot *self = (AlfredoDShot *)ctx;
  self->_rxCount = ev->num_symbols;
  self->_rxDone = true;
  return false;
}

// Push-pull mode: the frame is out, so hand the line back to the pull-up
// before the ESC starts answering. Only the pad's open-drain bit changes; the
// RMT signal routing stays put (see the gpio_set_direction() note in begin()).
bool AlfredoDShot::onTxDone(rmt_channel_handle_t,
                            const rmt_tx_done_event_data_t *, void *ctx) {
  AlfredoDShot *self = (AlfredoDShot *)ctx;
  if (self->_pushPull) gpio_ll_od_enable(&GPIO, self->_pin);
  return false;
}

void AlfredoDShot::armRx() {
  rmt_receive_config_t cfg = {};
  cfg.signal_range_min_ns = 100;  // glitch filter, well under any real pulse
  cfg.signal_range_max_ns = _rxIdleNs;
  _rxDone = false;
  _rxArmed = (rmt_receive(_rx, _rxBuf, sizeof(_rxBuf), &cfg) == ESP_OK);
}

void AlfredoDShot::buildFrame(uint16_t value) {
  uint16_t packet = (value & 0x07FF) << 1;  // telemetry-request bit stays 0
  uint16_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
  if (_bidir) crc = (~crc) & 0x0F;  // inverted CRC is what asks for a reply
  _frame = (packet << 4) | crc;

  const uint8_t active = _bidir ? 0 : 1;  // bidirectional DShot is inverted
  for (int i = 0; i < 16; ++i) {
    uint16_t hi = (_frame & (0x8000 >> i)) ? _t1h : _t0h;
    _txSym[i].level0 = active;
    _txSym[i].duration0 = hi;
    _txSym[i].level1 = !active;
    _txSym[i].duration1 = _tbit - hi;
  }
}

bool AlfredoDShot::send(uint16_t value) {
  bool fresh = false;

  if (_bidir && _rxArmed) {
    // Worst case: our frame + turnaround + 21 telemetry bits + idle timeout.
    const uint32_t budget =
        ((uint32_t)_tbit * 16 + ((_telemQ8 >> 8) * 21)) / (RMT_RES_HZ / 1000000) +
        40 + _rxIdleNs / 1000 + 30;
    const int64_t deadline = esp_timer_get_time() + budget;
    while (!_rxDone && esp_timer_get_time() < deadline) {
    }

    if (_rxDone) {
      _rxDone = false;
      _rxArmed = false;
      _status = decode(_rxCount);
    } else {
      // Capture never finished, so the line produced no edges at all.
      rmt_disable(_rx);
      rmt_enable(_rx);
      _rxArmed = false;
      _echoPulses = 0;
      _status = DSHOT_RX_NO_REPLY;
    }

    switch (_status) {
      case DSHOT_RX_OK: _stats.ok++; break;
      case DSHOT_RX_NO_REPLY: _stats.noReply++; break;
      case DSHOT_RX_FRAMING: _stats.framing++; break;
      case DSHOT_RX_BAD_GCR: _stats.badGcr++; break;
      case DSHOT_RX_BAD_CRC: _stats.badCrc++; break;
      default: break;
    }
    fresh = (_status == DSHOT_RX_OK);
  }

  // AM32 arms only after ~1 s of unbroken zeros (main.c: armed_timeout_count),
  // and any non-zero frame restarts its timer. Hold everything at zero until
  // then - including commands, which it also ignores while disarmed.
  if (!_armed && esp_timer_get_time() - _beginUs < ARM_HOLD_US) {
    value = 0;
  } else {
    _armed = true;
    if (_cmdRepeat) {
      value = _cmd;
      _cmdRepeat--;
    }
  }

  // Arm the receiver before transmitting. The capture then covers our own
  // frame, the turnaround gap and the reply in one shot, so there is no ISR
  // race in the ~30 us window where the ESC starts talking.
  if (_bidir) armRx();

  buildFrame(value);
  // The RMT output idles high, so dropping open-drain here drives the line
  // high until the frame's first bit - no glitch. onTxDone() restores it.
  if (_bidir && _pushPull) gpio_ll_od_disable(&GPIO, _pin);
  rmt_transmit_config_t txc = {};
  txc.flags.eot_level = _bidir ? 1 : 0;
  rmt_transmit(_tx, _enc, _txSym, sizeof(_txSym), &txc);
  _stats.sent++;

  return fresh;
}

bool AlfredoDShot::sendThrottle(float throttle) {
  if (!(throttle > 0.0f)) return send(0);  // also catches NaN
  if (throttle > 1.0f) throttle = 1.0f;
  return send(48 + (uint16_t)(throttle * 1999.0f + 0.5f));
}

void AlfredoDShot::command(uint16_t cmd, uint8_t repeat) {
  _cmd = cmd;
  _cmdRepeat = repeat ? repeat : 1;
}

#define PULSE_LVL(i) ((i) & 1 ? _rxBuf[(i) >> 1].level1 : _rxBuf[(i) >> 1].level0)
#define PULSE_DUR(i) ((i) & 1 ? _rxBuf[(i) >> 1].duration1 : _rxBuf[(i) >> 1].duration0)

DShotRxStatus AlfredoDShot::decode(size_t nsym) {
  const size_t np = nsym * 2;
  if (np == 0) return DSHOT_RX_NO_REPLY;

  // Step past our own transmitted frame by finding the turnaround gap: the
  // first long high run. It swallows the tail of our last bit plus the ~30 us
  // the ESC waits before answering.
  size_t p = 0;
  bool found = false;
  for (; p < np; ++p) {
    uint16_t d = PULSE_DUR(p);
    if (d == 0) break;  // RMT marks end of capture with a zero duration
    if (PULSE_LVL(p) && d >= _gapMin) {
      found = true;
      ++p;
      break;
    }
  }
  // Everything before the gap is our own frame read back off the wire - a free
  // check that the pin is actually being driven and released. See echoPulses().
  _echoPulses = found ? (uint16_t)(p - 1) : (uint16_t)p;
  if (!found || p >= np) return DSHOT_RX_NO_REPLY;

  // The spec puts the reply at 5/4 of our bitrate, but AM32 doesn't always
  // follow it: at DShot300 it answers at the DShot600 telemetry rate. So when
  // the expected bit time fails, try half and double, and keep whichever
  // passes GCR and CRC. A wrong bit time passes both about 1 time in 256.
  uint16_t d16;
  DShotRxStatus st = decodeReply(p, np, _telemQ8, d16);
  if (st == DSHOT_RX_FRAMING || st == DSHOT_RX_BAD_GCR || st == DSHOT_RX_BAD_CRC) {
    const uint32_t alt[] = {_telemNomQ8, _telemNomQ8 / 2, _telemNomQ8 * 2};
    for (uint32_t q : alt) {
      if (q == _telemQ8) continue;
      if (decodeReply(p, np, q, d16) == DSHOT_RX_OK) {
        _telemQ8 = q;
        st = DSHOT_RX_OK;
        break;
      }
    }
  }
  if (st == DSHOT_RX_OK) apply(d16 >> 4);
  return st;
}

DShotRxStatus AlfredoDShot::decodeReply(size_t p, size_t np, uint32_t telemQ8,
                                        uint16_t &d16) {
  // Rebuild the 21 transmitted bits from run lengths. The line is inverted, so
  // a low run contributes 1s and a high run 0s.
  uint32_t v = 0;
  int bits = 0;
  for (; p < np && bits < 21; ++p) {
    uint16_t d = PULSE_DUR(p);
    if (d == 0) break;
    if (bits == 0 && PULSE_LVL(p)) return DSHOT_RX_FRAMING;  // must start low

    uint32_t n = ((uint32_t)d * 256u + (telemQ8 >> 1)) / telemQ8;
    if (n == 0) n = 1;
    if (bits + (int)n > 21) n = 21 - bits;  // trailing idle run, clamp it

    v = (v << n) | (PULSE_LVL(p) ? 0u : ((1u << n) - 1u));
    bits += n;
  }
  if (bits == 0) return DSHOT_RX_NO_REPLY;
  if (bits < 21) v <<= (21 - bits);  // line went idle early, pad with 0s

  // A 1 in the GCR stream means "the line toggled here".
  const uint32_t gcr = (v ^ (v >> 1)) & 0xFFFFFu;
  const uint8_t n3 = kGcrToNibble[(gcr >> 15) & 0x1F];
  const uint8_t n2 = kGcrToNibble[(gcr >> 10) & 0x1F];
  const uint8_t n1 = kGcrToNibble[(gcr >> 5) & 0x1F];
  const uint8_t n0 = kGcrToNibble[gcr & 0x1F];
  if ((n3 | n2 | n1 | n0) & 0xF0) return DSHOT_RX_BAD_GCR;

  d16 = (n3 << 12) | (n2 << 8) | (n1 << 4) | n0;
  uint16_t csum = d16 ^ (d16 >> 8);
  csum ^= csum >> 4;
  if ((csum & 0x0F) != 0x0F) return DSHOT_RX_BAD_CRC;  // CRC is inverted
  return DSHOT_RX_OK;
}

#undef PULSE_LVL
#undef PULSE_DUR

void AlfredoDShot::apply(uint16_t data12) {
  // Extended telemetry reuses eRPM codings the ESC never emits: a zero mantissa
  // MSB with a non-zero exponent. Top nibble is the type, low byte the value.
  const uint8_t type = data12 >> 8;
  if (type != 0 && (type & 1) == 0) {
    const uint8_t val = data12 & 0xFF;
    _edtSeen = true;
    switch (type) {
      case 0x02: _edtTemp = val; break;           // deg C
      case 0x04: _edtVolts = val * 0.25f; break;  // 0.25 V steps, per spec
      // 0.5 A steps, not the 1 A the EDT spec calls for: AM32 sends
      // `actual_current / 50` and actual_current is in centiamps. Its own KISS
      // path sends that same variable as centiamps, so AM32's two telemetry
      // outputs disagree by 2x. Reported upstream as AM32 issue #408 - if that
      // lands, delete the 0.5 here and go back to the spec.
      case 0x06: _edtAmps = val * 0.5f; break;
      case 0x0C: _edtStress = val; break;
      case 0x0E: _edtStatus = val; break;
      default: break;  // 0x08 / 0x0A are firmware debug channels
    }
    return;
  }

  if (data12 == 0x0FFF) {  // ESC says "not spinning"
    _periodUs = 0;
    _erpm = 0;
  } else {
    _periodUs = (uint32_t)(data12 & 0x1FF) << (data12 >> 9);
    _erpm = _periodUs ? (60000000u / _periodUs) : 0;
  }
  _lastOkUs = esp_timer_get_time();
}

uint32_t AlfredoDShot::ageUs() const {
  if (!_lastOkUs) return UINT32_MAX;
  return (uint32_t)(esp_timer_get_time() - _lastOkUs);
}

float AlfredoDShot::lossPercent() const {
  const uint32_t tried = _stats.ok + _stats.noReply + _stats.framing +
                         _stats.badGcr + _stats.badCrc;
  if (!tried) return 0.0f;
  return 100.0f * (tried - _stats.ok) / tried;
}
