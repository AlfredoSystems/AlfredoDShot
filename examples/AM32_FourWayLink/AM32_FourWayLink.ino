/*
  AM32_FourWayLink - configure several AM32 ESCs from am32.ca over their DShot
  wires, with the ESP32-S3 acting as a flight controller.

  The am32.ca configurator reaches multiple ESCs through a flight controller:
  it talks MSP to the FC, asks it to switch into "4-way interface" mode, and
  then sends 4-way commands that name an ESC by number. The FC speaks the
  AM32 bootloader protocol to that ESC's signal wire. This sketch does the
  same job. Every ESC in ESC_PINS shows up in the configurator, and each one
  can be read, configured and flashed in turn.

  AM32_ConfiguratorLink is the one-ESC version: a plain serial bridge that
  the configurator drives in "Direct Connect" mode.

  ---------------------------------------------------------------------------
  REQUIRED BOARD SETTINGS (both, or it will not build):
    Tools > USB Mode         : "USB-OTG (TinyUSB)"
    Tools > USB CDC On Boot  : "Disabled"
  ---------------------------------------------------------------------------
  WIRING - identical to the DShot wiring, see README.md, one pin per ESC:

                            3V3
                             |
                            [ ] 1 kO pull-up
                             |
    GPIO 4 ----[ 100 O ]-----+----------------------- ESC 1 signal
    GPIO 5 ----[ 100 O ]-----+---[ ] 1 kO -- 3V3 ---- ESC 2 signal
                             .
    GND ---------------------.----------------------- ESC grounds

    The ESCs run from their own battery. Ground must be shared.
  ---------------------------------------------------------------------------
  HOW TO CONNECT:
    1. Flash this sketch and power the ESCs. With nothing driving DShot, an
       AM32 ESC reboots into its bootloader within ~2 s and waits there as
       long as the pull-up holds its line HIGH, so the order doesn't matter.
    2. In am32.ca click Connect, pick the "AM32 4-way Linker" port (115200),
       and the configurator lists every ESC it finds.

  An ESC that the configurator has told to reboot (after a flash, or on
  disconnect) runs its motor firmware, times out without a signal and lands
  back in its bootloader a couple of seconds later, so Reconnect just works.
  Never call AlfredoDShot::releaseBootloader() here: it pulls the line LOW,
  which is exactly what makes AM32 leave its bootloader.

  Protocol references: betaflight/src/main/io/serial_4way*.c (the interface
  this emulates) and AM32-bootloader/bootloader/main.c (the ESC side).
*/

#include <HardwareSerial.h>

#include "USB.h"
#include "USBCDC.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

#if ARDUINO_USB_MODE
#error "Set Tools > USB Mode to 'USB-OTG (TinyUSB)'."
#endif
#if ARDUINO_USB_CDC_ON_BOOT
#error "Set Tools > USB CDC On Boot to 'Disabled' for this VID/PID-spoofing build."
#endif

// ESC signal pins, in the order the configurator numbers them (ESC #1 first).
// Rotini V4: {40, 41} for FOO and BAR.
const int ESC_PINS[] = {40, 41};
const uint8_t ESC_COUNT = sizeof(ESC_PINS) / sizeof(ESC_PINS[0]);

const long ESC_BAUD = 19200;  // AM32 bootloader one-wire rate, 8N1
const uint32_t TURNAROUND_US = 2000;  // quiet time after the ESC's last byte before we send

// am32.ca only offers ports whose USB vendor ID is on its flight-controller
// list. 0x1209 (pid.codes) is on it.
const uint16_t SPOOF_VID = 0x1209;
const uint16_t SPOOF_PID = 0x4D32;

USBCDC USBSerial;       // PC-facing TinyUSB CDC port
HardwareSerial ESC(1);  // UART1, re-routed to whichever ESC pin is selected

// ---- 4-way interface protocol (from betaflight serial_4way.c) ---------------
enum : uint8_t {
  CMD_LOCAL_ESCAPE = 0x2F,   // '/' host -> us
  CMD_REMOTE_ESCAPE = 0x2E,  // '.' us -> host

  CMD_INTERFACE_TEST_ALIVE = 0x30,
  CMD_PROTOCOL_GET_VERSION = 0x31,
  CMD_INTERFACE_GET_NAME = 0x32,
  CMD_INTERFACE_GET_VERSION = 0x33,
  CMD_INTERFACE_EXIT = 0x34,
  CMD_DEVICE_RESET = 0x35,
  CMD_DEVICE_INIT_FLASH = 0x37,
  CMD_DEVICE_ERASE_ALL = 0x38,
  CMD_DEVICE_PAGE_ERASE = 0x39,
  CMD_DEVICE_READ = 0x3A,
  CMD_DEVICE_WRITE = 0x3B,
  CMD_DEVICE_C2CK_LOW = 0x3C,
  CMD_DEVICE_READ_EEPROM = 0x3D,
  CMD_DEVICE_WRITE_EEPROM = 0x3E,
  CMD_INTERFACE_SET_MODE = 0x3F,
  CMD_DEVICE_VERIFY = 0x40,

  ACK_OK = 0x00,
  ACK_I_INVALID_CMD = 0x02,
  ACK_I_INVALID_CRC = 0x03,
  ACK_I_VERIFY_ERROR = 0x04,
  ACK_I_INVALID_CHANNEL = 0x08,
  ACK_I_INVALID_PARAM = 0x09,
  ACK_D_GENERAL_ERROR = 0x0F,

  IM_SIL_BLB = 1,
  IM_ATM_BLB = 2,
  IM_SK = 3,
  IM_ARM_BLB = 4,  // AM32
};

const uint8_t PROTOCOL_VERSION = 108;
const char INTERFACE_NAME[] = "m4wFCIntf";
// betaflight reports its 4-way version 20.0.06 as (20006 / 100, 20006 % 100)
const uint8_t INTERFACE_VERSION_HI = 200;
const uint8_t INTERFACE_VERSION_LO = 6;

// ---- AM32 bootloader protocol (from AM32-bootloader main.c) -----------------
enum : uint8_t {
  BL_CMD_RUN = 0x00,
  BL_CMD_PROG_FLASH = 0x01,
  BL_CMD_ERASE_FLASH = 0x02,
  BL_CMD_READ_FLASH = 0x03,
  BL_CMD_KEEP_ALIVE = 0xFD,
  BL_CMD_SET_BUFFER = 0xFE,
  BL_CMD_SET_ADDRESS = 0xFF,

  BL_ACK_OK = 0x30,
  BL_ACK_BAD_CMD = 0xC1,
  BL_ACK_BAD_CRC = 0xC2,
};

// ---- MSP (just enough for am32.ca's handshake) ------------------------------
enum : uint8_t {
  MSP_API_VERSION = 1,
  MSP_FC_VARIANT = 2,
  MSP_MOTOR = 104,
  MSP_BATTERY_STATE = 130,
  MSP_MOTOR_CONFIG = 131,
  MSP_SET_PASSTHROUGH = 245,
};

// ---- state ------------------------------------------------------------------
int currentPin = -1;             // ESC pin UART1 is routed to, -1 = none yet
int selectedEsc = -1;            // index into ESC_PINS
bool connected = false;          // bootloader on selectedEsc answered InitFlash
uint8_t deviceInfo[4] = {0};     // signature lo, hi, bootloader pin code, mode
uint8_t interfaceMode = IM_ARM_BLB;

uint8_t inParams[256];
uint8_t outParams[256 + 3];      // room for a 256-byte read plus CRC and ACK
uint8_t blBuf[256 + 3];

// ---- debug log --------------------------------------------------------------
// USB carries the protocol, so there is no serial console. Failures are noted
// here instead, and MSP command 0xF0 (private to this sketch) returns and
// clears the text: send "$M<" 0x00 0xF0 0xF0 and read the "$M>" reply.
char dbg[1024];
size_t dbgLen = 0;

void dbgf(const char *fmt, ...) {
  if (dbgLen >= sizeof(dbg) - 1) return;
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(dbg + dbgLen, sizeof(dbg) - dbgLen, fmt, ap);
  va_end(ap);
  if (n > 0) dbgLen = min(dbgLen + (size_t)n, sizeof(dbg) - 1);
}

// =============================================================================
// One-wire UART on the selected ESC pin
// =============================================================================

// Puts UART1's TX and RX on `pin`, open-drain with the pull-up driving the
// HIGH level.
//
// ORDER IS LOAD-BEARING (see AM32_ConfiguratorLink): gpio_set_direction()
// detaches the UART signals from the pad, so the two esp_rom_gpio_connect_*
// calls must come after it.
void routeUart(int pin) {
  gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT_OUTPUT_OD);
  gpio_set_pull_mode((gpio_num_t)pin, GPIO_PULLUP_ONLY);
  esp_rom_gpio_connect_out_signal((gpio_num_t)pin, U1TXD_OUT_IDX, false, false);
  esp_rom_gpio_connect_in_signal((gpio_num_t)pin, U1RXD_IN_IDX, false);
  currentPin = pin;
  // which pad feeds UART1 RX, and which signal drives each ESC pad
  dbgf("route pin %d: rx<-gpio%lu", pin,
       (unsigned long)(REG_READ(GPIO_FUNC0_IN_SEL_CFG_REG + 4 * U1RXD_IN_IDX) & 0x3F));
  for (uint8_t i = 0; i < ESC_COUNT; i++) {
    dbgf(" out%d=%lu", ESC_PINS[i],
         (unsigned long)(REG_READ(GPIO_FUNC0_OUT_SEL_CFG_REG + 4 * ESC_PINS[i]) & 0x1FF));
  }
  dbgf(" (uart1 tx sig %d)\n", U1TXD_OUT_IDX);
}

// Moves the one-wire UART to another ESC. The previous pin is left as an
// input, where the external pull-up holds it HIGH and its ESC stays in its
// bootloader.
//
// UART1 is started once and re-routed afterwards: HardwareSerial::end() +
// begin() on a different pin was seen to hang on core 3.3.
void selectPin(int pin) {
  if (pin == currentPin) return;

  if (currentPin >= 0) {
    // gpio_set_direction() alone leaves the UART TX signal routed to the pad
    // (seen on core 3.3.10), so hand the pad back to plain GPIO explicitly.
    esp_rom_gpio_connect_out_signal((gpio_num_t)currentPin, SIG_GPIO_OUT_IDX, false, false);
    gpio_set_direction((gpio_num_t)currentPin, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)currentPin, GPIO_PULLUP_ONLY);
  } else {
    // Both RX and TX on the pin so the receiver is enabled; we need to hear
    // the ESC (and our own echo) on the same wire.
    ESC.setRxBufferSize(1024);  // a 256-byte write echoes back before we read it
    ESC.begin(ESC_BAUD, SERIAL_8N1, pin, pin);
  }
  routeUart(pin);
  // Re-routing the pad can produce one glitch byte on the receiver.
  delay(2);
  blDrain();
}

// Holds the selected ESC's line LOW for `ms`, then puts the UART back. AM32's
// bootloader jumps to the motor firmware when it sees the line low.
void pulseLow(uint32_t ms) {
  int pin = currentPin;
  if (pin < 0) return;
  gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT_OD);
  gpio_set_level((gpio_num_t)pin, 0);
  delay(ms);
  gpio_set_level((gpio_num_t)pin, 1);
  routeUart(pin);
}

void blDrain() {
  while (ESC.available()) ESC.read();
}

uint32_t lastRxUs = 0;  // when the ESC's last byte arrived

// Reads exactly n bytes, giving up if the line goes quiet for `idleMs`.
bool blReadBytes(uint8_t *buf, size_t n, uint32_t idleMs) {
  uint32_t last = millis();
  size_t got = 0;
  while (got < n) {
    int c = ESC.read();
    if (c >= 0) {
      buf[got++] = (uint8_t)c;
      lastRxUs = micros();
      last = millis();
    } else if (millis() - last > idleMs) {
      return false;
    }
  }
  return true;
}

// CRC-16/ARC as the bootloader computes it: poly 0xA001 reflected, init 0.
uint16_t blCrc(const uint8_t *data, size_t n) {
  uint16_t crc = 0;
  for (size_t i = 0; i < n; i++) {
    uint8_t xb = data[i];
    for (int j = 0; j < 8; j++) {
      if ((xb ^ crc) & 1) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
      xb >>= 1;
    }
  }
  return crc;
}

// Sends data (plus its CRC, low byte first, when `withCrc`) and swallows the
// echo. The receiver shares the wire with the transmitter, so every byte we
// send comes straight back; anything after that is the ESC.
bool blSend(const uint8_t *data, size_t n, bool withCrc) {
  // Turnaround: the bootloader isn't listening again the instant its last
  // byte ends. A command sent 8 us after an ACK was mangled on the wire.
  while ((uint32_t)(micros() - lastRxUs) < TURNAROUND_US) {
  }
  blDrain();
  ESC.write(data, n);
  uint8_t crc[2];
  if (withCrc) {
    uint16_t c = blCrc(data, n);
    crc[0] = c & 0xFF;
    crc[1] = c >> 8;
    ESC.write(crc, 2);
  }
  ESC.flush();

  size_t total = n + (withCrc ? 2 : 0);
  uint8_t echo[8];
  for (size_t i = 0; i < total; i++) {
    uint8_t expect = i < n ? data[i] : crc[i - n];
    if (!blReadBytes(echo, 1, 50)) {
      dbgf("send %02x: echo byte %u/%u missing\n", data[0], (unsigned)i, (unsigned)total);
      return false;
    }
    if (echo[0] != expect) {  // line driven by someone else
      dbgf("send %02x: echo byte %u/%u got %02x want %02x\n", data[0], (unsigned)i, (unsigned)total, echo[0], expect);
      return false;
    }
  }
  return true;
}

// Waits up to `timeoutMs` for one byte from the ESC; -1 if none.
int blGetAck(uint32_t timeoutMs) {
  uint8_t b;
  if (!blReadBytes(&b, 1, timeoutMs)) {
    dbgf("ack: none in %lu ms\n", (unsigned long)timeoutMs);
    return -1;
  }
  if (b != BL_ACK_OK) dbgf("ack: %02x\n", b);
  return b;
}

// The BLHeli "hello". A bootloader answers with 9 bytes: "471", its pin code,
// signature hi, signature lo, bootloader version, pages, ACK.
bool blConnect() {
  static const uint8_t hello[21] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x0D,
                                    'B', 'L', 'H', 'e', 'l', 'i', 0xF4, 0x7D};
  for (int attempt = 0; attempt < 3; attempt++) {
    if (blSend(hello, sizeof(hello), false) && blReadBytes(blBuf, 9, 100) &&
        blBuf[0] == '4' && blBuf[1] == '7' && blBuf[2] == '1') {
      deviceInfo[0] = blBuf[5];  // signature low byte
      deviceInfo[1] = blBuf[4];  // signature high byte
      deviceInfo[2] = blBuf[3];  // bootloader pin code, e.g. 0x02 = PA2
      deviceInfo[3] = interfaceMode;
      connected = true;
      dbgf("connect pin %d: ok, bl v%u\n", currentPin, blBuf[6]);
      return true;
    }
    dbgf("connect pin %d: try %d, got %02x %02x %02x\n", currentPin, attempt, blBuf[0], blBuf[1], blBuf[2]);
    delay(50);
  }
  return false;
}

bool blSetAddress(uint16_t addr) {
  uint8_t cmd[4] = {BL_CMD_SET_ADDRESS, 0, (uint8_t)(addr >> 8), (uint8_t)addr};
  return blSend(cmd, 4, true) && blGetAck(100) == BL_ACK_OK;
}

// Reads n (1..256) bytes at `addr` into out. The ESC replies with the data,
// its CRC and an ACK.
bool blRead(uint16_t addr, uint8_t *out, uint16_t n) {
  if (!blSetAddress(addr)) { dbgf("read %04x: set address failed\n", addr); return false; }
  uint8_t cmd[2] = {BL_CMD_READ_FLASH, (uint8_t)(n == 256 ? 0 : n)};
  if (!blSend(cmd, 2, true)) return false;
  if (!blReadBytes(blBuf, n + 3, 100)) { dbgf("read %04x: short reply\n", addr); return false; }
  if (blBuf[n + 2] != BL_ACK_OK) { dbgf("read %04x: ack %02x\n", addr, blBuf[n + 2]); return false; }
  if (blCrc(blBuf, n) != (uint16_t)(blBuf[n] | (blBuf[n + 1] << 8))) { dbgf("read %04x: bad crc\n", addr); return false; }
  memcpy(out, blBuf, n);
  return true;
}

// Writes n (1..256) bytes at `addr`: set address, hand over the buffer, then
// program it. The buffer-size command gets no ACK; the other two do.
bool blWrite(uint16_t addr, const uint8_t *data, uint16_t n) {
  if (!blSetAddress(addr)) { dbgf("write %04x: set address failed\n", addr); return false; }
  uint8_t cmd[4] = {BL_CMD_SET_BUFFER, 0, (uint8_t)(n == 256 ? 1 : 0), (uint8_t)n};
  if (!blSend(cmd, 4, true)) return false;
  delay(2);
  if (!blSend(data, n, true) || blGetAck(200) != BL_ACK_OK) { dbgf("write %04x: buffer not accepted\n", addr); return false; }
  uint8_t prog[2] = {BL_CMD_PROG_FLASH, 0x01};
  if (!blSend(prog, 2, true) || blGetAck(1500) != BL_ACK_OK) { dbgf("write %04x: program failed\n", addr); return false; }
  return true;
}

bool blErase(uint16_t addr) {
  if (!blSetAddress(addr)) return false;
  uint8_t cmd[2] = {BL_CMD_ERASE_FLASH, 0x01};
  return blSend(cmd, 2, true) && blGetAck(3000) == BL_ACK_OK;
}

// Keep-alive is deliberately an unknown command: a live bootloader answers it
// with BAD_CMD, which is the reply we want.
bool blKeepAlive() {
  uint8_t cmd[2] = {BL_CMD_KEEP_ALIVE, 0};
  return blSend(cmd, 2, true) && blGetAck(100) == BL_ACK_BAD_CMD;
}

// CMD_RUN with its zero CRC: four zero bytes. The ESC starts its firmware.
void blRun() {
  uint8_t cmd[2] = {BL_CMD_RUN, 0};
  blSend(cmd, 2, true);
  connected = false;
}

// =============================================================================
// USB side: MSP handshake, then 4-way commands
// =============================================================================

int usbReadByte(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (!USBSerial.available()) {
    if (millis() - start > timeoutMs) return -1;
    delay(1);
  }
  return USBSerial.read();
}

uint16_t crc16Xmodem(uint16_t crc, uint8_t b) {
  crc ^= (uint16_t)b << 8;
  for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  return crc;
}

// $M> len cmd payload checksum, or $M! for an unknown command
void mspReply(uint8_t cmd, const uint8_t *payload, uint8_t len, bool error) {
  uint8_t hdr[5] = {'$', 'M', (uint8_t)(error ? '!' : '>'), len, cmd};
  uint8_t sum = len ^ cmd;
  for (uint8_t i = 0; i < len; i++) sum ^= payload[i];
  USBSerial.write(hdr, 5);
  if (len) USBSerial.write(payload, len);
  USBSerial.write(sum);
  USBSerial.flush();
}

// The '$' has been read. Request: $M< len cmd payload checksum.
void handleMsp() {
  if (usbReadByte(100) != 'M' || usbReadByte(100) != '<') return;
  int len = usbReadByte(100), cmd = usbReadByte(100);
  if (len < 0 || cmd < 0) return;
  uint8_t sum = len ^ cmd;
  for (int i = 0; i < len; i++) {
    int b = usbReadByte(100);
    if (b < 0) return;
    if (i < (int)sizeof(inParams)) inParams[i] = b;
    sum ^= b;
  }
  if (usbReadByte(100) != sum) return;

  switch (cmd) {
    case MSP_API_VERSION: {
      uint8_t p[3] = {0, 1, 46};  // MSP protocol 0, API 1.46
      mspReply(cmd, p, 3, false);
      break;
    }
    case MSP_FC_VARIANT:
      mspReply(cmd, (const uint8_t *)"BTFL", 4, false);
      break;
    case MSP_BATTERY_STATE: {
      // cells, capacity, voltage (0.1 V), mAh drawn, amps (0.01 A), state, voltage (0.01 V)
      uint8_t p[11] = {0};
      mspReply(cmd, p, 11, false);
      break;
    }
    case MSP_MOTOR_CONFIG: {
      // min throttle, max throttle, min command, motor count, poles, dshot telemetry, esc sensor
      uint8_t p[10] = {0xB8, 0x04, 0xD0, 0x07, 0xE8, 0x03, ESC_COUNT, 14, 0, 0};
      mspReply(cmd, p, 10, false);
      break;
    }
    case MSP_MOTOR: {
      // one 16-bit value per motor; non-zero means the motor exists
      uint8_t p[16] = {0};
      for (int i = 0; i < ESC_COUNT && i < 8; i++) {
        p[i * 2] = 1000 & 0xFF;
        p[i * 2 + 1] = 1000 >> 8;
      }
      mspReply(cmd, p, 16, false);
      break;
    }
    case MSP_SET_PASSTHROUGH: {
      // Reply with the ESC count; the configurator then switches to 4-way
      // commands, which we accept at any time.
      uint8_t p[1] = {ESC_COUNT};
      mspReply(cmd, p, 1, false);
      break;
    }
    case 0xF0: {  // private: return and clear the debug log
      uint8_t n = dbgLen > 255 ? 255 : dbgLen;
      mspReply(cmd, (const uint8_t *)dbg, n, false);
      memmove(dbg, dbg + n, dbgLen - n);
      dbgLen -= n;
      break;
    }
    default:
      mspReply(cmd, nullptr, 0, true);
      break;
  }
}

// . cmd addrH addrL len params ack crcH crcL
void fourWayReply(uint8_t cmd, uint16_t addr, const uint8_t *params, uint16_t len, uint8_t ack) {
  uint8_t hdr[5] = {CMD_REMOTE_ESCAPE, cmd, (uint8_t)(addr >> 8), (uint8_t)addr,
                    (uint8_t)(len == 256 ? 0 : len)};
  uint16_t crc = 0;
  for (int i = 0; i < 5; i++) crc = crc16Xmodem(crc, hdr[i]);
  for (uint16_t i = 0; i < len; i++) crc = crc16Xmodem(crc, params[i]);
  crc = crc16Xmodem(crc, ack);
  uint8_t tail[3] = {ack, (uint8_t)(crc >> 8), (uint8_t)crc};
  USBSerial.write(hdr, 5);
  USBSerial.write(params, len);
  USBSerial.write(tail, 3);
  USBSerial.flush();
}

// The '/' has been read. Request: / cmd addrH addrL len params crcH crcL.
void handleFourWay() {
  uint8_t hdr[4];
  uint16_t crc = crc16Xmodem(0, CMD_LOCAL_ESCAPE);
  for (int i = 0; i < 4; i++) {
    int b = usbReadByte(100);
    if (b < 0) return;
    hdr[i] = b;
    crc = crc16Xmodem(crc, b);
  }
  uint8_t cmd = hdr[0];
  uint16_t addr = (hdr[1] << 8) | hdr[2];
  uint16_t len = hdr[3] ? hdr[3] : 256;
  for (uint16_t i = 0; i < len; i++) {
    int b = usbReadByte(100);
    if (b < 0) return;
    inParams[i] = b;
    crc = crc16Xmodem(crc, b);
  }
  int crcHi = usbReadByte(100), crcLo = usbReadByte(100);
  if (crcHi < 0 || crcLo < 0) return;

  uint8_t ack = ACK_OK;
  const uint8_t *out = outParams;
  uint16_t outLen = 1;
  outParams[0] = 0;

  if (((crcHi << 8) | crcLo) != crc) {
    fourWayReply(cmd, addr, out, outLen, ACK_I_INVALID_CRC);
    return;
  }

  switch (cmd) {
    case CMD_INTERFACE_TEST_ALIVE:
      if (connected && !blKeepAlive()) {
        connected = false;
        ack = ACK_D_GENERAL_ERROR;
      }
      break;

    case CMD_PROTOCOL_GET_VERSION:
      outParams[0] = PROTOCOL_VERSION;
      break;

    case CMD_INTERFACE_GET_NAME:
      out = (const uint8_t *)INTERFACE_NAME;
      outLen = strlen(INTERFACE_NAME);
      break;

    case CMD_INTERFACE_GET_VERSION:
      outParams[0] = INTERFACE_VERSION_HI;
      outParams[1] = INTERFACE_VERSION_LO;
      outLen = 2;
      break;

    case CMD_INTERFACE_EXIT:
      connected = false;
      break;

    case CMD_INTERFACE_SET_MODE:
      if (inParams[0] >= IM_SIL_BLB && inParams[0] <= IM_ARM_BLB) interfaceMode = inParams[0];
      else ack = ACK_I_INVALID_PARAM;
      break;

    case CMD_DEVICE_INIT_FLASH:
      connected = false;
      if (inParams[0] >= ESC_COUNT) {
        ack = ACK_I_INVALID_CHANNEL;
        break;
      }
      selectedEsc = inParams[0];
      selectPin(ESC_PINS[selectedEsc]);
      memset(deviceInfo, 0, sizeof(deviceInfo));
      if (!blConnect()) ack = ACK_D_GENERAL_ERROR;
      out = deviceInfo;
      outLen = 4;
      break;

    case CMD_DEVICE_RESET:
      if (inParams[0] >= ESC_COUNT) {
        ack = ACK_I_INVALID_CHANNEL;
        break;
      }
      selectedEsc = inParams[0];
      selectPin(ESC_PINS[selectedEsc]);
      blRun();
      if ((addr & 0xFF) == 1) pulseLow(300);  // betaflight: address 1 = reboot it too
      break;

    case CMD_DEVICE_READ: {
      uint16_t n = inParams[0] ? inParams[0] : 256;
      if (!connected || !blRead(addr, outParams, n)) ack = ACK_D_GENERAL_ERROR;
      else outLen = n;
      break;
    }

    case CMD_DEVICE_WRITE:
      if (!connected || !blWrite(addr, inParams, len)) ack = ACK_D_GENERAL_ERROR;
      break;

    case CMD_DEVICE_PAGE_ERASE:
      // ARM: page * 1024
      if (!connected || !blErase((uint16_t)inParams[0] << 10)) ack = ACK_D_GENERAL_ERROR;
      break;

    case CMD_DEVICE_ERASE_ALL:
    case CMD_DEVICE_C2CK_LOW:
    case CMD_DEVICE_READ_EEPROM:
    case CMD_DEVICE_WRITE_EEPROM:
    case CMD_DEVICE_VERIFY:
    default:
      ack = ACK_I_INVALID_CMD;
      break;
  }

  fourWayReply(cmd, addr, out, outLen, ack);
}

void setup() {
  // Leave every ESC line alone until the configurator asks for it: as inputs
  // the external pull-ups hold them HIGH, which keeps AM32 in its bootloader.
  for (uint8_t i = 0; i < ESC_COUNT; i++) {
    gpio_set_direction((gpio_num_t)ESC_PINS[i], GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)ESC_PINS[i], GPIO_PULLUP_ONLY);
  }

  // Present the spoofed identity before starting USB - VID/PID must be set
  // before begin() or the descriptor is already published.
  USB.VID(SPOOF_VID);
  USB.PID(SPOOF_PID);
  USB.productName("AM32 4-way Linker");
  USB.manufacturerName("AM32");
  USBSerial.setRxBufferSize(1024);  // a 256-byte write packet is 263 bytes
  USBSerial.begin();
  USB.begin();
}

void loop() {
  int b = usbReadByte(10);
  if (b == CMD_LOCAL_ESCAPE) handleFourWay();
  else if (b == '$') handleMsp();
}
