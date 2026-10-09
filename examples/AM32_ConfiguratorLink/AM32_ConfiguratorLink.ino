/*
  AM32_ConfiguratorLink - read and write AM32 ESC settings over the DShot wire

  Turns the ESP32-S3 into an AM32 "Direct Connect" USB linker so the am32.ca web
  configurator can talk to the ESC through the same single signal wire the rest
  of this library drives with DShot. No adapter, no rewiring - flash this sketch
  when you want to change settings, flash your DShot sketch when you want to run
  the motor.

  It bridges the USB CDC port to a half-duplex 19200 8N1 UART on one open-drain
  pin, and spoofs a USB VID/PID from am32.ca's direct-connect list so the web
  configurator lists the port. The receiver hearing its own transmission gives
  the one-wire echo the configurator expects from a real Direct Connect adapter.

  One ESC per flash. For several ESCs at once, see AM32_FourWayLink.

  ---------------------------------------------------------------------------
  REQUIRED BOARD SETTINGS (both, or it will not build):
    Tools > USB Mode         : "USB-OTG (TinyUSB)"
    Tools > USB CDC On Boot  : "Disabled"
  ---------------------------------------------------------------------------
  WIRING - identical to the DShot wiring, see README.md:

                            3V3
                             |
                            [ ] 1 kO pull-up
                             |
    GPIO 8 ----[ 100 O ]-----+----------------------- ESC signal
                             .
    GND ---------------------.----------------------- ESC ground

    The ESC runs from its own battery. Ground must be shared.
  ---------------------------------------------------------------------------
  HOW TO CONNECT:
    1. Leave the ESC UNPOWERED.
    2. Click Connect in the am32.ca configurator and pick this port.
    3. Now apply the ESC battery.

  Why the order matters: AM32's bootloader samples this line for ~45 ms at
  power-up and jumps to the motor firmware the moment it reads LOW. The pull-up
  and an idle UART hold it HIGH, so it waits for the configurator instead.
  Never drive this pin low before the ESC powers up - the opposite of what the
  DShot side needs, so do NOT call AlfredoDShot::releaseBootloader() here.

  A failed connect usually means the configurator was mid-handshake during
  those 45 ms and the ESC jumped to the motor firmware; drop the battery and
  retry. An ESC already stuck in its bootloader (what happens if you reset the
  ESP while a DShot sketch runs) can be connected to with no power cycle.
  ---------------------------------------------------------------------------
  BOARDS THAT POWER THE ESP AND THE ESC TOGETHER (Rotini V4):
    You can't apply the ESC's battery after clicking Connect, so set
    ENTER_BOOTLOADER below. On every boot the sketch then sends 4 s of
    zero-throttle DShot and stops. AM32 arms on the frames, reboots ~0.5 s after
    they stop, finds the line held HIGH and waits in its bootloader.
      1. Battery in, flash or reset this sketch, wait ~7 s.
      2. Click Connect in am32.ca and pick this port.
    To reconnect after the configurator has restarted the ESC, press RESET.
*/

#include <AlfredoDShot.h>
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

const int PIN_ESC = 4;          // same pin your DShot sketch uses
const long ESC_BAUD = 19200;    // AM32 / BLHeli one-wire rate, 8N1

// Put an already-powered ESC into its bootloader at startup. See the header.
const bool ENTER_BOOTLOADER = true;

const uint16_t SPOOF_VID = 0x26BA;  // on am32.ca's direct-connect VID list
const uint16_t SPOOF_PID = 0x0001;  // any value except 0xE204

USBCDC USBSerial;    // PC-facing TinyUSB CDC port
HardwareSerial ESC(1);  // UART1 -> ESC signal wire

static void configureOneWire() {
  // Pass the pin as both RX and TX so the UART's receiver actually gets
  // enabled - rx = -1 leaves it off, and we need to hear the ESC.
  ESC.begin(ESC_BAUD, SERIAL_8N1, PIN_ESC, PIN_ESC);

  // Open-drain with a pull-up: the ESP only ever pulls the line LOW and lets
  // the resistor drive it HIGH, so it never fights the ESC's replies.
  //
  // ORDER BELOW IS LOAD-BEARING. gpio_set_direction() internally calls
  // gpio_hal_matrix_out_default(), which detaches the UART TX signal from the
  // pad and leaves it a plain GPIO - the pin goes dead silent. The two
  // esp_rom_gpio_connect_*_signal() calls put UART1 back on the pad and must
  // come afterwards. Do not reorder these, and do not swap in pinMode().
  gpio_set_direction((gpio_num_t)PIN_ESC, GPIO_MODE_INPUT_OUTPUT_OD);
  gpio_set_pull_mode((gpio_num_t)PIN_ESC, GPIO_PULLUP_ONLY);

  esp_rom_gpio_connect_out_signal((gpio_num_t)PIN_ESC, U1TXD_OUT_IDX, false, false);
  esp_rom_gpio_connect_in_signal((gpio_num_t)PIN_ESC, U1RXD_IN_IDX, false);
}

// Sends zero-throttle DShot long enough for AM32 to arm, then goes quiet with
// the line released. AM32 reboots when the signal stops (0.5 s armed, 2 s
// unarmed), and its bootloader stays put while the pull-up holds the line
// HIGH. An ESC that was already in its bootloader just stays there.
static void enterBootloader() {
  AlfredoDShot esc;
  if (!esc.begin(PIN_ESC, DSHOT300, true)) return;

  uint32_t start = millis();
  uint32_t next = micros();
  while (millis() - start < 4000) {
    if ((int32_t)(micros() - next) < 0) continue;
    next += 1000;
    esc.send(0);
  }
  esc.end();

  // Float the pad so nothing can hold the line low while the ESC reboots.
  gpio_set_direction((gpio_num_t)PIN_ESC, GPIO_MODE_INPUT);
  delay(2500);
}

void setup() {
  if (ENTER_BOOTLOADER) enterBootloader();
  configureOneWire();

  // Present the spoofed identity before starting USB - VID/PID must be set
  // before begin() or the descriptor is already published.
  USB.VID(SPOOF_VID);
  USB.PID(SPOOF_PID);
  USB.productName("AM32 1-Wire Linker");
  USB.manufacturerName("AM32");
  USBSerial.begin();
  USB.begin();
}

void loop() {
  // PC -> ESC. The shared wire means we also read these bytes straight back,
  // which is exactly the one-wire echo the configurator is looking for.
  while (USBSerial.available()) ESC.write((uint8_t)USBSerial.read());

  // ESC wire -> PC: our echoed bytes, followed by the ESC's replies.
  while (ESC.available()) USBSerial.write((uint8_t)ESC.read());
}
