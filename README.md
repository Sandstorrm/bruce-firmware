# Bruce firmware — RYLR998 LoRa remote-control fork

Fork of **[BruceDevices/firmware](https://github.com/BruceDevices/firmware)** ([wiki](https://wiki.bruce.computer/),
[flasher](https://bruce.computer/flasher), [Discord](https://discord.gg/WJ9XF9czVT)). For what Bruce is, the supported
boards, features and installation, see the original project. Everything not listed below is unchanged from upstream.

Branch: `feat/rylr998-lora-uart`. Target board: **M5Stack Core2** with a Reyax **RYLR998** LoRa module on Grove Port A.

## What this fork changes

**RYLR998 support (UART / AT commands)**
- New LoRa backend for boards with no SPI LoRa radio, enabled by `-DUSE_RYLR998_VIA_UART`. LoRa Chat works with it; the
  SX1276/SX1262 code paths are untouched.
- Core2 wiring: G32 (TX) → module RXD, G33 (RX) → module TXD, 115200 baud. Pins are `LORA_RYLR998_RX_PIN` /
  `LORA_RYLR998_TX_PIN` in `boards/m5stack-core2/m5stack-core2.ini`.

**Bruce's CLI over LoRa (`src/modules/lora/`)**
- `lora_link_engine`: a reliable link on top of the RYLR998. CRC-16, stop-and-wait ARQ with resends, session resume after
  a dropout, and automatic re-pairing after either side changes profile. Plain C++ with no hardware dependencies, so
  it is tested off-device.
- `LoRaLink`: starts at boot and runs the link on core 0. The CLI is served on USB and LoRa at once, and output comes
  back over the air.
- Six range/speed profiles (Range → Turbo), switched from the controller or on the device under **LoRa → Range / Speed**.
- Automatic TX power control, so two radios close together don't overload each other. `-DLORA_LINK_TX_POWER=N` sets the
  maximum (default 22).
- Module watchdogs: reset at boot, settings drift check, stuck-transmit and deaf-while-linked detection, with a crash
  and reset-reason log kept across reboots.

**New and changed CLI commands**
- `lora`, `lora reset`, `lora restart`: link status, module conversation and crash history; kick the module or the link.
- `screen view` (`open N`, `back`, `sel`, `next`, `prev`): the current menu or app screen as one line of JSON, so it can be
  read and driven without seeing the display. It reads the text drawn on the display, so apps that draw only graphics
  show nothing.
- `options` with no or a non-numeric argument now lists the entries instead of selecting entry 0.
- `loader open` on a main-menu entry is handed to the UI task when run remotely, so it no longer blocks the CLI task.
- `nav` and `options` feedback goes to whichever device issued the command (USB or LoRa).
- `display dump` no longer crashes the device (heap buffer, sent in small chunks).

**Core changes that support the above**
- `tftLogger` keeps a text shadow of what is on screen and a redraw counter (`getScreenText()`, `getScreenGen()`).
- `uiState` global, set while a menu is being shown, so remote commands can wait for the UI.
- `USBSerial::vprintf` bounded with `vsnprintf`.

## Building

```sh
pio run -e m5stack-core2 -t upload
```

## Companion controller

A terminal UI for the Mac (Textual): the Core2's CLI with output feedback, a range/speed slider, power control, and a
keyboard-driven screen mode that shows the Core2's menus and apps. It lives in `../lora-tui` here and isn't published
yet. The wire protocol is documented in its `PROTOCOL.md`.

## License

Same as upstream: see [LICENSE](LICENSE).
