// Remote control of the Bruce CLI over LoRa (Reyax RYLR998 on a UART).
//
// A background task speaks the link protocol (see lora_link_engine.h) to a controller and bridges
// it to the normal serial CLI: commands arriving over LoRa run exactly like commands typed on USB,
// and whatever they print is sent back over the air.  USB keeps working at the same time.
#ifndef BRUCE_LORA_LINK_H
#define BRUCE_LORA_LINK_H

#if defined(USE_RYLR998_VIA_UART) && !defined(LITE_VERSION)
#include <Arduino.h>

struct LoRaLinkStatus {
    bool running = false;   // task alive
    bool moduleOk = false;  // the RYLR998 answered its AT handshake
    bool linked = false;    // a controller session is active
    bool switching = false; // a range/speed change is being negotiated
    uint8_t profile = 0;    // radio profile in use (0 = Range .. 5 = Turbo)
    const char *profileName = "";
    int rssi = 0, snr = 0;  // of the last frame from the controller
    uint32_t lastRxAgoMs = 0xFFFFFFFF;
    uint32_t framesRx = 0, framesTx = 0, badFrames = 0, bytesIn = 0, bytesOut = 0, sessions = 0;
    uint8_t want = 0xF;     // slider: 0..5 = fixed profile, 0xF = leave it to the controller
    uint8_t power = 22;     // configured maximum TX power, dBm
    uint8_t powerNow = 22;  // power in use right now (the controller lowers it when the link is very strong)
};

void loraLinkBegin();               // start the link task (idempotent)
void loraLinkStop();                // stop it and release the UART (idempotent)
bool loraLinkRunning();
bool loraLinkRemoteActive();        // the CLI command being run came in over LoRa
LoRaLinkStatus loraLinkStatus();
void loraLinkSliderMenu();          // "Range / Speed" screen
String loraLinkDebug();             // multi-line status for the `lora` CLI command
void loraLinkResetModule();         // ask the link task to reset and re-initialise the radio module

#endif
#endif
