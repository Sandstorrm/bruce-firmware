#if !defined(LITE_VERSION)
#include "LoRaRF.h"
#include "WString.h"
#include "core/bus_HAL.h"
#include "core/config.h"
#include "core/configPins.h"
#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>
#include <RadioLib.h>
#include <core/display.h>
#include <core/mykeyboard.h>
#include <core/utils.h>
#include <globals.h>
#include <vector>

extern BruceConfigPins bruceConfigPins;

// RYLR998 is a UART/AT-command LoRa module (as opposed to the SPI-attached SX1276/SX1262
// radios below). It is only wired up as the active backend when a board both opts in
// (USE_RYLR998_VIA_UART) and has no SPI LoRa radio of its own (LORA_SCK unset) -- e.g. the
// M5Stack Core2, which exposes the RYLR998 over its Grove UART port. Boards that already
// have an SPI radio keep their existing SX1276/SX1262 selection UI untouched.
#if defined(USE_RYLR998_VIA_UART) && (LORA_SCK < 0)
#define LORA_UART_ONLY_BACKEND 1
#endif

bool update = false;
String msg;
String rcvmsg;
String displayName;
bool intlora = true;
// scrolling thing
std::vector<String> messages;
int scrollOffset = 0;
const int maxMessages = 19;

#define spreadingFactor 9
#define SignalBandwidth 31.25E3
#define codingRateDenominator 8
#define preambleLength 8

int contentWidth = tftWidth - 20;
int yStart = 35;
int yPos = yStart;
int ySpacing = 10;
int rightColumnX = tftWidth / 2 + 10;
SPIClass *loraSpi = nullptr;
Module *loraModule = nullptr;
SX1276 *lora1276 = nullptr;
SX1262 *lora1262 = nullptr;
volatile bool loraPacketReceived = false;
volatile bool loraInterruptEnabled = true;
enum class LoRaRadioVariant { SX1276, SX1262, RYLR998 };
#if defined(LORA_UART_ONLY_BACKEND)
LoRaRadioVariant loraRadioVariant = LoRaRadioVariant::RYLR998;
#else
LoRaRadioVariant loraRadioVariant = LoRaRadioVariant::SX1276;
#endif

#if defined(USE_RYLR998_VIA_UART)
// ---- RYLR998 UART/AT-command backend -------------------------------------------------
// Talks to the Reyax RYLR998 over a plain HardwareSerial using its AT command set instead
// of RadioLib/SPI. It is kept self-contained here and only wired into the shared
// startLoraRadio()/sendLoraMessage()/reciveMessage()/clearLoraRadio() entry points used by
// the rest of lorachat(), so the SPI radios below are completely unaffected.
#define RYLR998_BAUD 115200
#define RYLR998_ADDRESS 0    // Every chat participant uses the same address so AT+SEND
                             // reaches every other RYLR998 listening on the same frequency,
                             // mirroring the "anyone in range hears it" behavior of the
                             // SPI radios below.
#define RYLR998_AT_TIMEOUT 2000

HardwareSerial *rylrSerial = nullptr;
String rylrLineBuffer;

void handleRylrReceivedLine(const String &line);

// Drains whatever bytes are currently available. Any complete "+RCV=..." line is treated
// as an inbound chat message (this is what plays the role of reciveMessage() for this
// backend). When waitForResponse is true, blocks (up to timeoutMs) for the next non-+RCV
// line, which is the reply to an AT command just sent.
String rylrPollLines(uint32_t timeoutMs, bool waitForResponse) {
    if (!rylrSerial) return "";
    String lastResponse;
    unsigned long start = millis();
    while (true) {
        while (rylrSerial->available()) {
            char c = (char)rylrSerial->read();
            if (c == '\n') {
                rylrLineBuffer.trim();
                if (rylrLineBuffer.length()) {
                    if (rylrLineBuffer.startsWith("+RCV=")) {
                        handleRylrReceivedLine(rylrLineBuffer);
                    } else {
                        lastResponse = rylrLineBuffer;
                    }
                }
                rylrLineBuffer = "";
            } else if (c != '\r') {
                rylrLineBuffer += c;
            }
        }
        if (!waitForResponse || lastResponse.length() || (millis() - start > timeoutMs)) return lastResponse;
    }
}

bool rylrSendCommand(const String &cmd, uint32_t timeoutMs = RYLR998_AT_TIMEOUT) {
    if (!rylrSerial) return false;
    rylrSerial->print(cmd);
    rylrSerial->print("\r\n");
    String resp = rylrPollLines(timeoutMs, true);
    return resp.startsWith("+OK");
}

// Parses "+RCV=<address>,<length>,<data>,<rssi>,<snr>" and feeds <data> into the same
// history/UI plumbing (messages vector, /chats.txt, update flag) that reciveMessage() uses
// for the SPI radios, so LoRa Chat behaves identically regardless of backend.
void handleRylrReceivedLine(const String &line) {
    int firstComma = line.indexOf(',');
    int secondComma = firstComma < 0 ? -1 : line.indexOf(',', firstComma + 1);
    if (firstComma < 0 || secondComma < 0) return;
    int len = line.substring(firstComma + 1, secondComma).toInt();
    if (len <= 0 || secondComma + 1 + len > (int)line.length()) return;
    // Slice exactly <len> bytes for the payload (rather than splitting on ',') so a message
    // containing commas is not truncated.
    String data = line.substring(secondComma + 1, secondComma + 1 + len);

    rcvmsg = data;
    Serial.println("Recived:" + rcvmsg);
    File file = LittleFS.open("/chats.txt", "a");
    file.println(rcvmsg);
    file.close();
    messages.push_back(rcvmsg);
    if (messages.size() > maxMessages) { scrollOffset = messages.size() - maxMessages; }
    update = true;
}

bool startRylrRadio(float bandMHz) {
    if (bruceConfigPins.LoRa_uart_bus.rx == GPIO_NUM_NC || bruceConfigPins.LoRa_uart_bus.tx == GPIO_NUM_NC) {
        Serial.println("RYLR998 UART pins not configured!");
        displayError("RYLR998 pins not configured!", true);
        return false;
    }

    rylrSerial = new HardwareSerial(2);
    rylrSerial->begin(
        RYLR998_BAUD, SERIAL_8N1, bruceConfigPins.LoRa_uart_bus.rx, bruceConfigPins.LoRa_uart_bus.tx
    );
    delay(100);
    rylrLineBuffer = "";
    while (rylrSerial->available()) rylrSerial->read();

    if (!rylrSendCommand("AT")) {
        Serial.println("RYLR998 not responding");
        displayError("RYLR998 not found", true);
        return false;
    }

    uint32_t bandHz = (uint32_t)(bandMHz * 1000000.0f + 0.5f);
    if (!rylrSendCommand("AT+BAND=" + String(bandHz))) {
        Serial.println("RYLR998 failed to set frequency");
        displayError("LoRa Init Failed", true);
        return false;
    }

    // Best-effort modulation setup: reuses the spreadingFactor/codingRateDenominator
    // constants above where their meaning maps directly onto the RYLR998's AT+PARAMETER
    // fields (coding rate N/(N+4) -> codingRateDenominator-4). Bandwidth/preamble codes
    // differ across Reyax firmware revisions, so a conservative, widely-documented 125 kHz
    // bandwidth and the standard preamble of 4 are used instead of trying to reproduce the
    // SPI radios' 31.25 kHz setting exactly. This call is intentionally non-fatal: as long
    // as both ends of the chat run the same firmware defaults they can still talk even if a
    // particular device rejects one of these values, so failure here only logs a warning
    // instead of aborting init (unlike the AT/AT+BAND checks above, which every RYLR998
    // firmware revision is documented to support).
    if (!rylrSendCommand("AT+PARAMETER=" + String(spreadingFactor) + ",7," +
                         String(codingRateDenominator - 4) + ",4")) {
        Serial.println("RYLR998: AT+PARAMETER not applied, continuing with current/default radio parameters");
    }

    if (!rylrSendCommand("AT+ADDRESS=" + String(RYLR998_ADDRESS))) {
        Serial.println("RYLR998: AT+ADDRESS not applied, continuing with current/default address");
    }

    Serial.println("RYLR998 Started");
    return true;
}
#endif

int getLoraIrqPin() {
#ifdef LORA_IRQ
    return LORA_IRQ;
#else
    return bruceConfigPins.LoRa_bus.io2;
#endif
}

int getLoraBusyPin() {
#ifdef LORA_BUSY
    return LORA_BUSY;
#else
    return GPIO_NUM_NC;
#endif
}

int getLoraResetPin() { return bruceConfigPins.LoRa_bus.io0; }
int getLoraCsPin() { return bruceConfigPins.LoRa_bus.cs; }

void clearLoraRadio() {
    if (lora1276) {
        delete lora1276;
        lora1276 = nullptr;
    }
    if (lora1262) {
        delete lora1262;
        lora1262 = nullptr;
    }
    if (loraModule) {
        delete loraModule;
        loraModule = nullptr;
    }
#if defined(USE_RYLR998_VIA_UART)
    if (rylrSerial) {
        rylrSerial->end();
        delete rylrSerial;
        rylrSerial = nullptr;
    }
#endif
}

void onLoraPacket() {
    if (!loraInterruptEnabled) return;
    loraPacketReceived = true;
}

SPIClass *selectLoraSPIBus() {
    SPIClass *bus =
        acquireSPIBus(bruceConfigPins.LoRa_bus.sck, bruceConfigPins.LoRa_bus.miso, bruceConfigPins.LoRa_bus.mosi);
    if (!bus) {
        Serial.println("No hardware SPI bus available for LoRa, falling back to default SPI");
        return &SPI;
    }
    return bus;
}

bool startLoraRadio(float bandMHz) {
    intlora = false;
    loraPacketReceived = false;
    loraInterruptEnabled = true;

#if defined(USE_RYLR998_VIA_UART)
    if (loraRadioVariant == LoRaRadioVariant::RYLR998) {
        clearLoraRadio();
        if (!startRylrRadio(bandMHz)) {
            clearLoraRadio();
            return false;
        }
        intlora = true;
        Serial.println("LoRa Started");
        return true;
    }
#endif

    const int irqPin = getLoraIrqPin();
    if (getLoraCsPin() == GPIO_NUM_NC || bruceConfigPins.LoRa_bus.mosi == GPIO_NUM_NC ||
        bruceConfigPins.LoRa_bus.miso == GPIO_NUM_NC || bruceConfigPins.LoRa_bus.sck == GPIO_NUM_NC) {
        Serial.println("LoRa pins not configured!");
        displayError("LoRa pins not configured!", true);
        return false;
    }
    if (irqPin == GPIO_NUM_NC) {
        Serial.println("LoRa IRQ pin not configured!");
        displayError("LoRa IRQ pin not configured!", true);
        return false;
    }

    loraSpi = selectLoraSPIBus();
    clearLoraRadio();
    const int busyPin = (loraRadioVariant == LoRaRadioVariant::SX1262) ? getLoraBusyPin() : GPIO_NUM_NC;
    if (loraRadioVariant == LoRaRadioVariant::SX1262 && busyPin == GPIO_NUM_NC) {
        Serial.println("Warning: SX1262 selected but BUSY pin is not configured");
    }
    loraModule = new Module(getLoraCsPin(), irqPin, getLoraResetPin(), busyPin, *loraSpi);

    int state = RADIOLIB_ERR_NONE;
    if (loraRadioVariant == LoRaRadioVariant::SX1276) {
        lora1276 = new SX1276(loraModule);
        state = lora1276->begin(bandMHz);
        if (state == RADIOLIB_ERR_NONE) { lora1276->setDio0Action(onLoraPacket, CHANGE); }
        if (state == RADIOLIB_ERR_NONE) state = lora1276->setSpreadingFactor(spreadingFactor);
        if (state == RADIOLIB_ERR_NONE) state = lora1276->setBandwidth(SignalBandwidth / 1000.0);
        if (state == RADIOLIB_ERR_NONE) state = lora1276->setCodingRate(codingRateDenominator);
        if (state == RADIOLIB_ERR_NONE) state = lora1276->setPreambleLength(preambleLength);
        if (state == RADIOLIB_ERR_NONE) state = lora1276->startReceive();
    } else {
        lora1262 = new SX1262(loraModule);
        state = lora1262->begin(bandMHz);
        if (state == RADIOLIB_ERR_NONE) { lora1262->setDio1Action(onLoraPacket); }
        if (state == RADIOLIB_ERR_NONE) state = lora1262->setSpreadingFactor(spreadingFactor);
        if (state == RADIOLIB_ERR_NONE) state = lora1262->setBandwidth(SignalBandwidth / 1000.0);
        if (state == RADIOLIB_ERR_NONE) state = lora1262->setCodingRate(codingRateDenominator);
        if (state == RADIOLIB_ERR_NONE) state = lora1262->setPreambleLength(preambleLength);
        if (state == RADIOLIB_ERR_NONE) state = lora1262->startReceive();
    }

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("Starting LoRa failed! Err %d\n", state);
        displayError("LoRa Init Failed", true);
        clearLoraRadio();
        return false;
    }
    intlora = true;
    Serial.println("LoRa Started");
    return true;
}

bool sendLoraMessage(String &payload) {
    if (!intlora) return false;
    loraInterruptEnabled = false;
#if defined(USE_RYLR998_VIA_UART)
    if (loraRadioVariant == LoRaRadioVariant::RYLR998) {
        bool ok = rylrSendCommand(
            "AT+SEND=" + String(RYLR998_ADDRESS) + "," + String(payload.length()) + "," + payload, 3000
        );
        loraInterruptEnabled = true;
        if (!ok) {
            Serial.println("RYLR998 transmit failed");
            displayError("LoRa send failed");
            return false;
        }
        return true;
    }
#endif
    int state = RADIOLIB_ERR_NONE;
    if (loraRadioVariant == LoRaRadioVariant::SX1276 && lora1276) {
        state = lora1276->transmit(payload);
        lora1276->startReceive();
    } else if (loraRadioVariant == LoRaRadioVariant::SX1262 && lora1262) {
        state = lora1262->transmit(payload);
        lora1262->startReceive();
    } else {
        loraInterruptEnabled = true;
        return false;
    }
    loraInterruptEnabled = true;
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("LoRa transmit failed: %d\n", state);
        displayError("LoRa send failed");
        return false;
    }
    return true;
}

void reciveMessage() {
#if defined(USE_RYLR998_VIA_UART)
    if (loraRadioVariant == LoRaRadioVariant::RYLR998) {
        if (!intlora) return;
        rylrPollLines(0, false); // non-blocking drain; any "+RCV=" line updates messages/rcvmsg
        return;
    }
#endif
    if (!loraPacketReceived || !intlora) return;
    loraInterruptEnabled = false;
    loraPacketReceived = false;
    String incoming;
    int state = (loraRadioVariant == LoRaRadioVariant::SX1262 && lora1262)
                    ? lora1262->readData(incoming)
                    : (lora1276 ? lora1276->readData(incoming) : -1);
    if (state == RADIOLIB_ERR_NONE) {
        rcvmsg = incoming;
        Serial.println("Recived:" + rcvmsg);
        File file = LittleFS.open("/chats.txt", "a");
        file.println(rcvmsg);
        file.close();
        messages.push_back(rcvmsg);
        if (messages.size() > maxMessages) { scrollOffset = messages.size() - maxMessages; }
        update = true;
    } else {
        Serial.printf("LoRa read failed: %d\n", state);
    }
    if (loraRadioVariant == LoRaRadioVariant::SX1262 && lora1262) {
        lora1262->startReceive();
    } else if (lora1276) {
        lora1276->startReceive();
    }
    loraInterruptEnabled = true;
}

// render stuff

void render() {
    if (!update) return;
    tft.setTextSize(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(0x6DFC);
    if (!intlora) { tft.drawString("Lora Init Failed", 10, 13); }
    Serial.println(String(displayName));
    tft.drawString("USRN: " + String(displayName), 10, 25);

    int yPos = yStart;
    int endLine = scrollOffset + maxMessages;
    if (endLine > messages.size()) endLine = messages.size();
    for (int i = scrollOffset; i < endLine; i++) {
        tft.setTextColor(bruceConfig.priColor);
        tft.drawString(messages[i], 10, yPos);
        yPos += ySpacing;
    }
    update = false;
}

void loadMessages() {
    messages.clear();
    File file = LittleFS.open("/chats.txt", "r");
    while (file.available()) {
        String line = file.readStringUntil('\n');
        messages.push_back(line);
    }
    file.close();
    if (messages.size() > maxMessages) {
        scrollOffset = messages.size() - maxMessages;
    } else {
        scrollOffset = 0;
    }
}

// optional call funcs
void sendmsg() {
    Serial.println("C bttn");
    tft.fillScreen(TFT_BLACK);
    if (!intlora) {
        tft.setTextColor(bruceConfig.priColor);

        tft.setTextColor(TFT_RED);
        tft.setTextSize(2);
        tft.setCursor(10, tftHeight / 2 - 10);
        tft.print("LoRa not init!");

        tft.drawCentreString("LoRa not initialized!", tftWidth / 2, tftHeight / 2, 2);
        delay(1500);
        update = true;
        return;
    }
    msg = keyboard(msg, 256, "Message:");
    if (msg == "\x1B") return;
    msg = String(displayName) + ": " + msg;
    if (msg == "") return;
    Serial.println(msg);
    if (!sendLoraMessage(msg)) {
        update = true;
        return;
    }
    tft.fillScreen(TFT_BLACK);
    update = true;
    File file = LittleFS.open("/chats.txt", "a");
    file.println(msg);
    file.close();

    messages.push_back(msg);
    if (messages.size() > maxMessages) { scrollOffset = messages.size() - maxMessages; }
    msg = "";
}

void upress() {
    Serial.println("Up Pressed");
    if (scrollOffset > 0) {
        scrollOffset--;
        update = true;
    }
}

void downpress() {
    Serial.println("Down Pressed");
    if (scrollOffset < messages.size() - maxMessages) {
        scrollOffset++;
        update = true;
    }
}

void selectRadioVariant(JsonDocument &doc) {
#if defined(LORA_UART_ONLY_BACKEND)
    // This board only has the RYLR998 UART radio (no SPI LoRa hardware), so there is
    // nothing to choose: skip the SX1276/SX1262 picker entirely and stay on RYLR998.
    loraRadioVariant = LoRaRadioVariant::RYLR998;
    return;
#endif
    String stored = doc["LoRa_Radio"] | "SX1276";
    if (stored.equalsIgnoreCase("SX1262")) { loraRadioVariant = LoRaRadioVariant::SX1262; }
    std::vector<Option> radioOptions = {
        {"SX1276", []() {}},
        {"SX1262", []() {}}
    };
    int selected = loopOptions(
        radioOptions, MENU_TYPE_SUBMENU, "LoRa Radio", (loraRadioVariant == LoRaRadioVariant::SX1262) ? 1 : 0
    );
    if (selected >= 0) {
        loraRadioVariant = (selected == 1) ? LoRaRadioVariant::SX1262 : LoRaRadioVariant::SX1276;
        doc["LoRa_Radio"] = (loraRadioVariant == LoRaRadioVariant::SX1262) ? "SX1262" : "SX1276";
        File cfg = LittleFS.open("/lora_settings.json", "w");
        serializeJson(doc, cfg);
        cfg.close();
    }
}

void mainloop() {
    long pressStartTime = 0;
    bool isPressing = false;
    bool breakloop = false;
    while (true) {
        render();
        reciveMessage();
        if (breakloop) { break; }
#ifdef HAS_3_BUTTONS
        if (EscPress) {
            long _tmp = millis();

            LongPress = true;
            while (EscPress) {
                if (millis() - _tmp > 200) {
                    // start drawing arc after short delay; animate over 500ms
                    int sweep = 0;
                    long elapsed = millis() - (_tmp + 200);
                    if (elapsed > 0) sweep = 360 * elapsed / 500;
                    if (sweep > 360) sweep = 360;
                    tft.drawArc(
                        tftWidth / 2,
                        tftHeight / 2,
                        25,
                        15,
                        0,
                        sweep,
                        getColorVariation(bruceConfig.priColor),
                        bruceConfig.bgColor
                    );
                }
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
            // clear arc
            tft.drawArc(
                tftWidth / 2, tftHeight / 2, 25, 15, 0, 360, bruceConfig.bgColor, bruceConfig.bgColor
            );
            LongPress = false;
            // #endif

            // decide short vs long after release
            if (millis() - _tmp > 700) {
                // long press -> exit
                breakloop = true;
            } else {
                // short press -> scroll down; consume flag first
                check(EscPress);
                upress();
            }
        }

        if (check(NextPress)) downpress();
        if (check(SelPress)) sendmsg();
#else

        if (check(NextPress)) downpress();
        if (check(EscPress)) break;
        if (check(PrevPress)) upress();
        if (check(SelPress)) sendmsg();
#endif

        delay(20);
    }
}

void lorachat() {
    // set filesystem thing
    if (!LittleFS.exists("/chats.txt")) {
        File file = LittleFS.open("/chats.txt", "w");
        file.close();
        Serial.println("chat file created :)");
    }
    if (!LittleFS.exists("/lora_settings.json")) {
        Serial.println("creating lora settings .json file");
        JsonDocument doc;
        File file = LittleFS.open("/lora_settings.json", "w");
        doc["LoRa_Frequency"] = "434500000.00";
        doc["LoRa_Name"] = "BruceTest";
#if defined(LORA_UART_ONLY_BACKEND)
        doc["LoRa_Radio"] = "RYLR998";
#else
        doc["LoRa_Radio"] = "SX1276";
#endif
        serializeJson(doc, file);
        file.close();
    }
    File file = LittleFS.open("/lora_settings.json", "r");
    JsonDocument doc;
    deserializeJson(doc, file);
    displayName = doc["LoRa_Name"].as<String>();
    double BAND = doc["LoRa_Frequency"].as<String>().toDouble();
    file.close();
    selectRadioVariant(doc);
    float bandMHz = (BAND > 1000) ? BAND / 1000000.0f : BAND;
    if (bandMHz <= 0) {
        displayError("Invalid LoRa frequency", true);
        return;
    }
    tft.fillScreen(TFT_BLACK);
    update = true;
    Serial.println("Initializing LoRa...");
#if defined(USE_RYLR998_VIA_UART)
    if (loraRadioVariant == LoRaRadioVariant::RYLR998) {
        Serial.println(
            "RYLR998 UART RX:" + String(bruceConfigPins.LoRa_uart_bus.rx) +
            " TX:" + String(bruceConfigPins.LoRa_uart_bus.tx) + " BAND: " + String(bandMHz) +
            "MHz DisplayName:  " + displayName
        );
    } else
#endif
    {
        Serial.println(
            "Pins: SCK:" + String(bruceConfigPins.LoRa_bus.sck) +
            " MISO:" + String(bruceConfigPins.LoRa_bus.miso) +
            " MOSI:" + String(bruceConfigPins.LoRa_bus.mosi) + " CS:" + String(bruceConfigPins.LoRa_bus.cs) +
            " RST:" + String(getLoraResetPin()) + " IRQ:" + String(getLoraIrqPin()) +
            "BAND: " + String(bandMHz) +
            "MHz Radio: " + ((loraRadioVariant == LoRaRadioVariant::SX1262) ? "SX1262" : "SX1276") +
            " DisplayName:  " + displayName
        );
    }

    if (!startLoraRadio(bandMHz)) {
        update = true;
        return;
    }
    tft.setTextWrap(true, true);
    tft.setTextDatum(TL_DATUM);
    loadMessages();
    mainloop();
}

// settings
// check the saving and loading
void changeusername() {
    tft.fillScreen(TFT_BLACK);
    String username = keyboard(username, 64, "");
    if (username == "" || username == "\x1B") return;
    File file = LittleFS.open("/lora_settings.json", "r");
    JsonDocument doc;
    deserializeJson(doc, file);
    file.close();
    doc["LoRa_Name"] = username;
    file = LittleFS.open("/lora_settings.json", "w");
    serializeJson(doc, file);
    file.close();
}

void chfreq() {
    tft.fillScreen(TFT_BLACK);
    char buf[15];
    File file = LittleFS.open("/lora_settings.json", "r");
    JsonDocument doc;
    deserializeJson(doc, file);
    file.close();

    double dfreq = doc["LoRa_Frequency"].as<String>().toDouble();
    dfreq = dfreq / 1000000;
    snprintf(buf, sizeof(buf), "%.3f", dfreq);
    String freq = num_keyboard(buf, 12, "in Mhz");
    dfreq = freq.toDouble();
    if (dfreq == 0 || freq == "\x1B") {
        displayError("Invalid value");
        return;
    } else if (dfreq > 1000) {
        displayError("Invalid value, Exceeds 1Ghz");
        return;
    }
    dfreq = dfreq * 1000000;
    snprintf(buf, sizeof(buf), "%.2f", dfreq);
    doc["LoRa_Frequency"] = buf;

    file = LittleFS.open("/lora_settings.json", "w");
    serializeJson(doc, file);
    file.close();
}
#endif
