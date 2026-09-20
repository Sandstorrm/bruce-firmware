#if defined(USE_RYLR998_VIA_UART) && !defined(LITE_VERSION)
#include "LoRaLink.h"
#include "core/configPins.h"
#include "core/display.h"
#include "core/serialcmds.h"
#include "lora_link_engine.h"
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <SerialDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_system.h>
#include <freertos/task.h>
#include <globals.h>

extern BruceConfigPins bruceConfigPins;

#ifndef LORA_LINK_TX_POWER
#define LORA_LINK_TX_POWER 22 // dBm, 0..22. Lower it on the bench: two radios side by side overload each other.
#endif

namespace {

constexpr uint32_t kBaud = 115200;
constexpr size_t kRxRingSize = 1024; // controller -> CLI
constexpr size_t kTxRingSize = 6144; // CLI -> controller
constexpr uint32_t kBandDefaultHz = 915000000;
constexpr uint32_t kNetworkId = 18;
constexpr const char *kConfigPath = "/lora_link.json";

// ---- a small thread-safe byte queue -------------------------------------------------------------

class ByteRing {
  public:
    explicit ByteRing(size_t cap) : cap_(cap), buf_((uint8_t *)malloc(cap)), mtx_(xSemaphoreCreateMutex()) {}
    ~ByteRing() {
        free(buf_);
        vSemaphoreDelete(mtx_);
    }
    size_t size() { Lock l(mtx_); return n_; }
    size_t space() { Lock l(mtx_); return cap_ - n_; }
    size_t push(const uint8_t *d, size_t n) {
        Lock l(mtx_);
        if (n > cap_ - n_) n = cap_ - n_;
        for (size_t i = 0; i < n; i++) buf_[(r_ + n_ + i) % cap_] = d[i];
        n_ += n;
        return n;
    }
    size_t peek(uint8_t *dst, size_t max) {
        Lock l(mtx_);
        size_t n = max < n_ ? max : n_;
        for (size_t i = 0; i < n; i++) dst[i] = buf_[(r_ + i) % cap_];
        return n;
    }
    void drop(size_t n) {
        Lock l(mtx_);
        if (n > n_) n = n_;
        r_ = (r_ + n) % cap_;
        n_ -= n;
    }
    void clear() { Lock l(mtx_); r_ = n_ = 0; }
    int pop() {
        Lock l(mtx_);
        if (!n_) return -1;
        int b = buf_[r_];
        r_ = (r_ + 1) % cap_;
        n_--;
        return b;
    }
    bool hasByte(uint8_t b) {
        Lock l(mtx_);
        for (size_t i = 0; i < n_; i++)
            if (buf_[(r_ + i) % cap_] == b) return true;
        return false;
    }

  private:
    struct Lock {
        SemaphoreHandle_t m;
        explicit Lock(SemaphoreHandle_t mm) : m(mm) { xSemaphoreTake(m, portMAX_DELAY); }
        ~Lock() { xSemaphoreGive(m); }
    };
    size_t cap_, r_ = 0, n_ = 0;
    uint8_t *buf_;
    SemaphoreHandle_t mtx_;
};

ByteRing *rxRing = nullptr; // bytes for the CLI (controller -> device)
ByteRing *txRing = nullptr; // bytes from the CLI (device -> controller)

// ---- the CLI's view of the LoRa link -------------------------------------------------------------

class LoRaSerialDevice : public SerialDevice {
  public:
    // Output: everything the CLI prints for a remote command goes into txRing (CRs dropped: the
    // controller redraws lines itself, and it saves airtime).
    void emit(const uint8_t *d, size_t n) {
        if (!txRing) return;
        if (dropped_ && txRing->space() > 96) {
            char note[48];
            int m = snprintf(note, sizeof note, "\n[lora: %u bytes dropped]\n", (unsigned)dropped_);
            txRing->push((const uint8_t *)note, (size_t)m);
            dropped_ = 0;
        }
        uint8_t tmp[64];
        size_t k = 0;
        for (size_t i = 0; i < n; i++) {
            if (d[i] == '\r') continue;
            tmp[k++] = d[i];
            if (k == sizeof tmp || i == n - 1) {
                size_t put = txRing->push(tmp, k);
                dropped_ += k - put;
                k = 0;
            }
        }
        if (k) dropped_ += k - txRing->push(tmp, k);
    }
    size_t emitStr(const String &s) {
        emit((const uint8_t *)s.c_str(), s.length());
        return s.length();
    }

    size_t println(const String &s) override { return emitStr(s) + emitStr("\n"); }
    size_t println(size_t n) override { return emitStr(String((unsigned)n) + "\n"); }
    size_t println(uint32_t n) override { return emitStr(String((unsigned long)n) + "\n"); }
    size_t println() override { return emitStr("\n"); }
    size_t println(int n, int format) override { return emitStr(String(n, format) + "\n"); }
    size_t print(int n, int format) override { return emitStr(String(n, format)); }
    size_t print(const String &s) override { return emitStr(s); }
    void vprintf(const char *fmt, va_list args) override {
        char buf[256];
        va_list copy;
        va_copy(copy, args);
        int n = vsnprintf(buf, sizeof buf, fmt, args);
        if (n >= (int)sizeof buf) {
            char *big = (char *)malloc(n + 1);
            if (big) {
                vsnprintf(big, n + 1, fmt, copy);
                emit((const uint8_t *)big, n);
                free(big);
            }
        } else if (n > 0) {
            emit((const uint8_t *)buf, n);
        }
        va_end(copy);
    }
    size_t write(uint8_t *str, size_t size) override {
        emit(str, size);
        return size;
    }
    void flush() override {}

    // Input: a line is "available" only once its newline arrived, so the CLI never blocks in
    // readStringUntil() waiting for the rest of a command.
    int available() override {
        if (!rxRing) return 0;
        size_t n = rxRing->size();
        return (n && (rxRing->hasByte('\n') || n >= 128)) ? (int)n : 0;
    }
    int read() override { return rxRing ? rxRing->pop() : -1; }
    String readStringUntil(char terminator) override {
        String s;
        int c;
        while ((c = read()) >= 0) {
            if (c == (uint8_t)terminator) break;
            if (c != '\r') s += (char)c;
        }
        return s;
    }

  private:
    size_t dropped_ = 0;
};

LoRaSerialDevice loraSerial;

// Routes the CLI to USB and LoRa at once. Input comes from whichever has a line; output goes to USB
// always (so a wired terminal sees everything) and to LoRa while the current command came from there.
class MuxSerial : public SerialDevice {
  public:
    SerialDevice *usb = nullptr;
    volatile bool loraActive = false;
    uint32_t lastLoraInput = 0;

    bool remote() const { return loraActive && (millis() - lastLoraInput) < 120000; }
    // Only the CLI task talks to LoRa; other tasks (screen logger, web UI) that share serialDevice
    // must never spill into the air.
    bool toLora() const { return remote() && xTaskGetCurrentTaskHandle() == serialcmdsTaskHandle; }

    size_t println(const String &s) override { size_t n = usb->println(s); if (toLora()) loraSerial.println(s); return n; }
    size_t println(size_t v) override { size_t n = usb->println(v); if (toLora()) loraSerial.println(v); return n; }
    size_t println(uint32_t v) override { size_t n = usb->println(v); if (toLora()) loraSerial.println(v); return n; }
    size_t println() override { size_t n = usb->println(); if (toLora()) loraSerial.println(); return n; }
    size_t println(int v, int f) override { size_t n = usb->println(v, f); if (toLora()) loraSerial.println(v, f); return n; }
    size_t print(int v, int f) override { size_t n = usb->print(v, f); if (toLora()) loraSerial.print(v, f); return n; }
    size_t print(const String &s) override { size_t n = usb->print(s); if (toLora()) loraSerial.print(s); return n; }
    void vprintf(const char *fmt, va_list args) override {
        va_list copy;
        va_copy(copy, args);
        usb->vprintf(fmt, args);
        if (toLora()) loraSerial.vprintf(fmt, copy);
        va_end(copy);
    }
    size_t write(uint8_t *s, size_t n) override {
        size_t r = usb->write(s, n);
        if (toLora()) loraSerial.write(s, n);
        return r;
    }
    void flush() override { usb->flush(); }

    int available() override {
        int n = usb->available();
        return n > 0 ? n : loraSerial.available();
    }
    int read() override {
        if (usb->available() > 0) { loraActive = false; return usb->read(); }
        if (loraSerial.available() > 0) { loraActive = true; lastLoraInput = millis(); return loraSerial.read(); }
        return -1;
    }
    String readStringUntil(char t) override {
        if (usb->available() > 0) { loraActive = false; return usb->readStringUntil(t); }
        if (loraSerial.available() > 0) { loraActive = true; lastLoraInput = millis(); return loraSerial.readStringUntil(t); }
        return "";
    }
};

MuxSerial mux;

// ---- settings ------------------------------------------------------------------------------------

struct Settings {
    uint8_t want = blp::kWantAuto; // slider: fixed profile or "auto" (controller decides)
    uint8_t power = LORA_LINK_TX_POWER;
} cfg;

void loadSettings() {
    File f = LittleFS.open(kConfigPath, "r");
    if (!f) return;
    JsonDocument doc;
    if (!deserializeJson(doc, f)) {
        int w = doc["want"] | (int)blp::kWantAuto;
        int p = doc["power"] | (int)LORA_LINK_TX_POWER;
        cfg.want = (w >= 0 && w < blp::kNumProfiles) ? (uint8_t)w : blp::kWantAuto;
        cfg.power = (p >= 0 && p <= 22) ? (uint8_t)p : LORA_LINK_TX_POWER;
    }
    f.close();
}

void saveSettings() {
    File f = LittleFS.open(kConfigPath, "w");
    if (!f) return;
    JsonDocument doc;
    doc["want"] = cfg.want;
    doc["power"] = cfg.power;
    serializeJson(doc, f);
    f.close();
}

uint32_t configuredBandHz() {
    // Bruce's LoRa menu stores the band (Hz) in /lora_settings.json; the RYLR998 only covers ~820-960 MHz.
    File f = LittleFS.open("/lora_settings.json", "r");
    if (!f) return kBandDefaultHz;
    JsonDocument doc;
    uint32_t hz = kBandDefaultHz;
    if (!deserializeJson(doc, f)) {
        double v = doc["LoRa_Frequency"].as<String>().toDouble();
        if (v >= 820e6 && v <= 960e6) hz = (uint32_t)(v + 0.5);
    }
    f.close();
    return hz;
}

// ---- the link runtime ----------------------------------------------------------------------------

volatile int8_t pendingPower = -1; // set by the UI, applied by the link task (AT+CRFOP)

struct LinkHost : blp::Host {
    uint8_t frame[blp::kMaxFrame + 8];
    size_t frameLen = 0;
    volatile bool framePending = false;
    volatile int8_t wantRadioProfile = -1;

    void sendFrame(const uint8_t *f, size_t n) override {
        if (n > sizeof frame) return;
        memcpy(frame, f, n);
        frameLen = n;
        framePending = true;
    }
    void setProfile(uint8_t idx) override { wantRadioProfile = (int8_t)idx; }
    void setPower(uint8_t dbm) override { pendingPower = (int8_t)dbm; }
    uint8_t defaultPower() override { return cfg.power; }
    size_t rxSpace() override { return rxRing->space(); }
    void rxPush(const uint8_t *d, size_t n) override { rxRing->push(d, n); }
    size_t txAvail() override { return txRing->size(); }
    size_t txPeek(uint8_t *dst, size_t max) override { return txRing->peek(dst, max); }
    void txDrop(size_t n) override { txRing->drop(n); }
    void txClear() override { txRing->clear(); }
    void log(const char *m) override { Serial.printf("[lora] %s\n", m); }
};

enum class Rs : uint8_t { Idle, WaitAt, WaitParam, WaitSend };

HardwareSerial *uart = nullptr;
TaskHandle_t linkTask = nullptr;
volatile bool stopRequested = false;
volatile bool taskAlive = false;
portMUX_TYPE statusLock = portMUX_INITIALIZER_UNLOCKED;
LoRaLinkStatus statusCopy;

LinkHost host;
blp::Engine *engine = nullptr;
Rs rs = Rs::Idle;
uint32_t rsDeadline = 0;
uint8_t radioProfile = blp::kHome;
bool moduleOk = false;
uint32_t bandHz = kBandDefaultHz;

char lineBuf[360];
size_t lineLen = 0;

// The module conversation, kept in RTC memory: it survives panics, watchdog and software resets (not power loss),
// so after a crash `lora status` can still show what the link was doing just before it.
constexpr uint32_t kRtcMagic = 0xB10A11C0;
struct RtcLog {
    uint32_t magic;
    uint32_t boots;
    uint8_t n;
    struct Ev { uint32_t at; char what[28]; } ev[16];
};
RTC_NOINIT_ATTR RtcLog rtcLog;
RtcLog prevLog;
bool prevValid = false;
esp_reset_reason_t lastReset = ESP_RST_UNKNOWN;

const char *resetReasonName(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON: return "power-on";
        case ESP_RST_EXT: return "external pin";
        case ESP_RST_SW: return "software reset (esp_restart / 'reboot')";
        case ESP_RST_PANIC: return "PANIC (crash)";
        case ESP_RST_INT_WDT: return "INTERRUPT WATCHDOG (a task hogged the CPU)";
        case ESP_RST_TASK_WDT: return "TASK WATCHDOG";
        case ESP_RST_WDT: return "watchdog";
        case ESP_RST_DEEPSLEEP: return "woke from deep sleep";
        case ESP_RST_BROWNOUT: return "BROWNOUT (supply dipped)";
        default: return "unknown";
    }
}

// diagnostics / watchdog
struct Diag {
    uint32_t atTimeouts = 0, radioTimeouts = 0, moduleErrors = 0, moduleResets = 0, probes = 0, probeFails = 0;
    uint32_t badLines = 0, rcvLines = 0, okLines = 0;
    uint32_t lastRadioActivity = 0, lastProbe = 0, lastTxDone = 0, lastRcv = 0;
    char lastError[40] = "";
    bool probing = false;
    uint32_t lastModuleInit = 0;
    uint32_t moduleRestarts = 0, configDrifts = 0; // module rebooted itself (+READY) / came back with other settings
    // What we last said to the module (and what it said back): the wedge is only visible in this ordering.
    struct Ev { uint32_t at; char what[28]; } hist[20];
    uint8_t histN = 0;
    void note(const char *fmt, ...) {
        Ev &e = hist[histN++ % 20];
        e.at = millis();
        va_list a;
        va_start(a, fmt);
        vsnprintf(e.what, sizeof e.what, fmt, a);
        va_end(a);
        if (rtcLog.magic == kRtcMagic) { // mirror into RTC memory
            RtcLog::Ev &r = rtcLog.ev[rtcLog.n++ % 16];
            r.at = e.at;
            memcpy(r.what, e.what, sizeof r.what);
        }
    }
} diag;
volatile bool resetRequested = false;
volatile bool reinitNow = false; // module lost its settings: re-initialise immediately, don't wait for the retry timer

void uartSend(const String &s) {
    diag.note("> %s", s.c_str());
    uart->print(s);
    uart->print("\r\n");
}

String paramString(uint8_t idx) {
    const blp::Profile &p = blp::kProfiles[idx];
    return String("AT+PARAMETER=") + p.sf + "," + p.bw + "," + p.cr + "," + p.pre;
}

// Blocking AT command used only while (re)initialising the module; receive lines are discarded.
bool atBlocking(const String &cmd, uint32_t timeoutMs = 1500) {
    while (uart->available()) uart->read();
    uartSend(cmd);
    String line;
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        while (uart->available()) {
            char c = (char)uart->read();
            if (c == '\n') {
                line.trim();
                if (line.startsWith("+OK")) return true;
                if (line.startsWith("+ERR")) { Serial.printf("[lora] %s -> %s\n", cmd.c_str(), line.c_str()); return false; }
                line = "";
            } else {
                line += c;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    Serial.printf("[lora] %s -> (timeout)\n", cmd.c_str());
    return false;
}

bool initModule() {
    // The ESP32's reset button does not reset the RYLR998 (it has its own power), and a module can be left deaf yet
    // still answering AT commands. So every (re)initialisation starts from a hard reset of the radio chip.
    moduleOk = false;
    diag.moduleResets++;
    diag.note("AT+RESET");
    uartSend("AT+RESET");
    uint32_t t0 = millis();
    String seen;
    while (millis() - t0 < 2500) {
        while (uart->available()) {
            seen += (char)uart->read();
            if (seen.length() > 64) seen.remove(0, 32);
        }
        if (seen.indexOf("+READY") >= 0 && millis() - t0 > 300) break;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    delay(150);
    while (uart->available()) uart->read();
    for (int i = 0; i < 3 && !moduleOk; i++) moduleOk = atBlocking("AT", 600);
    if (!moduleOk) return false;
    bool ok = atBlocking("AT+BAND=" + String(bandHz)) && atBlocking("AT+NETWORKID=" + String(kNetworkId)) &&
              atBlocking("AT+ADDRESS=0") && atBlocking("AT+CRFOP=" + String(cfg.power)) &&
              atBlocking(paramString(blp::kHome));
    radioProfile = blp::kHome;
    if (!ok) moduleOk = false;
    Serial.printf("[lora] module %s, band %.3f MHz, power %u dBm\n", ok ? "ready" : "init failed", bandHz / 1e6, cfg.power);
    return ok;
}

// "+RCV=<addr>,<len>,<data>,<rssi>,<snr>": the payload is cut by its declared length, never by commas.
bool parseRcv(const char *s, size_t n, const uint8_t **data, size_t *dlen, int *rssi, int *snr) {
    if (n < 10 || strncmp(s, "+RCV=", 5) != 0) return false;
    const char *end = s + n;
    const char *c1 = (const char *)memchr(s + 5, ',', end - (s + 5));
    if (!c1) return false;
    const char *c2 = (const char *)memchr(c1 + 1, ',', end - (c1 + 1));
    if (!c2) return false;
    int len = atoi(c1 + 1);
    if (len <= 0 || c2 + 1 + len >= end) return false;
    const char *tail = c2 + 1 + len;
    if (*tail != ',') return false;
    const char *c3 = (const char *)memchr(tail + 1, ',', end - (tail + 1));
    if (!c3) return false;
    *data = (const uint8_t *)(c2 + 1);
    *dlen = (size_t)len;
    *rssi = atoi(tail + 1);
    *snr = atoi(c3 + 1);
    return true;
}

void onModuleOk(uint32_t now) {
    diag.okLines++;
    diag.note("< +OK (state %u)", (unsigned)rs);
    if (rs == Rs::WaitAt && diag.probing) {
        diag.probing = false;
        diag.probeFails = 0;
    }
    if (rs == Rs::WaitSend) {
        diag.lastTxDone = now;
        rs = Rs::Idle;
        engine->onTxDone(now, true);
    } else if (rs != Rs::Idle) {
        rs = Rs::Idle;
    }
}

void onModuleErr(uint32_t now, const char *line) {
    Serial.printf("[lora] module: %s\n", line);
    diag.moduleErrors++;
    diag.note("< %s", line);
    strncpy(diag.lastError, line, sizeof diag.lastError - 1);
    if (rs == Rs::WaitSend) {
        rs = Rs::Idle;
        engine->onTxDone(now, false);
    } else {
        rs = Rs::Idle;
    }
}

void handleLine(uint32_t now, const char *s, size_t n) {
    if (n == 0) return;
    if (s[0] == '+' && n >= 4 && strncmp(s, "+RCV", 4) == 0) {
        const uint8_t *data;
        size_t dlen;
        int rssi, snr;
        if (parseRcv(s, n, &data, &dlen, &rssi, &snr)) {
            diag.rcvLines++;
            diag.note("< RCV %u B", (unsigned)dlen);
            diag.lastRcv = diag.lastRadioActivity = now;
            engine->onFrame(now, data, dlen, rssi, snr);
        } else {
            diag.badLines++;
        }
    } else if (strncmp(s, "+READY", 6) == 0) {
        // The module rebooted on its own (a supply dip while the Core2 starts WiFi, say) and is back on factory
        // settings, where it cannot hear us -- yet it still answers AT. Bring it back at once.
        diag.note("< +READY (restarted)");
        diag.moduleRestarts++;
        Serial.println("[lora] the RYLR998 restarted by itself: re-initialising it");
        moduleOk = false;
        reinitNow = true;
        rs = Rs::Idle;
        diag.probing = false;
    } else if (strncmp(s, "+PARAMETER=", 11) == 0 && diag.probing) {
        const blp::Profile &p = blp::kProfiles[radioProfile];
        char want[24];
        snprintf(want, sizeof want, "%u,%u,%u,%u", p.sf, p.bw, p.cr, p.pre);
        diag.probing = false;
        diag.probeFails = 0;
        rs = Rs::Idle;
        if (strcmp(s + 11, want) != 0) {
            Serial.printf("[lora] module settings drifted (%s, expected %s): re-initialising\n", s + 11, want);
            diag.note("cfg drift %s", s + 11);
            diag.configDrifts++;
            moduleOk = false;
            reinitNow = true;
        }
    } else if (strncmp(s, "+OK", 3) == 0) {
        onModuleOk(now);
    } else if (strncmp(s, "+ERR", 4) == 0) {
        onModuleErr(now, s);
    }
}

void pumpUart(uint32_t now) {
    while (uart->available()) {
        char c = (char)uart->read();
        if (c == '\n') {
            while (lineLen && lineBuf[lineLen - 1] == '\r') lineLen--;
            lineBuf[lineLen] = '\0';
            handleLine(now, lineBuf, lineLen);
            lineLen = 0;
        } else if (lineLen < sizeof lineBuf - 1) {
            lineBuf[lineLen++] = c;
        } else {
            lineLen = 0; // overlong garbage: resynchronise on the next newline
        }
    }
}

// One radio operation at a time: a power change, then a pending profile change, then the next frame.
void serviceRadio(uint32_t now) {
    if (rs != Rs::Idle) {
        if ((int32_t)(now - rsDeadline) > 0) {
            Serial.printf("[lora] radio timeout in state %u\n", (unsigned)rs);
            if (rs == Rs::WaitSend) {
                // The module never reported the end of a transmission (it normally does within ~60 ms of the airtime):
                // its radio is stuck. It still answers AT, but neither sends nor receives until it is reset.
                diag.radioTimeouts++;
                diag.note("TX never finished");
                engine->onTxDone(now, false);
                resetRequested = true;
            }
            else { diag.atTimeouts++; if (diag.probing) { diag.probing = false; diag.probeFails++; } }
            rs = Rs::Idle;
        }
        return;
    }
    // Watchdog: a module that stops answering must not silence the remote for good.
    if (!host.framePending && host.wantRadioProfile < 0 && pendingPower < 0 &&
        now - diag.lastRadioActivity > 12000 && now - diag.lastProbe > 12000) {
        diag.lastProbe = now;
        diag.probes++;
        diag.probing = true;
        uartSend("AT+PARAMETER?"); // answers with the settings actually in force: catches a module that reset itself
        rs = Rs::WaitAt;
        rsDeadline = now + 1000;
        return;
    }
    if (pendingPower >= 0) {
        uartSend("AT+CRFOP=" + String((int)pendingPower));
        pendingPower = -1;
        rs = Rs::WaitAt;
        rsDeadline = now + 1500;
        return;
    }
    if (host.wantRadioProfile >= 0) {
        uint8_t p = (uint8_t)host.wantRadioProfile;
        host.wantRadioProfile = -1;
        uartSend(paramString(p));
        radioProfile = p;
        rs = Rs::WaitParam;
        rsDeadline = now + 1500;
        return;
    }
    if (host.framePending) {
        host.framePending = false;
        diag.note("> SEND %u", (unsigned)host.frameLen);
        uart->print("AT+SEND=0,");
        uart->print((unsigned)host.frameLen);
        uart->print(",");
        uart->write(host.frame, host.frameLen);
        uart->print("\r\n");
        rs = Rs::WaitSend;
        diag.lastRadioActivity = now;
        rsDeadline = now + blp::airtimeMs(radioProfile, host.frameLen) + 700;
    }
}

void publishStatus(uint32_t now) {
    LoRaLinkStatus s;
    s.running = true;
    s.moduleOk = moduleOk;
    s.linked = engine->state() == blp::LinkState::Linked;
    s.switching = engine->switching();
    s.profile = engine->profile();
    s.profileName = blp::kProfiles[s.profile].name;
    s.rssi = engine->lastRssi();
    s.snr = engine->lastSnr();
    s.lastRxAgoMs = engine->stats().framesRx ? now - engine->lastRxMs() : 0xFFFFFFFF;
    const blp::Stats &st = engine->stats();
    s.framesRx = st.framesRx; s.framesTx = st.framesTx; s.badFrames = st.badFrames;
    s.bytesIn = st.bytesIn; s.bytesOut = st.bytesOut; s.sessions = st.sessions;
    s.want = cfg.want;
    s.power = cfg.power;
    s.powerNow = engine->power();
    portENTER_CRITICAL(&statusLock);
    statusCopy = s;
    portEXIT_CRITICAL(&statusLock);
}

void hardResetModule() {
    Serial.println("[lora] resetting the RYLR998");
    moduleOk = false; // the task loop re-initialises it, starting with AT+RESET (and restarts the engine)
    rs = Rs::Idle;
    diag.probing = false;
    diag.probeFails = 0;
    host.framePending = false;
    host.wantRadioProfile = -1;
    lineLen = 0;
}

void linkTaskFn(void *) {
    taskAlive = true;
    uint32_t lastInitTry = 0, lastStatus = 0;
    while (!stopRequested) {
        uint32_t now = millis();
        // A module can go deaf while still answering AT commands, and deaf looks exactly like "no controller
        // around" -- so a long silence gets the radio reset (a controller polls at least every few seconds).
        bool silent = moduleOk && (now - diag.lastRcv > 45000) && (now - diag.lastModuleInit > 45000) &&
                      engine->state() == blp::LinkState::Listening;
        // Linked, yet not a single frame for 20 s although the controller polls every few seconds: the radio went deaf.
        bool deafWhileLinked = moduleOk && engine->state() == blp::LinkState::Linked && (now - diag.lastRcv > 20000) &&
                               (now - diag.lastModuleInit > 20000);
        if (resetRequested || diag.probeFails >= 2 || silent || deafWhileLinked) {
            if (silent || deafWhileLinked) diag.note("silence reset");
            resetRequested = false;
            hardResetModule();
            lastInitTry = 0;
            continue;
        }
        if (!moduleOk) {
            if (reinitNow) {
                reinitNow = false;
                lastInitTry = 0;
            }
            if (now - lastInitTry > 5000 || lastInitTry == 0) {
                lastInitTry = now ? now : 1;
                if (initModule()) {
                    engine->begin(millis());
                    engine->setWant(cfg.want);
                    diag.lastRadioActivity = millis();
                    diag.lastProbe = millis();
                    diag.lastModuleInit = millis();
                    diag.note("module init");
                }
            }
            publishStatus(now);
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        pumpUart(now);
        engine->tick(now);
        serviceRadio(now);
        if (now - lastStatus > 200) {
            lastStatus = now;
            publishStatus(now);
            // BLE (or anything else) may have swapped the serial device back to plain USB
            if (serialDevice == (SerialDevice *)&USBserial) {
                mux.usb = &USBserial;
                serialDevice = &mux;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    taskAlive = false;
    vTaskDelete(nullptr);
}

} // namespace

// ---- public API ----------------------------------------------------------------------------------

void loraLinkBegin() {
    static bool firstBegin = true;
    if (firstBegin) {
        firstBegin = false;
        lastReset = esp_reset_reason();
        prevValid = lastReset != ESP_RST_POWERON && rtcLog.magic == kRtcMagic;
        if (prevValid) prevLog = rtcLog;
        uint32_t boots = prevValid ? prevLog.boots + 1 : 1;
        memset(&rtcLog, 0, sizeof rtcLog);
        rtcLog.magic = kRtcMagic;
        rtcLog.boots = boots;
        Serial.printf("[lora] boot #%u, reset reason: %s\n", (unsigned)boots, resetReasonName(lastReset));
    }
    if (linkTask) return;
    if (bruceConfigPins.LoRa_uart_bus.rx == GPIO_NUM_NC || bruceConfigPins.LoRa_uart_bus.tx == GPIO_NUM_NC) {
        Serial.println("[lora] RYLR998 UART pins not configured");
        return;
    }
    if (!rxRing) rxRing = new ByteRing(kRxRingSize);
    if (!txRing) txRing = new ByteRing(kTxRingSize);
    if (!engine) engine = new blp::Engine(host);
    rxRing->clear();
    txRing->clear();
    loadSettings();
    bandHz = configuredBandHz();
    moduleOk = false;
    rs = Rs::Idle;
    pendingPower = -1;
    lineLen = 0;
    host.framePending = false;
    host.wantRadioProfile = -1;

    uart = new HardwareSerial(2);
    uart->setRxBufferSize(1024);
    uart->begin(kBaud, SERIAL_8N1, bruceConfigPins.LoRa_uart_bus.rx, bruceConfigPins.LoRa_uart_bus.tx);
    stopRequested = false;

    if (serialDevice == (SerialDevice *)&USBserial) {
        mux.usb = &USBserial;
        serialDevice = &mux;
    }
    statusCopy = LoRaLinkStatus();
    xTaskCreatePinnedToCore(linkTaskFn, "lora_link", 8192, nullptr, 2, &linkTask, 0);
    if (!linkTask) Serial.println("[lora] failed to start the link task");
}

void loraLinkStop() {
    if (!linkTask) return;
    if (serialDevice == &mux) serialDevice = &USBserial;
    mux.loraActive = false;
    stopRequested = true;
    for (int i = 0; i < 100 && taskAlive; i++) vTaskDelay(pdMS_TO_TICKS(20));
    linkTask = nullptr;
    if (uart) {
        uart->end();
        delete uart;
        uart = nullptr;
    }
    portENTER_CRITICAL(&statusLock);
    statusCopy = LoRaLinkStatus();
    portEXIT_CRITICAL(&statusLock);
}

bool loraLinkRunning() { return linkTask != nullptr; }
void loraLinkResetModule() { resetRequested = true; }

String loraLinkDebug() {
    if (!linkTask || !engine) return "LoRa link: not running\n";
    uint32_t now = millis();
    LoRaLinkStatus st = loraLinkStatus();
    const blp::Stats &s = engine->stats();
    String o;
    o += String("LoRa link: ") + (st.moduleOk ? "module ok" : "MODULE NOT ANSWERING") + ", " +
         (st.linked ? "linked" : "listening") + "\n";
    o += String("  up ") + (now / 1000) + " s, boot #" + rtcLog.boots + ", last reset: " + resetReasonName(lastReset) +
         ", free heap " + ESP.getFreeHeap() + " (lowest " + ESP.getMinFreeHeap() + ")\n";
    if (prevValid) {
        o += "  before that reboot, the link's last module events (ms since that boot):\n";
        for (int i = 0; i < 16; i++) {
            const RtcLog::Ev &e = prevLog.ev[(prevLog.n + i) % 16];
            if (e.at) o += String("    ") + e.at + "  " + e.what + "\n";
        }
    }
    o += String("  profile ") + blp::kProfiles[st.profile].name + (st.switching ? " (switching)" : "") +
         ", radio set to " + blp::kProfiles[radioProfile].name + ", want " + (st.want == 0xF ? String("auto") : String(st.want)) + "\n";
    o += String("  power ") + st.powerNow + " dBm (max " + st.power + "), last rssi " + st.rssi + " snr " + st.snr + "\n";
    o += String("  frames rx ") + s.framesRx + " tx " + s.framesTx + " bad " + s.badFrames + " dup " + s.dupSegments +
         " retx " + s.retransmits + ", sessions " + s.sessions + ", reverts " + s.revertsToHome + "\n";
    o += String("  bytes in ") + s.bytesIn + " out " + s.bytesOut + ", queued out " + (txRing ? txRing->size() : 0) + "\n";
    o += String("  radio state ") + (int)rs + ", pending frame " + (host.framePending ? "yes" : "no") +
         ", pending profile " + (int)host.wantRadioProfile + "\n";
    o += String("  rcv lines ") + diag.rcvLines + " bad " + diag.badLines + ", ok " + diag.okLines + ", module errors " +
         diag.moduleErrors + " (last: " + diag.lastError + ")\n";
    o += String("  timeouts: tx ") + diag.radioTimeouts + " at " + diag.atTimeouts + ", probes " + diag.probes +
         " failed " + diag.probeFails + ", module resets " + diag.moduleResets + ", module self-restarts " +
         diag.moduleRestarts + ", settings drifts " + diag.configDrifts + "\n";
    o += String("  last rx ") + (diag.lastRcv ? String((now - diag.lastRcv) / 1000) + " s ago" : String("never")) +
         ", last tx done " + (diag.lastTxDone ? String((now - diag.lastTxDone) / 1000) + " s ago" : String("never")) + "\n";
    o += "  module conversation (oldest first):\n";
    for (int i = 0; i < 20; i++) {
        const Diag::Ev &e = diag.hist[(diag.histN + i) % 20];
        if (e.at) o += String("    -") + ((now - e.at) / 1000.0f) + "s  " + e.what + "\n";
    }
    return o;
}
bool loraLinkRemoteActive() { return serialDevice == &mux && mux.remote(); }

LoRaLinkStatus loraLinkStatus() {
    LoRaLinkStatus s;
    portENTER_CRITICAL(&statusLock);
    s = statusCopy;
    portEXIT_CRITICAL(&statusLock);
    return s;
}

// ---- "Range / Speed" screen ----------------------------------------------------------------------

namespace {

const char *kStopNames[] = {"Auto", "Range", "Long", "Standard", "Fast", "Quick", "Turbo"};
constexpr int kStops = 7; // Auto + 6 profiles

int stopFromWant(uint8_t want) { return want == blp::kWantAuto ? 0 : want + 1; }
uint8_t wantFromStop(int stop) { return stop == 0 ? blp::kWantAuto : (uint8_t)(stop - 1); }

void drawSliderScreen(int row, int stop, uint8_t power, const LoRaLinkStatus &st) {
    const int w = tftWidth, x0 = 14, x1 = w - 14;
    const uint16_t fg = bruceConfig.priColor, dim = bruceConfig.secColor, bg = bruceConfig.bgColor;
    tft.fillRect(x0 - 4, 34, x1 - x0 + 8, tftHeight - 44, bg);
    tft.setTextColor(fg, bg);

    // slider
    tft.setTextSize(1);
    tft.setCursor(x0, 40);
    tft.setTextColor(row == 0 ? fg : dim, bg);
    tft.print("Range / Speed");
    tft.setTextSize(2);
    tft.setCursor(x0, 54);
    tft.setTextColor(fg, bg);
    tft.print(kStopNames[stop]);
    tft.setTextSize(1);

    int ty = 92, tw = x1 - x0;
    tft.drawFastHLine(x0, ty, tw, dim);
    for (int i = 0; i < kStops; i++) {
        int x = x0 + (tw * i) / (kStops - 1);
        tft.drawFastVLine(x, ty - 4, 9, dim);
    }
    int kx = x0 + (tw * stop) / (kStops - 1);
    tft.fillCircle(kx, ty, 6, fg);
    tft.setTextColor(dim, bg);
    tft.setCursor(x0, ty + 10);
    tft.print("range");
    tft.setCursor(x1 - 6 * 5, ty + 10);
    tft.print("speed");

    // what the selection means
    tft.setTextColor(fg, bg);
    tft.setCursor(x0, ty + 26);
    if (stop == 0) {
        tft.print("Controller picks the best profile");
    } else {
        const blp::Profile &p = blp::kProfiles[stop - 1];
        tft.printf("SF%u %uk CR4/%u  %lu ms/frame", p.sf, p.bw == 7 ? 125 : (p.bw == 8 ? 250 : 500), p.cr + 4,
                   (unsigned long)blp::airtimeMs(stop - 1, 128));
        tft.setCursor(x0, ty + 38);
        tft.setTextColor(dim, bg);
        tft.printf("sensitivity about %d dBm", p.sensitivityDbm);
    }

    // TX power
    tft.setTextColor(row == 1 ? fg : dim, bg);
    tft.setCursor(x0, ty + 58);
    tft.printf("Max TX power  %u dBm", power);
    if (st.linked && st.powerNow != power) {
        tft.setTextColor(dim, bg);
        tft.printf("  (now %u)", st.powerNow);
    }

    // live link state
    tft.setTextColor(st.linked ? fg : dim, bg);
    tft.setCursor(x0, ty + 78);
    if (!st.moduleOk) {
        tft.print("Radio: no answer from RYLR998");
    } else if (st.linked) {
        tft.printf("Linked  %s%s  %d dBm  SNR %d", st.profileName, st.switching ? "*" : "", st.rssi, st.snr);
    } else {
        tft.print("Waiting for controller...");
    }
    tft.setTextColor(dim, bg);
    tft.setCursor(x0, ty + 90);
    tft.printf("rx %lu tx %lu bad %lu", (unsigned long)st.framesRx, (unsigned long)st.framesTx, (unsigned long)st.badFrames);
}

} // namespace

void loraLinkSliderMenu() {
    int row = 0;
    LoRaLinkStatus st = loraLinkStatus();
    loadSettings();
    int stop = stopFromWant(cfg.want);
    uint8_t power = cfg.power;
    bool changed = false;

    drawMainBorderWithTitle("LoRa Remote");
    drawSliderScreen(row, stop, power, st);
    uint32_t lastDraw = millis();
    while (true) {
        bool dirty = false;
        if (check(EscPress)) break;
        if (check(SelPress)) { row ^= 1; dirty = true; }
        int dir = 0;
        if (check(NextPress) || check(DownPress)) dir = 1;
        if (check(PrevPress) || check(UpPress)) dir = -1;
        if (dir) {
            dirty = true;
            if (row == 0) {
                stop = (stop + dir + kStops) % kStops;
            } else {
                int p = (int)power + dir * 2;
                power = (uint8_t)constrain(p, 0, 22);
            }
            changed = true;
        }
        if (dirty || millis() - lastDraw > 500) {
            st = loraLinkStatus();
            drawSliderScreen(row, stop, power, st);
            lastDraw = millis();
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    if (changed) {
        cfg.want = wantFromStop(stop);
        cfg.power = power;
        saveSettings();
        if (engine && loraLinkRunning()) {
            engine->setWant(cfg.want);      // the controller sees this in its next reply and follows
            pendingPower = (int8_t)cfg.power;
        }
    }
}

#endif // USE_RYLR998_VIA_UART && !LITE_VERSION
