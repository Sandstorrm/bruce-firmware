// Bruce LoRa link -- device-side protocol engine (v1).
//
// Pure C++ (no Arduino/ESP headers) so it can be unit-tested on a host machine against the
// Python controller. The controller is the master: it sends a frame, this engine answers it
// exactly once. Wire format and behaviour are specified in PROTOCOL.md of the
// bruce-lora-control project; every constant here mirrors bruce_lora/protocol.py.
#ifndef BRUCE_LORA_LINK_ENGINE_H
#define BRUCE_LORA_LINK_ENGINE_H

#include <stddef.h>
#include <stdint.h>

namespace blp {

constexpr uint8_t kVersion = 1;
constexpr uint8_t kFlagMore = 0x1;  // sender has more queued data
constexpr uint8_t kFlagSync = 0x2;  // seq restarts the stream: receiver adopts it
constexpr uint8_t kFlagFlush = 0x4; // controller -> device: drop queued output
constexpr uint8_t kWantAuto = 0xF;  // "no preference"
constexpr uint8_t kHome = 0;        // rendezvous profile
constexpr uint32_t kLostMs = 40000; // no valid frame this long -> drop session, go home
constexpr uint8_t kMaxPower = 22;    // RYLR998 AT+CRFOP range 0..22 dBm
constexpr size_t kMaxFrame = 200;

struct Profile {
    const char *name;
    uint8_t sf, bw, cr, pre; // bw: RYLR998 code 7=125k 8=250k 9=500k
    uint16_t maxFrame;
    int16_t sensitivityDbm; // rough, for the UI
};
constexpr uint8_t kNumProfiles = 6;
extern const Profile kProfiles[kNumProfiles];

uint16_t crc16(const uint8_t *d, size_t n, uint16_t crc = 0xFFFF);
uint32_t airtimeMs(uint8_t profile, size_t frameLen);
uint32_t waitMs(uint8_t profile);    // controller's reply timeout
uint32_t confirmMs(uint8_t profile); // device revert timer after switching to `profile`

// Escaping (payload bytes -> printable ASCII). All return lengths; `escapeInto` stops before a
// byte that would not fit, reporting how many raw bytes it consumed.
size_t escapedLen(uint8_t b);
size_t escapeInto(const uint8_t *raw, size_t rawLen, uint8_t *out, size_t budget, size_t *rawUsed);
// Returns decoded length, or -1 if `text` is not a valid escaped stream / does not fit.
int unescapeInto(const uint8_t *text, size_t n, uint8_t *out, size_t outCap);

// What the engine needs from the platform.
class Host {
  public:
    virtual ~Host() = default;
    // Transmit one frame. Completion must be reported through Engine::onTxDone().
    virtual void sendFrame(const uint8_t *frame, size_t len) = 0;
    // Apply a radio profile (AT+PARAMETER); called from within onFrame/onTxDone/tick.
    virtual void setProfile(uint8_t idx) = 0;
    // Apply a TX power in dBm (AT+CRFOP); the platform must apply it before the next frame it sends.
    virtual void setPower(uint8_t dbm) = 0;
    virtual uint8_t defaultPower() = 0; // power to fall back to (the user's setting, normally 22)
    // Bytes going to the CLI (controller -> device).
    virtual size_t rxSpace() = 0;
    virtual void rxPush(const uint8_t *d, size_t n) = 0;
    // Bytes coming from the CLI (device -> controller). peek copies without consuming.
    virtual size_t txAvail() = 0;
    virtual size_t txPeek(uint8_t *dst, size_t max) = 0;
    virtual void txDrop(size_t n) = 0;
    virtual void txClear() = 0;
    virtual void log(const char *msg) { (void)msg; }
};

struct Stats {
    uint32_t framesRx = 0, framesTx = 0, badFrames = 0, dupSegments = 0;
    uint32_t bytesIn = 0, bytesOut = 0, sessions = 0, revertsToHome = 0, retransmits = 0;
};

enum class LinkState : uint8_t { Listening, Linked };

class Engine {
  public:
    explicit Engine(Host &host) : host_(host) {}

    void begin(uint32_t now);
    // A complete frame was received. rssi in dBm, snr in dB (as reported by the module).
    void onFrame(uint32_t now, const uint8_t *data, size_t len, int rssi, int snr);
    // The frame passed to Host::sendFrame finished transmitting (ok=false: module refused it).
    void onTxDone(uint32_t now, bool ok);
    void tick(uint32_t now);

    // The user's requested profile (0..5) or kWantAuto; reported to the controller.
    void setWant(uint8_t want) { want_ = want; }
    uint8_t want() const { return want_; }

    LinkState state() const { return sid_ ? LinkState::Linked : LinkState::Listening; }
    uint8_t profile() const { return profile_; }
    uint8_t power() const { return power_; }
    bool switching() const { return switching_ || applyAfterTx_; }
    int lastRssi() const { return rssi_; }
    int lastSnr() const { return snr_; }
    uint32_t lastRxMs() const { return lastRx_; }
    const Stats &stats() const { return stats_; }

  private:
    void resetSession();
    void goHome(uint32_t now);
    void restorePower();
    void reply(const uint8_t *frame, size_t len);
    void replyData();
    void handleData(uint32_t now, const uint8_t *body, size_t n);
    size_t finish(uint8_t *buf, size_t bodyLen);

    Host &host_;
    Stats stats_;
    uint8_t sid_ = 0;
    uint8_t profile_ = kHome;
    uint8_t want_ = kWantAuto;
    int rssi_ = -255, snr_ = 0;
    uint32_t lastRx_ = 0;

    // data channel
    uint8_t sndSeq_ = 0;
    bool pend_ = false, pendSync_ = false, syncNeeded_ = false, pendSent_ = false;
    uint8_t pendBuf_[kMaxFrame];
    size_t pendLen_ = 0;
    uint8_t rcvNext_ = 0;

    // profile switching
    bool applyAfterTx_ = false, switching_ = false;
    uint8_t swTarget_ = 0, swFrom_ = 0;
    uint32_t confirmDeadline_ = 0;

    // TX power control
    uint8_t power_ = 22;
    bool applyPowerAfterTx_ = false;
    uint8_t swPower_ = 22;
};

} // namespace blp

#endif
