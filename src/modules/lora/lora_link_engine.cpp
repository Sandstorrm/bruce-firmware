#include "lora_link_engine.h"
#include <math.h>
#include <string.h>

namespace blp {

const Profile kProfiles[kNumProfiles] = {
    {"Range",    9, 7, 4, 4, 128, -130},
    {"Long",     8, 7, 4, 4, 128, -127},
    {"Standard", 7, 7, 3, 4, 128, -125},
    {"Fast",     7, 8, 2, 4, 160, -122},
    {"Quick",    7, 9, 1, 4, 192, -119},
    {"Turbo",    5, 9, 1, 4, 192, -114},
};

static const char kHex[] = "0123456789ABCDEF";
static inline uint8_t hc(uint8_t v) { return (uint8_t)kHex[v & 0xF]; }
static inline int hv(uint8_t c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

uint16_t crc16(const uint8_t *d, size_t n, uint16_t crc) {
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

static double airtimeExact(uint8_t p, size_t frameLen) {
    const Profile &pr = kProfiles[p];
    double bw = pr.bw == 7 ? 125e3 : (pr.bw == 8 ? 250e3 : 500e3);
    double tsym = (double)(1u << pr.sf) / bw;
    int de = tsym > 0.016 ? 1 : 0;
    double tpre = (pr.pre + (pr.sf < 7 ? 6.25 : 4.25)) * tsym;
    double num = 8.0 * frameLen - 4.0 * pr.sf + 28 + 16 + (pr.sf < 7 ? 8 : 0);
    double q = ceil(num / (4.0 * (pr.sf - 2 * de)));
    double nsym = 8 + fmax(q * (pr.cr + 4), 0.0);
    return (tpre + nsym * tsym) * 1000.0;
}
uint32_t airtimeMs(uint8_t p, size_t frameLen) { return (uint32_t)ceil(airtimeExact(p, frameLen)); }
uint32_t waitMs(uint8_t p) { return (uint32_t)(150.0 + 1.2 * airtimeExact(p, kProfiles[p].maxFrame)); }
uint32_t confirmMs(uint8_t p) { return 5 * waitMs(p) + 3000; }

size_t escapedLen(uint8_t b) {
    if (b == 0x5C || b == 0x0A || b == 0x0D) return 2;
    return (b >= 0x20 && b <= 0x7E) ? 1 : 4;
}

size_t escapeInto(const uint8_t *raw, size_t rawLen, uint8_t *out, size_t budget, size_t *rawUsed) {
    size_t o = 0, i = 0;
    for (; i < rawLen; i++) {
        uint8_t b = raw[i];
        size_t need = escapedLen(b);
        if (o + need > budget) break;
        if (b == 0x5C) { out[o++] = '\\'; out[o++] = '\\'; }
        else if (b == 0x0A) { out[o++] = '\\'; out[o++] = 'n'; }
        else if (b == 0x0D) { out[o++] = '\\'; out[o++] = 'r'; }
        else if (b >= 0x20 && b <= 0x7E) out[o++] = b;
        else { out[o++] = '\\'; out[o++] = 'x'; out[o++] = hc(b >> 4); out[o++] = hc(b); }
    }
    if (rawUsed) *rawUsed = i;
    return o;
}

int unescapeInto(const uint8_t *t, size_t n, uint8_t *out, size_t cap) {
    size_t o = 0, i = 0;
    while (i < n) {
        if (o >= cap) return -1;
        uint8_t c = t[i];
        if (c != '\\') {
            if (c < 0x20 || c > 0x7E) return -1;
            out[o++] = c;
            i++;
            continue;
        }
        if (i + 1 >= n) return -1;
        uint8_t e = t[i + 1];
        if (e == '\\') { out[o++] = '\\'; i += 2; }
        else if (e == 'n') { out[o++] = '\n'; i += 2; }
        else if (e == 'r') { out[o++] = '\r'; i += 2; }
        else if (e == 'x' && i + 3 < n) {
            int hi = hv(t[i + 2]), lo = hv(t[i + 3]);
            if (hi < 0 || lo < 0) return -1;
            out[o++] = (uint8_t)(hi << 4 | lo);
            i += 4;
        } else return -1;
    }
    return (int)o;
}

// ---- frame helpers -------------------------------------------------------------------------

static void putQuality(uint8_t *p, int rssi, int snr) {
    int rr = -rssi;
    if (rr < 0) rr = 0;
    if (rr > 255) rr = 255;
    int ss = snr + 64;
    if (ss < 0) ss = 0;
    if (ss > 255) ss = 255;
    p[0] = hc(rr >> 4); p[1] = hc(rr); p[2] = hc(ss >> 4); p[3] = hc(ss);
}

size_t Engine::finish(uint8_t *buf, size_t bodyLen) {
    uint16_t c = crc16(buf, bodyLen);
    buf[bodyLen] = hc(c >> 12); buf[bodyLen + 1] = hc(c >> 8); buf[bodyLen + 2] = hc(c >> 4); buf[bodyLen + 3] = hc(c);
    return bodyLen + 4;
}

void Engine::reply(const uint8_t *frame, size_t len) {
    host_.sendFrame(frame, len);
    stats_.framesTx++;
}

// ---- engine ---------------------------------------------------------------------------------

void Engine::begin(uint32_t now) {
    lastRx_ = now;
    profile_ = kHome;
    switching_ = applyAfterTx_ = false;
    applyPowerAfterTx_ = false;
    power_ = host_.defaultPower();
    resetSession();
}

// A controller that lost us (HELLO) or went silent must always be able to hear us again.
void Engine::restorePower() {
    uint8_t def = host_.defaultPower();
    applyPowerAfterTx_ = false;
    if (power_ != def) {
        power_ = def;
        host_.setPower(def);
    }
}

void Engine::resetSession() {
    sid_ = 0;
    sndSeq_ = 0;
    rcvNext_ = 0;
    pend_ = pendSync_ = syncNeeded_ = pendSent_ = false;
    pendLen_ = 0;
    host_.txClear();
}

void Engine::goHome(uint32_t now) {
    (void)now;
    switching_ = applyAfterTx_ = false;
    profile_ = kHome;
    host_.setProfile(kHome);
    stats_.revertsToHome++;
}

void Engine::onTxDone(uint32_t now, bool ok) {
    if (applyPowerAfterTx_) {
        applyPowerAfterTx_ = false;
        if (ok) { // (if not, the controller never saw the confirmation and will ask again)
            power_ = swPower_;
            host_.setPower(power_);
        }
    }
    if (!applyAfterTx_) return;
    applyAfterTx_ = false;
    if (!ok) return; // the controller never saw Q, so it will not switch either
    switching_ = true;
    profile_ = swTarget_;
    host_.setProfile(profile_);
    confirmDeadline_ = now + confirmMs(profile_);
}

void Engine::tick(uint32_t now) {
    if (switching_ && (int32_t)(now - confirmDeadline_) > 0) { // controller never showed up: undo
        switching_ = false;
        profile_ = swFrom_;
        host_.setProfile(profile_);
        lastRx_ = now;
    }
    if (!applyAfterTx_ && (int32_t)(now - lastRx_) > (int32_t)kLostMs) {
        if (sid_) resetSession();
        if (profile_ != kHome) goHome(now);
        restorePower();
        lastRx_ = now;
    }
}

void Engine::replyData() {
    uint8_t buf[kMaxFrame + 8];
    uint8_t budget = (uint8_t)(kProfiles[profile_].maxFrame - 10 - 4);

    if (!pend_ && host_.txAvail() > 0) {
        uint8_t raw[kMaxFrame];
        size_t got = host_.txPeek(raw, budget < sizeof raw ? budget : sizeof raw);
        size_t used = 0;
        pendLen_ = escapeInto(raw, got, pendBuf_, budget, &used);
        if (used > 0) {
            host_.txDrop(used);
            stats_.bytesOut += used;
            pend_ = true;
            pendSent_ = false;
            pendSync_ = syncNeeded_;
            syncNeeded_ = false;
        }
    }
    uint8_t flags = 0;
    if (host_.txAvail() > 0) flags |= kFlagMore;
    if (pend_ && pendSync_) flags |= kFlagSync;

    size_t n = 0;
    buf[n++] = 'D'; buf[n++] = hc(sid_); buf[n++] = hc(sndSeq_); buf[n++] = hc(rcvNext_); buf[n++] = hc(flags);
    buf[n++] = hc(want_);
    putQuality(buf + n, rssi_, snr_);
    n += 4;
    if (pend_) {
        memcpy(buf + n, pendBuf_, pendLen_);
        n += pendLen_;
        if (pendSent_) stats_.retransmits++;
        pendSent_ = true;
    }
    reply(buf, finish(buf, n));
}

void Engine::handleData(uint32_t now, const uint8_t *b, size_t n) {
    (void)now;
    int seq = hv(b[2]), ack = hv(b[3]), flags = hv(b[4]);
    if (seq < 0 || ack < 0 || flags < 0) { stats_.badFrames++; return; }
    uint8_t tmp[kMaxFrame];
    int m = unescapeInto(b + 5, n - 5, tmp, sizeof tmp);
    if (m < 0) { stats_.badFrames++; return; }

    if (pend_ && ack == ((sndSeq_ + 1) & 15)) { // our segment arrived
        pend_ = false;
        sndSeq_ = (uint8_t)((sndSeq_ + 1) & 15);
    }
    if (m > 0) {
        if (seq == rcvNext_) {
            if (host_.rxSpace() >= (size_t)m) {
                host_.rxPush(tmp, (size_t)m);
                stats_.bytesIn += (uint32_t)m;
                rcvNext_ = (uint8_t)((rcvNext_ + 1) & 15);
                if (flags & kFlagFlush) {
                    host_.txClear();
                    pend_ = false;
                    sndSeq_ = (uint8_t)((sndSeq_ + 1) & 15);
                    syncNeeded_ = true;
                }
            }
        } else if (seq == ((rcvNext_ + 15) & 15)) {
            stats_.dupSegments++;
        }
    }
    replyData();
}

void Engine::onFrame(uint32_t now, const uint8_t *d, size_t len, int rssi, int snr) {
    if (len < 7 || len > kMaxFrame) { stats_.badFrames++; return; }
    for (size_t i = 0; i < len; i++)
        if (d[i] < 0x20 || d[i] > 0x7E) { stats_.badFrames++; return; }
    size_t bl = len - 4;
    uint16_t want = crc16(d, bl);
    int c0 = hv(d[bl]), c1 = hv(d[bl + 1]), c2 = hv(d[bl + 2]), c3 = hv(d[bl + 3]);
    if (c0 < 0 || c1 < 0 || c2 < 0 || c3 < 0 ||
        (uint16_t)(c0 << 12 | c1 << 8 | c2 << 4 | c3) != want) {
        stats_.badFrames++;
        return;
    }
    const uint8_t *b = d;
    uint8_t kind = b[0];
    int sid = bl >= 2 ? hv(b[1]) : -1;
    if (sid < 0) { stats_.badFrames++; return; }

    // From here on the frame is authentic enough to count as "the controller is there".
    stats_.framesRx++;
    lastRx_ = now;
    rssi_ = rssi;
    snr_ = snr;
    if (switching_) { // first valid frame after a profile change: the switch is confirmed
        switching_ = false;
    }

    uint8_t buf[32];
    size_t n = 0;
    if (kind == 'H' && bl == 4) {
        int ver = hv(b[2]), fresh = hv(b[3]);
        if (sid == 0 || ver != kVersion || fresh < 0) return;
        restorePower();
        uint8_t resumed = 1;
        if (fresh || sid != sid_) {
            resetSession();
            sid_ = (uint8_t)sid;
            stats_.sessions++;
            resumed = 0;
        }
        buf[n++] = 'W'; buf[n++] = hc(sid_); buf[n++] = hc(kVersion); buf[n++] = hc(profile_); buf[n++] = hc(want_);
        buf[n++] = hc(resumed);
        buf[n++] = hc(power_ >> 4); buf[n++] = hc(power_);
        putQuality(buf + n, rssi_, snr_);
        n += 4;
        reply(buf, finish(buf, n));
    } else if (kind == 'D' && bl >= 5) {
        if (sid != sid_ || sid_ == 0) goto nosession;
        handleData(now, b, bl);
    } else if (kind == 'T' && bl == 4) {
        if (sid != sid_ || sid_ == 0) goto nosession;
        int hi = hv(b[2]), lo = hv(b[3]);
        if (hi < 0 || lo < 0 || (hi << 4 | lo) > kMaxPower) return;
        uint8_t pw = (uint8_t)(hi << 4 | lo);
        buf[n++] = 'U'; buf[n++] = hc(sid_); buf[n++] = hc(pw >> 4); buf[n++] = hc(pw);
        putQuality(buf + n, rssi_, snr_);
        n += 4;
        if (pw != power_) {
            applyPowerAfterTx_ = true;
            swPower_ = pw;
        }
        reply(buf, finish(buf, n));
    } else if (kind == 'P' && bl == 3) {
        if (sid != sid_ || sid_ == 0) goto nosession;
        int target = hv(b[2]);
        if (target < 0 || target >= kNumProfiles) return;
        buf[n++] = 'Q'; buf[n++] = hc(sid_); buf[n++] = hc((uint8_t)target);
        putQuality(buf + n, rssi_, snr_);
        n += 4;
        if (target != profile_) {
            applyAfterTx_ = true;
            swTarget_ = (uint8_t)target;
            swFrom_ = profile_;
        }
        reply(buf, finish(buf, n));
    } else {
        stats_.badFrames++;
    }
    return;

nosession:
    buf[n++] = 'N'; buf[n++] = hc((uint8_t)sid);
    putQuality(buf + n, rssi_, snr_);
    n += 4;
    reply(buf, finish(buf, n));
}

} // namespace blp
