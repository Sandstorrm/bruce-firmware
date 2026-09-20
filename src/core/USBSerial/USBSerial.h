#ifndef BRUCE_USBSERIAL_H
#define BRUCE_USBSERIAL_H

#include "SerialDevice.h"
#include <Arduino.h>

class USBSerial : public SerialDevice {
public:
    size_t println(const String &s) override { return out->println(s); }
    size_t print(const String &s) override { return out->print(s); }
    size_t print(const int n, int format) override { return out->print(n, format); }
    void vprintf(const char *fmt, va_list args) override {
        // (this used to call out->printf(fmt, args): a va_list is not a vararg -- undefined behaviour and garbage output)
        char buf[256];
        va_list copy;
        va_copy(copy, args);
        int n = vsnprintf(buf, sizeof buf, fmt, args);
        if (n > 0 && (size_t)n < sizeof buf) {
            out->write((const uint8_t *)buf, (size_t)n);
        } else if (n >= (int)sizeof buf) {
            char *big = (char *)malloc((size_t)n + 1);
            if (big) {
                vsnprintf(big, (size_t)n + 1, fmt, copy);
                out->write((const uint8_t *)big, (size_t)n);
                free(big);
            }
        }
        va_end(copy);
    }
    size_t println() override { return out->println(); }
    size_t println(size_t n) override { return out->println(n); }
    size_t println(const uint32_t n) override { return out->println(n); }
    size_t println(const int n, int format) override { return out->println(n, format); }
    String readStringUntil(char terminator) override { return out->readStringUntil(terminator); }
    void flush() override { out->flush(); }
    int available() override { return out->available(); }
    size_t write(uint8_t *str, size_t size) override { return out->write(str, size); }
    int read() override { return out->read(); }
    void setSerialOutput(Stream *in) { out = in; }
    Stream *getSerialOutput() { return out; }
    USBSerial(Stream *in = &Serial) { out = in; }
    ~USBSerial() override = default;

private:
    Stream *out;
};

#endif // BRUCE_USBSERIAL_H
