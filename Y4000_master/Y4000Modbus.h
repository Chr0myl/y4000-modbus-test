// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 YOUR NAME HERE

/*
 * Y4000Modbus.h - minimal Modbus RTU master for the Yosemitech Y4000 sonde.
 *
 * Header-only, no library dependencies, works on any Arduino core that gives
 * you a Stream (HardwareSerial). Written for RAK11300/RP2040 + RAK5802, but
 * nothing here is board-specific.
 *
 * The RAK5802 (TP8485E) drives its own DE/RE, so there is no direction pin to
 * toggle. It does echo your own transmission back on RX, which is why
 * everything below resyncs on the address+function byte pair instead of
 * assuming the first byte received is the start of the reply.
 */

#pragma once
#include <Arduino.h>
#include <string.h>

// How the sonde lays a 32-bit IEEE-754 float across two Modbus registers.
// ABCD = standard big-endian (A is the MSB). Yosemitech normally uses DCBA,
// but confirm with the bench sketch before trusting it.
enum Y4000FloatOrder { ORDER_ABCD, ORDER_DCBA, ORDER_CDAB, ORDER_BADC };

class Y4000Modbus {
  public:
    static const uint8_t  NUM_PARAMS       = 8;
    static const uint16_t REG_START_MEAS   = 0x2500;
    static const uint16_t REG_STOP_MEAS    = 0x2E00;
    static const uint16_t REG_VALUES       = 0x2600;  // 16 regs = 8 floats
    static const uint16_t REG_BRUSH        = 0x2F00;

    void begin(Stream *serial, uint8_t slaveAddr = 0x01) {
        _s    = serial;
        _addr = slaveAddr;
    }

    void setFloatOrder(Y4000FloatOrder o) { _order = o; }
    void setTimeout(uint32_t ms)          { _timeoutMs = ms; }
    void setDebug(Stream *dbg)            { _dbg = dbg; }

    // Only needed for bare MAX485/MAX3485 breakouts, where you drive DE and
    // /RE yourself. Leave unset for the RAK5802, which is self-directing, and
    // for a direct TTL cross-connection between two boards.
    void setDePin(int8_t pin) {
        _dePin = pin;
        if (_dePin >= 0) {
            pinMode(_dePin, OUTPUT);
            digitalWrite(_dePin, LOW);   // receive
        }
    }

    // Yosemitech's non-standard "command" form: function 0x03 with a register
    // quantity of zero. The reply is short and does not follow the normal
    // response layout, so we only check that something came back.
    bool startMeasurement() { return sendCommand(REG_START_MEAS); }
    bool stopMeasurement()  { return sendCommand(REG_STOP_MEAS); }
    bool activateBrush()    { return sendCommand(REG_BRUSH); }

    // Reads all 8 parameters in one transaction.
    // Order as implemented by EnviroDIY/YosemitechModbus:
    //   [0] DO mg/L  [1] Turbidity NTU  [2] Conductivity mS/cm  [3] pH
    //   [4] Temp C   [5] ORP mV         [6] Chlorophyll ug/L    [7] BGA ug/L
    // VERIFY this against your own sonde before labelling anything.
    // rawOut, if given, must be at least 32 bytes.
    bool readValues(float out[NUM_PARAMS], uint8_t *rawOut = nullptr) {
        uint8_t data[32];
        if (!readHoldingRegisters(REG_VALUES, 16, data, sizeof(data))) return false;
        if (rawOut) memcpy(rawOut, data, sizeof(data));
        for (uint8_t i = 0; i < NUM_PARAMS; i++)
            out[i] = decodeFloat(&data[i * 4], _order);
        return true;
    }

    // Generic function 0x03. dest receives the payload bytes (2 * qty).
    bool readHoldingRegisters(uint16_t startReg, uint16_t qty,
                              uint8_t *dest, size_t destLen) {
        if (destLen < (size_t)qty * 2) return false;

        uint8_t req[8];
        req[0] = _addr;
        req[1] = 0x03;
        req[2] = highByte(startReg);
        req[3] = lowByte(startReg);
        req[4] = highByte(qty);
        req[5] = lowByte(qty);
        appendCRC(req, 6);

        uint8_t resp[64];
        int n = transact(req, 8, resp, sizeof(resp));
        if (n < 5) return false;

        uint8_t byteCount = resp[2];
        if (byteCount != qty * 2) {
            log("bad byte count");
            return false;
        }
        if (n < 3 + byteCount + 2) return false;
        if (!checkCRC(resp, 3 + byteCount + 2)) {
            log("bad CRC");
            return false;
        }
        memcpy(dest, &resp[3], byteCount);
        return true;
    }

    static float decodeFloat(const uint8_t *w, Y4000FloatOrder order) {
        // Host (Cortex-M0+) stores a float LSB-first, i.e. as D C B A.
        uint8_t h[4];
        switch (order) {
            case ORDER_ABCD: h[0]=w[3]; h[1]=w[2]; h[2]=w[1]; h[3]=w[0]; break;
            case ORDER_DCBA: h[0]=w[0]; h[1]=w[1]; h[2]=w[2]; h[3]=w[3]; break;
            case ORDER_CDAB: h[0]=w[1]; h[1]=w[0]; h[2]=w[3]; h[3]=w[2]; break;
            case ORDER_BADC: h[0]=w[2]; h[1]=w[3]; h[2]=w[0]; h[3]=w[1]; break;
            default:         h[0]=w[0]; h[1]=w[1]; h[2]=w[2]; h[3]=w[3]; break;
        }
        float f;
        memcpy(&f, h, 4);
        return f;
    }

    static uint16_t crc16(const uint8_t *buf, size_t len) {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < len; i++) {
            crc ^= buf[i];
            for (uint8_t b = 0; b < 8; b++)
                crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
        }
        return crc;
    }

  private:
    Stream          *_s        = nullptr;
    Stream          *_dbg      = nullptr;
    uint8_t          _addr     = 0x01;
    uint32_t         _timeoutMs = 500;
    int8_t           _dePin    = -1;
    Y4000FloatOrder  _order    = ORDER_DCBA;

    void log(const char *m) { if (_dbg) { _dbg->print(F("  [modbus] ")); _dbg->println(m); } }

    static void appendCRC(uint8_t *buf, size_t len) {
        uint16_t c = crc16(buf, len);
        buf[len]     = lowByte(c);   // Modbus sends CRC low byte first
        buf[len + 1] = highByte(c);
    }

    static bool checkCRC(const uint8_t *buf, size_t totalLen) {
        uint16_t c = crc16(buf, totalLen - 2);
        return buf[totalLen - 2] == lowByte(c) && buf[totalLen - 1] == highByte(c);
    }

    bool sendCommand(uint16_t reg) {
        uint8_t req[8];
        req[0] = _addr;
        req[1] = 0x03;
        req[2] = highByte(reg);
        req[3] = lowByte(reg);
        req[4] = 0x00;
        req[5] = 0x00;          // quantity = 0: Yosemitech's command form
        appendCRC(req, 6);

        uint8_t resp[16];
        return transact(req, 8, resp, sizeof(resp)) > 0;
    }

    // Write a frame, then collect the reply, discarding the local echo and
    // any leading garbage. Returns bytes placed in resp, or -1.
    int transact(const uint8_t *req, size_t reqLen, uint8_t *resp, size_t respLen) {
        while (_s->available()) _s->read();      // drain stale bytes

        if (_dePin >= 0) digitalWrite(_dePin, HIGH);   // transmit
        _s->write(req, reqLen);
        _s->flush();                             // block until fully shifted out
        if (_dePin >= 0) digitalWrite(_dePin, LOW);    // back to receive

        uint8_t  buf[80];
        size_t   n    = 0;
        uint32_t last = millis();

        // Read until the line goes quiet for ~4 character times, or we hit
        // the overall timeout. 4 chars @ 9600 8N1 is about 4.2 ms; 15 ms is
        // a safe idle gap.
        while (millis() - last < 15 && millis() - last < _timeoutMs) {
            if (_s->available()) {
                if (n < sizeof(buf)) buf[n++] = (uint8_t)_s->read();
                else                 (void)_s->read();
                last = millis();
            } else if (n == 0 && millis() - last > _timeoutMs) {
                break;
            }
        }

        if (n == 0) { log("no response"); return -1; }

        if (_dbg) {
            _dbg->print(F("  [modbus] rx:"));
            for (size_t i = 0; i < n; i++) {
                _dbg->print(buf[i] < 0x10 ? " 0" : " ");
                _dbg->print(buf[i], HEX);
            }
            _dbg->println();
        }

        // Skip the echoed request if the transceiver looped it back, then find
        // the first [addr][func] pair that starts the real reply.
        size_t start = 0;
        if (n >= reqLen && memcmp(buf, req, reqLen) == 0) start = reqLen;

        for (size_t i = start; i + 1 < n; i++) {
            if (buf[i] == _addr && (buf[i + 1] == 0x03 || buf[i + 1] == 0x83)) {
                size_t len = n - i;
                if (len > respLen) len = respLen;
                memcpy(resp, &buf[i], len);
                if (resp[1] & 0x80) {
                    log("modbus exception");
                    return -1;
                }
                return (int)len;
            }
        }
        log("no valid frame in response");
        return -1;
    }
};
