// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Chr0myl

/*
 * Y4000_mock_sonde.ino  --  Arduino Mega 2560
 *
 * Pretends to be a Yosemitech Y4000 multiparameter sonde on a Modbus RTU bus.
 * Speaks the real protocol, including Yosemitech's non-standard "function 0x03
 * with quantity 0" command form, so your master code needs no special casing.
 *
 * Verified against the frames in the Yosemitech Modbus manual:
 *   start measurement  ->  01 03 25 00 00 00 4E C6
 *   reply              <-  01 03 00 20 F0
 *   read values        ->  01 03 26 00 00 10 4F 4E
 *
 * Bus  : Serial1  (TX1 = pin 18, RX1 = pin 19)
 * Debug: Serial   (USB, 115200)
 *
 * Type single-letter commands into the serial monitor to inject faults --
 * see handleConsole() at the bottom.
 */

#include <Arduino.h>

// ---------------------------------------------------------------- config ---

#define BUS            Serial1
#define BUS_BAUD       9600
#define SLAVE_ADDRESS  0x01

// Set to the DE/!RE pin if you are using a MAX485 breakout.
// Leave at -1 for a direct TTL cross-connection between two Megas.
#define DE_PIN         -1

// Which byte order to encode floats in. Change this to prove your master's
// decoder actually detects the difference.
//   0 = ABCD (big endian)   1 = DCBA (what Yosemitech uses)
//   2 = CDAB (word swap)    3 = BADC (byte swap)
uint8_t floatOrder = 1;

// ----------------------------------------------------------- sonde state ---

// Registers the real sonde uses.
const uint16_t REG_VALUES     = 0x2600;   // 16 registers = 8 floats
const uint16_t REG_START_MEAS = 0x2500;
const uint16_t REG_STOP_MEAS  = 0x2E00;
const uint16_t REG_BRUSH      = 0x2F00;

bool measuring   = false;
bool muteNext    = false;    // drop one reply, to test master timeouts
bool exceptNext  = false;    // reply with a Modbus exception once
bool garbleNext  = false;    // corrupt one CRC

// Baseline values, drifted slowly so you can see them move.
//  [0] DO mg/L  [1] Turbidity NTU  [2] Cond mS/cm  [3] pH
//  [4] Temp C   [5] ORP mV         [6] Chl ug/L    [7] BGA ug/L
const float BASE[8] = { 8.50f, 12.0f, 0.450f, 7.20f, 18.50f, 220.0f, 3.50f, 0.80f };
const float SWING[8] = { 0.60f,  3.0f, 0.030f, 0.25f,  1.20f,  25.0f, 0.90f, 0.20f };

// ------------------------------------------------------------ modbus i/o ---

uint16_t crc16(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
    }
    return crc;
}

void sendFrame(uint8_t *buf, size_t len)
{
    if (muteNext) {
        muteNext = false;
        Serial.println(F("  [fault] dropping this reply"));
        return;
    }

    uint16_t c = crc16(buf, len);
    buf[len]     = lowByte(c);
    buf[len + 1] = highByte(c);

    if (garbleNext) {
        garbleNext = false;
        buf[len] ^= 0xFF;
        Serial.println(F("  [fault] corrupting CRC on this reply"));
    }

    // Real Modbus slaves wait ~3.5 character times before answering. At 9600
    // baud that is about 3.6 ms. Without this some masters see the reply as
    // part of their own echo.
    delay(5);

    if (DE_PIN >= 0) digitalWrite(DE_PIN, HIGH);
    BUS.write(buf, len + 2);
    BUS.flush();
    if (DE_PIN >= 0) digitalWrite(DE_PIN, LOW);

    Serial.print(F("  tx:"));
    for (size_t i = 0; i < len + 2; i++) {
        Serial.print(buf[i] < 0x10 ? " 0" : " ");
        Serial.print(buf[i], HEX);
    }
    Serial.println();
}

void sendException(uint8_t function, uint8_t code)
{
    uint8_t r[5];
    r[0] = SLAVE_ADDRESS;
    r[1] = function | 0x80;
    r[2] = code;
    sendFrame(r, 3);
}

// --------------------------------------------------------- value encoder ---

void encodeFloat(float f, uint8_t *out)
{
    uint8_t h[4];
    memcpy(h, &f, 4);          // AVR stores a float LSB first: D C B A

    switch (floatOrder) {
        case 0: out[0]=h[3]; out[1]=h[2]; out[2]=h[1]; out[3]=h[0]; break; // ABCD
        case 1: out[0]=h[0]; out[1]=h[1]; out[2]=h[2]; out[3]=h[3]; break; // DCBA
        case 2: out[0]=h[1]; out[1]=h[0]; out[2]=h[3]; out[3]=h[2]; break; // CDAB
        case 3: out[0]=h[2]; out[1]=h[3]; out[2]=h[0]; out[3]=h[1]; break; // BADC
    }
}

float currentValue(uint8_t i)
{
    if (!measuring) return 0.0f;
    // Each parameter drifts on its own period so they don't move in lockstep.
    float phase = (millis() / 1000.0f) * (0.05f + i * 0.011f);
    return BASE[i] + SWING[i] * sin(phase);
}

// -------------------------------------------------------- request router ---

void handleRequest(uint8_t *req, size_t len)
{
    if (len < 4) return;

    // CRC check. A real slave silently ignores a bad frame.
    uint16_t c = crc16(req, len - 2);
    if (req[len - 2] != lowByte(c) || req[len - 1] != highByte(c)) {
        Serial.println(F("  bad CRC, ignoring"));
        return;
    }

    if (req[0] != SLAVE_ADDRESS && req[0] != 0xFF) return;   // not for us

    uint8_t function = req[1];
    if (function != 0x03) {
        sendException(function, 0x01);   // illegal function
        return;
    }

    uint16_t reg = ((uint16_t)req[2] << 8) | req[3];
    uint16_t qty = ((uint16_t)req[4] << 8) | req[5];

    if (exceptNext) {
        exceptNext = false;
        Serial.println(F("  [fault] replying with exception 0x02"));
        sendException(function, 0x02);
        return;
    }

    // ---- Yosemitech command form: function 0x03 with quantity 0 -----------
    if (qty == 0) {
        switch (reg) {
            case REG_START_MEAS:
                measuring = true;
                Serial.println(F("  START measurement"));
                break;
            case REG_STOP_MEAS:
                measuring = false;
                Serial.println(F("  STOP measurement"));
                break;
            case REG_BRUSH:
                Serial.println(F("  brush activated"));
                break;
            default:
                sendException(function, 0x02);
                return;
        }
        // The documented ack is 01 03 00 20 F0 -- byte count of zero.
        uint8_t r[5];
        r[0] = SLAVE_ADDRESS;
        r[1] = 0x03;
        r[2] = 0x00;
        sendFrame(r, 3);
        return;
    }

    // ---- Normal read of the measurement block -----------------------------
    if (reg == REG_VALUES && qty == 16) {
        uint8_t r[40];
        r[0] = SLAVE_ADDRESS;
        r[1] = 0x03;
        r[2] = 32;                       // byte count
        for (uint8_t i = 0; i < 8; i++)
            encodeFloat(currentValue(i), &r[3 + i * 4]);

        Serial.print(F("  read values, measuring="));
        Serial.println(measuring ? F("yes") : F("NO (all zeros)"));
        sendFrame(r, 35);
        return;
    }

    // Partial reads of the block are legal Modbus; support them too.
    if (reg >= REG_VALUES && reg + qty <= REG_VALUES + 16) {
        uint8_t block[32];
        for (uint8_t i = 0; i < 8; i++)
            encodeFloat(currentValue(i), &block[i * 4]);

        uint8_t r[40];
        r[0] = SLAVE_ADDRESS;
        r[1] = 0x03;
        r[2] = qty * 2;
        memcpy(&r[3], &block[(reg - REG_VALUES) * 2], qty * 2);
        sendFrame(r, 3 + qty * 2);
        return;
    }

    Serial.print(F("  unknown register 0x"));
    Serial.println(reg, HEX);
    sendException(function, 0x02);       // illegal data address
}

// ------------------------------------------------------------------ main ---

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 5000) delay(10);

    if (DE_PIN >= 0) {
        pinMode(DE_PIN, OUTPUT);
        digitalWrite(DE_PIN, LOW);
    }
    BUS.begin(BUS_BAUD, SERIAL_8N1);

    Serial.println(F("\n=== Mock Yosemitech Y4000 sonde ==="));
    Serial.print(F("Slave address 0x"));
    Serial.print(SLAVE_ADDRESS, HEX);
    Serial.print(F(", "));
    Serial.print(BUS_BAUD);
    Serial.println(F(" 8N1 on Serial1 (TX1=18, RX1=19)"));
    Serial.println(F("Console: s=start m=stop o=cycle byte order"));
    Serial.println(F("         d=drop one reply  e=exception  g=bad CRC"));
    Serial.println(F("Waiting for a master...\n"));
}

void loop()
{
    handleConsole();

    static uint8_t  buf[64];
    static size_t   n    = 0;
    static uint32_t last = 0;

    while (BUS.available()) {
        if (n < sizeof(buf)) buf[n++] = BUS.read();
        else                 BUS.read();
        last = millis();
    }

    // A Modbus frame ends when the line has been idle for 3.5 character times.
    // 4 ms is comfortable at 9600 baud.
    if (n > 0 && millis() - last > 4) {
        Serial.print(F("rx:"));
        for (size_t i = 0; i < n; i++) {
            Serial.print(buf[i] < 0x10 ? " 0" : " ");
            Serial.print(buf[i], HEX);
        }
        Serial.println();

        handleRequest(buf, n);
        n = 0;
    }
}

void handleConsole()
{
    if (!Serial.available()) return;

    const char *ORDER_NAME[4] = {"ABCD", "DCBA", "CDAB", "BADC"};

    switch (Serial.read()) {
        case 's': measuring = true;  Serial.println(F("> measuring")); break;
        case 'm': measuring = false; Serial.println(F("> stopped"));   break;
        case 'd': muteNext = true;   Serial.println(F("> will drop next reply")); break;
        case 'e': exceptNext = true; Serial.println(F("> will except next reply")); break;
        case 'g': garbleNext = true; Serial.println(F("> will corrupt next CRC")); break;
        case 'o':
            floatOrder = (floatOrder + 1) & 3;
            Serial.print(F("> float order now "));
            Serial.println(ORDER_NAME[floatOrder]);
            break;
    }
}
