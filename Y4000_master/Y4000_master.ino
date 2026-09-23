// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 Chr0myl

/*
 * Y4000_master.ino  --  Arduino Mega 2560
 *
 * Polls a Y4000 (real or the mock sonde sketch) over Modbus RTU and prints
 * the readings.
 *
 * This uses the exact same Y4000Modbus.h driver you will run on the RAK11300,
 * so whatever you prove here carries over. Copy Y4000Modbus.h next to this
 * .ino file (the Arduino IDE expects it in the sketch folder).
 *
 * Bus  : Serial1  (TX1 = pin 18, RX1 = pin 19)
 * Debug: Serial   (USB, 115200)
 */

#include <Arduino.h>
#include "Y4000Modbus.h"

// ---------------------------------------------------------------- config ---

#define BUS            Serial1
#define BUS_BAUD       9600
#define SONDE_ADDRESS  0x01

// DE/!RE pin if you are using a MAX485 breakout.
// Leave at -1 for a direct TTL cross-connection between two Megas.
#define DE_PIN         -1

#define POLL_MS        5000

// Against a real sonde this is 20 s or so. The mock replies instantly, so
// keep it short while you are bench testing.
#define WARMUP_MS      1000

Y4000Modbus sonde;

const char *PARAM_NAME[8] = {
    "DO        (mg/L) ",
    "Turbidity (NTU)  ",
    "Cond      (mS/cm)",
    "pH               ",
    "Temp      (C)    ",
    "ORP       (mV)   ",
    "Chl       (ug/L) ",
    "BGA       (ug/L) "
};

// ----------------------------------------------------------------- setup ---

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 5000) delay(10);

    Serial.println(F("\n=== Y4000 Modbus master (Arduino Mega) ==="));

    BUS.begin(BUS_BAUD, SERIAL_8N1);

    sonde.begin(&BUS, SONDE_ADDRESS);
    sonde.setDePin(DE_PIN);
    sonde.setDebug(&Serial);        // prints every raw frame; comment out later
    sonde.setTimeout(1000);
    sonde.setFloatOrder(ORDER_DCBA);

    delay(WARMUP_MS);

    Serial.println(F("Sending start-measurement..."));
    if (sonde.startMeasurement())
        Serial.println(F("  acked"));
    else
        Serial.println(F("  no ack (some units measure continuously)"));

    delay(500);
}

// ------------------------------------------------------------------ loop ---

void loop()
{
    float   vals[8];
    uint8_t raw[32];

    Serial.println(F("\n--- poll ---"));

    if (sonde.readValues(vals, raw)) {
        for (uint8_t i = 0; i < 8; i++) {
            Serial.print(F("  "));
            Serial.print(PARAM_NAME[i]);
            Serial.print(F(" = "));
            Serial.println(vals[i], 3);
        }
        dumpAllByteOrders(raw);
    } else {
        Serial.println(F("  read FAILED"));
    }

    delay(POLL_MS);
}

/*
 * Prints every candidate interpretation of the first four parameters. Press
 * 'o' on the mock sonde to change its byte order and watch which column
 * follows it -- that proves your decoder is doing what you think.
 *
 * Delete this once you have settled the order with a real sonde.
 */
void dumpAllByteOrders(const uint8_t *raw)
{
    const char *names[4] = {"ABCD", "DCBA", "CDAB", "BADC"};
    Serial.println(F("  byte-order check (first 4 params):"));
    for (uint8_t o = 0; o < 4; o++) {
        Serial.print(F("    "));
        Serial.print(names[o]);
        Serial.print(F(": "));
        for (uint8_t p = 0; p < 4; p++) {
            Serial.print(Y4000Modbus::decodeFloat(&raw[p * 4], (Y4000FloatOrder)o), 3);
            Serial.print(F("  "));
        }
        Serial.println();
    }
}
