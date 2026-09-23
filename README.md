<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Y4000 Modbus test harness

Two Arduino Megas: one impersonates a Yosemitech Y4000 multiparameter water
quality sonde, the other polls it over Modbus RTU. Lets you develop and debug
the protocol layer with no sensor, no 12 V supply and nothing wet.

Part of a larger project reading a Y4000 into a Meshtastic LoRa mesh on a
RAKwireless RAK11300. The master here uses the same `Y4000Modbus.h` driver that
runs on the RAK, so whatever you prove on the bench carries over.

**Status: works.** Verified over both a direct TTL cross-connection and MAX485
transceivers. Not yet tested against a real sonde.

```
Y4000_mock_sonde/   the fake sonde (Modbus RTU slave, with fault injection)
Y4000_master/       the polling master + Y4000Modbus.h
```

The mock has no `.h` file on purpose. It implements CRC separately from the
master, so the two sides agreeing is real evidence rather than two copies of the
same possible mistake.

## Wiring

**Simplest — direct TTL, no RS485 hardware.** Enough to shake out framing, CRCs,
timing and byte order:

```
Mega A pin 18 (TX1) ----> Mega B pin 19 (RX1)
Mega A pin 19 (RX1) <---- Mega B pin 18 (TX1)
Mega A GND ------------- Mega B GND
```

Leave `DE_PIN` at `-1` in both sketches.

**Closer to the real thing — two MAX485 breakouts.** Add this to exercise the
half-duplex turnaround:

```
Mega    MAX485          MAX485   Mega
18 -----> DI               DI <----- 18
19 <----- RO               RO -----> 19
 7 -----+-> DE           DE <-+----- 7
        +-> RE           RE <-+
5V -----> VCC             VCC <----- 5V
GND ----> GND             GND <----- GND
          A ------------- A
          B ------------- B
          GND ----------- GND
```

Set `#define DE_PIN 7` in **both** sketches. DE and RE get jumpered together on
each board. Both boards on 5 V — MAX485 is the 5 V part, don't substitute a
MAX3485.

The GND between the two modules is not optional. RS485 is differential but still
needs a common-mode reference.

## Running it

Flash the mock first and open its serial monitor at 115200. Flash the master to
the other Mega and open a second monitor. You should see request and reply frames
on both sides, and eight drifting values on the master.

The first poll returns all zeros. That's correct — the mock only produces data
after a start-measurement, same as the real sonde.

## Fault injection

Type these into the **mock's** serial monitor:

| key | effect |
|---|---|
| `s` / `m` | start / stop measuring |
| `o` | cycle float byte order (ABCD → DCBA → CDAB → BADC) |
| `d` | drop the next reply entirely |
| `e` | reply with Modbus exception 0x02 |
| `g` | corrupt the CRC on the next reply |

`o` is the useful one. The master prints all four decodings of the first four
parameters every poll — press `o` and watch which column tracks the mock's real
values. That's a direct test of byte-order handling, which is the thing most
likely to be silently wrong against a real sonde. Wrong order on a DCBA stream
mostly yields denormals that print as `0.000`, and all-zero reads look identical
to an uninstalled probe.

`d`, `e` and `g` exercise the master's error paths. All three should produce a
clean `read FAILED` and recovery on the next poll, not a hang.

## Fidelity

Byte-accurate against the frames in Yosemitech's Modbus manual:

```
start measurement   ->  01 03 25 00 00 00 4E C6
ack                 <-  01 03 00 20 F0
read values         ->  01 03 26 00 00 10 4F 4E
```

Note that ack: function `0x03`, byte count zero. Yosemitech overloads the read
function as a command by sending a register quantity of zero, and the reply
doesn't follow normal Modbus response layout. This is much of why stock Modbus
libraries choke on these sondes, and it's worth having in a mock rather than
meeting it in the field.

What the mock does **not** reproduce: warm-up time, wiper current draw, optical
probe settling, or electrical noise. Values drift on clean sine waves.

## Debugging note

If you see corrupt bytes, check *which* bytes. A bus that can't hold a long low
state corrupts `0x00` and leaves everything else intact, because `0x00` is the
only byte with no bit transitions — start bit plus eight zeros is nine
consecutive low bit-times. Bytes arriving as `0xC0` / `0xE0` / `0xF0` mean the
line drifted to idle partway through. Usual causes: a floating DE pin, a missing
GND between modules, or a transceiver running at 3.3 V.

Corruption by bit *position* rather than by byte value would point at a baud
mismatch instead.

## License

GPL-3.0-or-later. See [LICENSE](LICENSE).

## Trademarks

Yosemitech, RAKwireless and Meshtastic® are trademarks of their respective
owners. This project is not affiliated with or endorsed by any of them.

## Acknowledgements

Parameter ordering cross-checked against
[EnviroDIY/YosemitechModbus](https://github.com/EnviroDIY/YosemitechModbus),
which implements the full Y4000 command set and ships Yosemitech's Modbus manual
in its `doc/` folder.
