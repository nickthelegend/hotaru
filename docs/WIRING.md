# HOTARU — wiring

**Board: ESP32-S3 N16R8** (ROBODUINO, dual USB-C) — 16 MB flash, 8 MB **octal**
PSRAM. This is the board SPEC.md always assumed, and the one the base's board
bay is cut for.

This map shares almost nothing with a classic ESP32's. Do not port pins
between them.

Every pin below is defined once, in `config.h`. Change it there, nowhere else.

---

## 1. Power — read this before plugging anything in

The 5 V from the charger splits into **two legs at the connector**. They share
ground and nothing else.

```
  USB-C 5V/3A ──┬── leg A ──────────────► ESP32  5V (VIN) pin
                │
                ├── leg B ──┬───────────► servo bus V+  (all 4 reds)
                │           │
                │         2200uF          ◄── at the BRANCH, not back at
                │           │                 the charger
                │
                └── leg C ──┬─[1N4001]──► WS2812 ring 5V
                            │
                          1000uF
                            │
  USB-C GND ────┴───────────┴───────────► COMMON GROUND (everything)
```

The ring gets its **own leg**, not a tap off the servo bus. Four MG996R
dragging the rail down every time a joint moves is exactly the noise the ring
sits 300 mm downstream of, and leg C keeps it out of that (and out of the
ESP32's VBUS trace).

- **Common ground is mandatory.** A servo whose ground is not the ESP32's
  ground sees no valid pulse and just buzzes. This is the single most common
  failure on this build.
- **Never feed servos from the ESP32's 3V3 regulator.** The 5 V pin is wired
  to USB VBUS and bypasses the regulator, which is why leg A is safe.
- **Never let a joint stall.** One MG996R stalled is 2.5 A; four is 10 A.
  `config.h` carries software end stops on every joint for this reason.

### Current budget

| Load | Idle | Moving | Stalled |
|---|---|---|---|
| 4x MG996R | ~40 mA | ~2 A | 10 A |
| ESP32 (WiFi on) | 250 mA | 250 mA | — |
| WS2812 x16 @ cap 80/255 | 10 mA | 350 mA | — |
| MAX98357A + 4R driver | 5 mA | 500 mA peak | — |

Realistic worst case is around **3 A**, which is exactly what a 5 V/3 A
charger gives. It works, but there is no headroom — move one joint at a time,
or fit a 5 V/5 A supply if you want all four moving together.

### Rails per part

| Part | Rail | Note |
|---|---|---|
| ESP32 | 5 V leg A | into the `5V`/`VIN` pin |
| 4x MG996R | 5 V leg B | with the 2200 uF at the branch |
| MAX98357A | 5 V | 5 V for full output into 4R |
| **INMP441** | **3.3 V** | **3.3 V part. 5 V will kill it.** |
| WS2812 ring | 5 V, own leg | diode only if LED #1 misbehaves |

---

## 2. Servos — MG996R x4

Wire colours: **brown = GND, red = V+, orange = signal.**

All four reds go to servo bus V+. All four browns go to common ground. Only
the orange signal wires differ:

| Joint | Where it physically sits | Height (z) | Signal |
|---|---|---|---|
| **pan** | in the tub — **the bottom one**, spline pointing up | 2.4–45.3 | **GPIO 4** |
| **shoulder** | in the tower on the turntable disc | 64.8–105.5 | **GPIO 5** |
| **elbow** | inside link1's far end — **the middle one** | 164.8–205.5 | **GPIO 6** |
| **head** | in the head block behind the cone — **the top one** | 254.8–295.5 | **GPIO 7** |

Why these four: on the S3, **GPIO 22–25 do not exist at all** (the S3 is 0–21
and 26–48), 26–32 are the in-package SPI flash, and **33–37 are the octal
PSRAM** on an -R8 part. 4/5/6/7 are free, adjacent, and land next to each other
on the header, so the four servo leads leave the board as one tidy bundle.

---

## 3. WS2812 LED ring (16 x 5050)

| Ring pad | Goes to |
|---|---|
| **DI** | GPIO **15**, through a **330 R** series resistor |
| **5V** | 5 V, on its own leg — *not* the servo bus. Diode optional, see below |
| **GND** | common ground |
| **DO** | **leave unconnected** — it only chains a second ring |

Also fit **1000 uF across the ring's 5V/GND**, close to the ring.

**Do you need a level shifter? Probably not — test first.**

On paper, yes: a WS2812 on 5 V wants DI above 0.7 x VDD = **3.5 V**, and the
ESP32 drives **3.3 V**. Out of spec.

In practice it almost always works, and one detail makes trying it low-risk:
**only LED #1 ever sees the 3.3 V signal.** Every WS2812 regenerates the data
internally and passes it on at a full 5 V level, so you are fighting a single
chip's input threshold, not sixteen. If the first pixel latches, the other
fifteen are guaranteed clean.

So: wire it straight, run `r` and `L` for a minute, and **watch LED #1
specifically**. Flicker, a wrong colour, or an occasional lag on that one pixel
while the rest look fine is the threshold talking. Anything else is a different
fault.

If it does misbehave, two fixes, better one first:

- **74AHCT125 buffer on DI** — shifts to a true 5 V logic level. Correct fix.
- **1N4001 in series on the ring's 5 V leg** — drops it to ~4.3 V so the
  threshold falls to ~3.0 V. Works, but it carries the full ring current,
  dims the ring slightly, and adds a part that can fail mid-arm. Use a
  **1N4001** (1 A), never a 1N4148 (200 mA, it will cook), and not a Schottky
  (only ~0.35 V, leaving you marginal again).

The failure is visible and harmless — wrong colours, not damage — so nothing
burns out while you find out.

**Why the 1000 uF.** Two jobs: it absorbs the connect-time inrush that can kill
the first pixel, and — the one that matters here — it is a local reservoir. The
ring shares 5 V with four servos that yank the rail down on every move. Local
capacitance holds the ring's supply up while they thrash.

---

## 4. INMP441 microphone — I2S port 0

| Pin | Goes to |
|---|---|
| VDD | **3.3 V** (not 5 V) |
| GND | common ground |
| SCK | GPIO **10** |
| WS | GPIO **11** |
| SD | GPIO **12** |
| L/R | **GND** = left channel |

Unlike the classic ESP32 the S3 has no input-only pins, so SD is just an
ordinary GPIO here.

If `L/R` goes to VDD instead of GND the mic lands in the right slot and you
read pure silence. That is the usual "my mic is dead".

---

## 5. MAX98357A amplifier — I2S port 1

| Pin | Goes to |
|---|---|
| VIN | **5 V** |
| GND | common ground |
| BCLK | GPIO **13** |
| LRC | GPIO **14** |
| DIN | GPIO **21** |
| GAIN | leave floating = 9 dB |
| SD | leave floating = enabled. Tie to GND and it shuts down. |

**Speaker** (4 R 2 W, 2040): its two wires to the amp's **+** and **−**
screw terminals. No polarity that matters for one driver.

---

## 6. MX switch

| Leg | Goes to |
|---|---|
| either | GPIO **16** |
| other | common ground |

No polarity. `INPUT_PULLUP` in firmware, so the pin idles HIGH and a press
reads LOW. 25 ms debounce.

---

## 7. Complete pin map

| GPIO | Direction | Connected to |
|---|---|---|
| 4 | out | pan servo signal |
| 5 | out | shoulder servo signal |
| 6 | out | elbow servo signal |
| 7 | out | head servo signal |
| 10 | out | INMP441 SCK |
| 11 | out | INMP441 WS |
| 12 | in | INMP441 SD |
| 13 | out | MAX98357A BCLK |
| 14 | out | MAX98357A LRC |
| 15 | out | WS2812 DI (330 R) |
| 16 | in, pullup | MX switch |
| 21 | out | MAX98357A DIN |

**Free if you need more:** 1, 2, 17, 18, 38, 47 — plus 39–42 if you can give up
JTAG. **8 and 9 are deliberately left alone**: they are the S3's default I2C
pair, so an IMU or a ToF sensor drops straight on later.

**Never use:** 26–32 (SPI flash), 33–37 (octal PSRAM on -R8), 19/20 (native
USB), 43/44 (UART0), 0/3/45/46 (strapping), 48 (onboard RGB LED).
**22, 23, 24 and 25 do not exist on the S3.**

---

## 8. Bring-up order

Do these in order. Each step proves one thing, so a failure tells you where it is.

1. **ESP32 alone**, USB only. `arduino-cli monitor`, expect the banner and `?`.
   Use the **UART-bridge** USB-C port (`/dev/cu.usbserial-*`) — it always works.
   The native-USB port needs `CDCOnBoot=cdc`, which the FQBN sets.
2. **One servo**, pan on GPIO 4, on the servo leg with the cap. Send `p60`,
   `p120`. If the board resets, the rail is sagging — that is power, not code.
3. **Remaining three servos.** `1` home, `2` rest, `3` alert.
4. **LED ring.** `L` lamp, `r` spin. Flicker = the diode is missing.
5. **Amp + speaker.** `t` for a tone.
6. **Mic.** `m` for the level meter on the ring, then talk to it.
7. **MX key.** Press it — it chirps and toggles the wave animation.

`i` prints live status of everything at any point.
