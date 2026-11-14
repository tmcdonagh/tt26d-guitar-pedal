<!---
This file is used to generate your project datasheet.
-->

## How it works

The design is an I2S **master** and the brain of a small digital guitar pedal
(a "silicon pedalboard"). It generates the audio clocks — MCLK 24.576 MHz
(512 fs) on `uio[0]`, BCLK 3.072 MHz (64 fs) on `uio[6]`, LRCLK 48 kHz on
`uio[7]` — and exchanges 16-bit signed, MSB-first Philips-I2S audio: an
external ADC slave returns digitized samples on `ui[0]`, and the processed
sample goes out on `uio[1]`. Data occupies slots 1–16 of each 32-slot
half-frame after the one-slot I2S delay (slots 17–31 stay low); the master
samples the ADC line mid-slot on BCLK rising edges, left half only, and
shifts the DAC word out MSB-first, changing it on BCLK falling edges. The
**same** processed sample is transmitted on both channel halves (mono
contract). All arithmetic is 16-bit saturating.

Each received sample is latched once per frame (48 kHz) and runs through a
serial effect chain — a disabled stage passes its input through bit-exactly:

**fuzz → distortion → crush → tremolo**

- **Octave fuzz** (`en[0]`): full-wave rectifier (octave doubler) with
  saturating |x| (−32768 folds to +32767) and a fixed clip at 16384. The
  output is always non-negative; fuzz has no param bit.
- **Distortion** (`en[1]`): soft-knee piecewise-linear shaper on the stage
  input's magnitude — unity below the knee T, slope 1/2 above it
  (out = T + (a−T)/2), original sign reapplied. P0 picks the knee:
  T = 8192 (soft ceiling 20479) or T = 4096 (ceiling 18431). Both knees sit
  below the fuzz clip ceiling (16384), so a stacked fuzz works the shaper
  across its knee.
- **Crush** (`en[2]`): auto-lofi bitcrusher — the quantize depth follows the
  input envelope, so quiet passages get crunchier. The envelope tracks the
  pre-chain input magnitude: attack env ← (env+mag)/2, release
  env ← env − env/256 − 1 (~256-frame ≈ 5.3 ms release, decaying to exactly
  0); loud = env ≥ 16384. The depth pair {P1, loud} picks how many bits are
  kept: 11 → 8 bits (mask 0xFF00), 01/10 → 6 bits (0xFC00), 00 → 4 bits
  (0xF000) — the sign bit survives the mask. The depth lags the envelope by
  exactly one frame (a sample's own magnitude never sets its own depth).
- **Tremolo** (`en[3]`): stepped-gain tremolo — out = in >> s with the gain
  step s driven by a triangle LFO (s = LFO gap >> 6). P2 sets the rate
  (~23.4 Hz / ~46.9 Hz), P3 the depth span (gains 1, 1/2, 1/4, 1/8 =
  0/−6/−12/−18 dB, or 1, 1/2 = 0/−6 dB).

Fuzz and distortion are pure functions of the latched sample; the crush
depth and tremolo gain follow per-frame state (envelope / LFO).

**Panel.** An EC11 rotary encoder on `ui[1]`/`ui[2]` selects one of 8
presets: quadrature decoded ×2 on accepted A edges (direction from the B
level), preset counter wraps mod 8 in both directions. All panel lines are
sampled once per audio frame and a level is accepted only after 48
consecutive equal samples (~1 ms bounce filter), so contact glitches are
rejected. The preset number shows on a 7-seg digit — active-high segments
a–g on `uo[0..6]`. A stomp switch on `ui[3]` toggles a **latching** bypass:
each press-release cycle flips it; while bypassed every enable is forced
off (clean pass-through) and the display goes blank. The decimal point
`uo[7]` is a clip LED: it lights when the output sample hits full scale
(0x7FFF or −32768) and holds ~43 ms (2048 frames).

**Enables** — each stage's enable is the OR of its preset-ROM bit and its
DIP switch (bit order = chain order), forced all-off while bypassed:

`en = bypass ? 4'b0000 : (ROM[preset] | ui[7:4])`

**Params** — one character bit per effect, P3..P0 on `uio[5:2]` (tables
below).

Preset ROM patterns, distortion knee constants, crush envelope shifts, and
tremolo rates are provisional — tuned by listening session.

## Signal table

| Pin | Dir | Function |
|-----|-----|----------|
| ui[0] | in | I2S ADC data in |
| ui[1] | in | Encoder A (ext. pull-down, common to 3V3) |
| ui[2] | in | Encoder B (ext. pull-down, common to 3V3) |
| ui[3] | in | Stomp switch (ext. pull-down, common to 3V3, latching) |
| ui[4] | in | Enable: fuzz |
| ui[5] | in | Enable: distortion |
| ui[6] | in | Enable: crush |
| ui[7] | in | Enable: tremolo |
| uo[0] | out | 7-seg segment a (active high) |
| uo[1] | out | 7-seg segment b (active high) |
| uo[2] | out | 7-seg segment c (active high) |
| uo[3] | out | 7-seg segment d (active high) |
| uo[4] | out | 7-seg segment e (active high) |
| uo[5] | out | 7-seg segment f (active high) |
| uo[6] | out | 7-seg segment g (active high) |
| uo[7] | out | dp = clip LED (peak hold ~43 ms) |
| uio[0] | out | MCLK (24.576 MHz output) |
| uio[1] | out | I2S DAC data out |
| uio[2] | in | Param P0: distortion character |
| uio[3] | in | Param P1: crush base depth |
| uio[4] | in | Param P2: tremolo rate |
| uio[5] | in | Param P3: tremolo depth |
| uio[6] | out | I2S BCLK (master, 3.072 MHz) |
| uio[7] | out | I2S LRCLK/WS (master, 48 kHz) |

uio pin directions: `uio_oe = 8'b1100_0011` — uio[7:6] and uio[1:0] are
outputs (LRCLK/BCLK, MCLK/DAC data), uio[5:2] are inputs (params).

### Preset ROM (enable bits in chain order: fuzz, dist, crush, trem)

| Preset (7-seg digit) | fuzz | dist | crush | trem |
|---|---|---|---|---|
| 0 (0) | | | | |
| 1 (1) | x | | | |
| 2 (2) | | x | | |
| 3 (3) | | | x | |
| 4 (4) | | | | x |
| 5 (5) | x | x | | |
| 6 (6) | x | x | x | |
| 7 (7) | x | x | x | x |

### Param bits (P3..P0 on uio[5:2])

| Param | Pin | Effect | 0 | 1 |
|---|---|---|---|---|
| P0 | uio[2] | Distortion knee T | 8192 (ceiling 20479) | 4096 (ceiling 18431) |
| P1 | uio[3] | Crush base depth | quiet 4 / loud 6 bits | quiet 6 / loud 8 bits |
| P2 | uio[4] | Tremolo rate | ~23.4 Hz | ~46.9 Hz |
| P3 | uio[5] | Tremolo depth | span 255: gains 1, 1/2, 1/4, 1/8 | span 127: gains 1, 1/2 |

### DIP enables (ui[7:4])

ui[4]=fuzz, ui[5]=distortion, ui[6]=crush, ui[7]=tremolo — OR'd with the
preset ROM bits, all forced off while the stomp bypass is engaged.

## How to test

Hook up an I2S codec slave with all four I2S lines on uio: MCLK `uio[0]`,
BCLK `uio[6]`, LRCLK `uio[7]`, DAC data `uio[1]`, and ADC data on `ui[0]`
— e.g. a PCM1808 + PCM5102, or a Pico programmed as an I2S slave to inject
and capture audio. Play some music: turning the encoder walks presets 0–7,
shown on the 7-seg digit (preset 0 = all stages off). The stomp switch
blanks the display and passes audio through clean; the next press-release
re-engages the chain. Drive a full-scale sample (easiest with the chain
bypassed, or through crush, which keeps the sign bit) and the dp clip LED
lights with its ~43 ms hold.

Without hardware, the cocotb suite covers the I2S framing, every chain
stage and the enable matrix, the panel (encoder direction/wrap/bounce,
stomp latch, 7-seg + ROM), the auto-lofi envelope, and the clip LED:
`cd test && make -B`.

## External hardware

- **I2S codec**: PCM1808 ADC + PCM5102 DAC, configured as I2S **slaves**;
  the PCM1808's SCKI comes from `uio[0]` (MCLK, 24.576 MHz).
- **EC11 encoder + stomp switch**: external 10 kΩ pull-downs to GND are
  **required** on `ui[1]`, `ui[2]`, `ui[3]`, with the encoder and stomp
  switch commons tied to 3.3 V — the ASIC has no internal pull resistors on
  ui pins, so the lines float without them. Idle-low wiring also matches the
  panel's reset state, so the pedal boots at preset 0, unbypassed, with no
  spurious edges.
- **Display**: the Tiny Tapeout demoboard's onboard 7-seg digit, attached
  to `uo[0..7]` via the demoboard solder jumpers (segments active high).
- **Guitar front end**: a high-impedance JFET-input preamp (TL072) with
  mid-rail bias ahead of the ADC input.
- **Controls**: DIP switches (or a Pico) drive the enables `ui[7:4]` and
  the params `uio[5:2]`.
