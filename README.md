# Silicon Guitar Pedal

## My attempt at a guitar pedal within a 1x1 Tiny Tapeout digital design.

Since the tile itself is digital I chose to use an external ADC and DAC to turn
the guitar's analog signal into digital values that the tile can act on, then
it can turn it back into an analog signal with the DAC. Apparently I2S is not too
difficult to implement so that's what I'm hoping I can go with.

The design is the I2S master (MCLK 24.576 MHz, BCLK 3.072 MHz, LRCLK 48 kHz —
all on `uio`) and runs a serial effect chain over the digitized audio:

**fuzz (rectify + clip) → distortion (soft knee) → crush (auto-lofi) → tremolo (stepped gain)**

Each stage has an enable bit: a preset ROM — selected by an EC11 encoder and
shown on a 7-seg digit on `uo` — OR'd with DIP switches on `ui[7:4]`, all
forced off by the latching stomp bypass on `ui[3]`. One character bit per
effect rides `uio[5:2]` (P0 distortion knee, P1 crush base depth, P2 tremolo
rate, P3 tremolo depth). The 7-seg decimal point is a clip LED (~43 ms peak
hold).

| Preset | Chain |
|---|---|
| 0 | all stages off |
| 1 | fuzz |
| 2 | distortion |
| 3 | crush |
| 4 | tremolo |
| 5 | fuzz → distortion |
| 6 | fuzz → distortion → crush |
| 7 | fuzz → distortion → crush → tremolo |

Preset patterns and effect constants are provisional — tuned by ear.

See [docs/info.md](docs/info.md) for the full datasheet: pin map, per-effect
behavior, preset/param tables, and how to test.

## AI Usage
This project uses AI heavily. Specifically I've been experimenting with open weight
models provided through API access like Kimi K2.7 Code and GLM-5.3-Flash.
