# Silicon Guitar Pedal

## My attempt at a guitar pedal within a 1x1 Tiny Tapeout digital design.

Since the tile itself is digital I chose to use an external ADC and DAC to turn
the guitar's analog signal into digital values that the tile can act on, then
it can turn it back into an analog signal with the DAC. Apparently I2S is not too
difficult to implement so that's what I'm hoping I can go with.

## AI Usage
This project uses AI heavily. Specifically I've been experimenting with open weight
models provided through API access like Kimi K2.7 Code and GLM-5.3-Flash.

| Mode (ui[3:1]) | Effect |
|---|---|
| 0 | Bypass |
| 1 | Octave fuzz (full-wave rectify + clip) |
| 2 | Distortion (drive gain + hard clip) |
| 3 | Ring modulator (square-wave LFO sign multiply) |
| 4 | Tremolo (stepped-gain triangle LFO: 0/−6/−12/−18 dB) |
| 5 | Bitcrusher (bit-depth quantize + sample-hold decimate) |
| 6 | Fuzz → distortion chain |
| 7 | Test tone (internal square wave, no input needed) |

Parameters ride `ui[7:4]` (per-effect meaning in [docs/info.md](docs/info.md)).
A 5-LED peak level meter with per-sample decay drives `uo[5:1]`.

See [docs/info.md](docs/info.md) for pinout and testing details.
