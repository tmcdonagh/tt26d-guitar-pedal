# Silicon Guitar Pedal

## My attempt at a guitar pedal within a 1x1 Tiny Tapeout digital design.

Since the tile itself is digital I chose to use an external ADC and DAC to turn
the guitar's analog signal into digital values that the tile can act on, then
it can turn it back into an analog signal with the DAC. Apparently I2S is not too
difficult to implement so that's what I'm hoping I can go with.

The design is the I2S master (MCLK 24.576 MHz, BCLK 3.072 MHz, LRCLK 48 kHz —
all on `uio`) and runs a serial effect chain over the digitized audio:

**fuzz (rectify + clip) → distortion (soft knee) → crush (auto-lofi) → tremolo (stepped gain)**

Each stage has an enable bit wired **directly** to a DIP switch on
`ui[7:4]` (bit order = chain order, all-off = the "digital clean" mode).
One character bit per effect rides `uio[5:2]` (P0 distortion knee, P1 crush
base depth, P2 tremolo rate, P3 tremolo depth). True bypass is an external
3PDT footswitch — the ASIC has no bypass role. The 7-seg decimal point on
`uo[7]` is a clip LED (~43 ms peak hold); `ui[3:1]` and `uo[6:0]` are
reserved for a future 2×2-tile v3 project.

Effect constants are provisional — tuned by ear.

See [docs/info.md](docs/info.md) for the full datasheet: pin map, per-effect
behavior, param tables, true-bypass wiring, and how to test.

## AI Usage
This project uses AI heavily. Specifically I've been experimenting with open weight
models provided through API access like Kimi K2.7 Code and GLM-5.3-Flash.
