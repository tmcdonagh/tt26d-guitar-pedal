# Silicon Guitar Pedal

## My attempt at a guitar pedal within a 2x2 Tiny Tapeout digital design.

Since the tile itself is digital I chose to use an external ADC and DAC to turn
the guitar's analog signal into digital values that the tile can act on, then
it can turn it back into an analog signal with the DAC. Apparently I2S is not too
difficult to implement so that's what I went with.

The design is the I2S master (MCLK 24.576 MHz, BCLK 3.072 MHz, LRCLK 48 kHz,
all on `uio`) and runs a serial effect chain over the digitized audio:

**compressor -> fuzz (with octave switch) -> distortion -> tone + noise gate -> phaser (stub) -> tremolo**

- **Compressor**: smoothed envelope, 1.5 dB gain table, slewed gain.
- **Fuzz**: asymmetric clip; an octave switch on `ui[7]`.
- **Distortion**: diode knee, drive x4 by default.
- **Tone + gate**: a low-pass and a noise gate, active automatically when
  fuzz or distortion is on.
- **Phaser**: an empty slot for now (the enable does nothing).
- **Tremolo**: smooth, 75% depth, about 5 Hz by default.

The effects are switched by plain switches on `ui[1]`-`ui[5]` (comp, fuzz,
dist, phaser, trem), debounced on chip, so an effect engages within about
5-11 ms. They need external pull-downs, with the switch commons to 3V3.
Three optional knobs sit on `uio[2]`-`uio[4]`, read by RC timing (a pot and a
capacitor per pin, no ADC chip): K1 tremolo rate (about 0.73-11.6 Hz), K2
post-dirt tone, K3 distortion drive. With no knobs fitted the defaults
apply (tie unused knob pins to GND). `ui[6]` and the SPI pins are
reserved (CS idles high). The decimal point on `uo[7]` is a clip LED. True
bypass is an external 3PDT footswitch; the ASIC has no bypass role.

The clock can be any integer-divided clock near 24.576 MHz; the sample rate is
clk / 512.

See [docs/info.md](docs/info.md) for the full datasheet: pin map, per-effect
behavior, knob circuit, and how to test.

## AI Usage
This project uses AI heavily. The earlier work used open weight models
provided through API access, Kimi K2.7 Code and GLM-5.3-Flash, via opencode.
The latest round (the voicing, knobs and 2x2 work) used Claude: Opus 5.5 as
the controller with Sonnet and Opus subagents.
