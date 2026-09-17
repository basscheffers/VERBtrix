# VERBtrix

A high quality stereo reverb for the Teensy 4.x Audio Library, written as a
drop-in `AudioStream` object with stereo in/out and full parameter control.

## Installation

VERBtrix is a standard Arduino library.

- **Arduino IDE:** download the repository as a zip and use
  *Sketch > Include Library > Add .ZIP Library*, or clone it into your
  sketchbook's `libraries/` folder (`~/Documents/Arduino/libraries/VERBtrix`).
- **arduino-cli:** `arduino-cli lib install --git-url https://github.com/basscheffers/VERBtrix.git`
- **PlatformIO:** add `https://github.com/basscheffers/VERBtrix.git` to `lib_deps`.

Then `#include <VERBtrix.h>`. The library needs Teensyduino (the Audio
library and `AudioStream` come from the Teensy core) and a Teensy 4.0 or 4.1;
it refuses to compile for anything else because it needs 268 KB of delay
memory and the Cortex-M7 FPU.

## Files

| File | What it is |
| --- | --- |
| `src/VERBtrix.h` | The header to include |
| `src/AudioEffectVintageReverb.h/.cpp` | The Teensy Audio Library object (`AudioStream` subclass, 2 in / 2 out) |
| `src/VintageReverbCore.h` | The float DSP core. No Arduino dependencies, so it can be tested on a PC |
| `examples/Minimal` | Audio Shield line-in -> reverb -> out, one preset, five lines of setup |
| `examples/VERBtrix` | Demo with serial control of every parameter, type and preset |
| `extras/test/test_reverb.cpp` | Offline measurement harness (RT60, band decay, stability, echo density, stereo, level) and WAV renderer |
| `extras/test/param_sweep.cpp` | Changes every parameter while audio runs and checks for clicks |
| `extras/test/ir_dump.cpp` | Prints early impulse response statistics per 10 ms |
| `extras/test/out/*.wav` | Rendered listening examples (room, plate, hall, cathedral; wet, 35 % mix, impulse response). Not in git, the harness writes them |

## Algorithm

The design is a modulated feedback delay network (FDN), the topology family
behind the classic late 70s / 80s digital hall algorithms and their modern
software descendants:

```
in L/R -> low cut -> high cut (2-pole) -> pre-delay -> 4 series allpass diffusers
       -> 8 line tank, per line:
            modulated delay (6-point Lagrange interpolation)
            -> bass shelf (bass multiplier)
            -> high shelf (high frequency damping)
            -> decay gain from RT60
            -> in-loop allpass diffuser
       -> 8x8 Hadamard mixing matrix -> back into the delays
out L/R <- decorrelated weighted sums of the 8 line outputs + 8 early taps
```

Why it does not sound like the stock Teensy reverb:

- **Orthogonal feedback matrix.** All eight delays are coupled, so the modal
  density is that of the whole loop (about 20 000 samples), not of one comb
  filter at a time. That removes the metallic ringing of Schroeder/Freeverb
  networks.
- **Delay modulation.** Each line is modulated by its own slow sine at an
  incommensurate rate. Modes never settle into a stationary ring. Depth 0.3
  to 0.5 is subtle; higher depths give the chorused 80s hall sound.
- **6-point Lagrange interpolation** for every fractional delay inside the
  loop (tank, in-loop allpasses, pre-delay). A fractional read is a low pass,
  and inside a feedback loop that loss shortens the treble decay: at a half
  sample offset linear interpolation loses 1.5 dB per pass at 8 kHz, cubic
  Hermite 0.3 dB, 6-point Lagrange 0.08 dB. Delays that are not moving use
  weights computed once per block.
- **Frequency dependent decay** computed per line from the requested RT60
  (Jot's method). Bass and treble get their own RT60 multipliers through
  first-order shelving filters that are provably bounded below unity gain, so
  the loop is stable at any setting, including 120 s decays with a 4x bass
  multiplier.
- **In-loop allpass diffusers** keep multiplying echo density on every pass.
  The normalized echo density reaches 1.0 (Gaussian, fully mixed) by about
  50 ms at room size and 80 ms at hall size.
- **Float processing** throughout. No fixed-point noise floor building up in
  the feedback loop.

## Using it

```cpp
#include <VERBtrix.h>

AudioInputI2S            in;
AudioEffectVintageReverb reverb;
AudioOutputI2S           out;
AudioConnection c1(in, 0, reverb, 0);
AudioConnection c2(in, 1, reverb, 1);
AudioConnection c3(reverb, 0, out, 0);
AudioConnection c4(reverb, 1, out, 1);

void setup() {
  AudioMemory(20);
  reverb.decay(3.0f);      // RT60 in seconds
  reverb.size(1.0f);       // 0.2 .. 1.5
  reverb.predelay(20.0f);  // ms
  reverb.mix(0.3f);        // 0 dry .. 1 wet (default 1.0 = wet only)
}
```

For mono sources connect the same output to both inputs.

### Types and presets

A **type** is the family of space the algorithm imitates. Selecting one loads
its characteristic diffusion, modulation, damping and bandwidth together with
a typical size and decay:

| Type | Character |
| --- | --- |
| `AMBIENCE` | Very short and dense, adds air without an audible tail |
| `ROOM` | Small to medium rooms |
| `CHAMBER` | Classic echo chamber: medium size, dense, slightly dark |
| `PLATE` | EMT style plate: instant build-up, bright, thin bass, strong modulation |
| `HALL` | Concert hall |
| `CATHEDRAL` | Very large stone space, long dark tail |
| `ARENA` | Stadium or arena: long pre-delay, less diffuse, distant |
| `CHORUS_SPACE` | Heavily modulated 80s digital hall |

A **preset** is a named, complete starting point built on a type. Mix and wet
gain are not part of a preset, since they belong to the patch rather than the
space.

```cpp
reverb.type(AudioEffectVintageReverb::PLATE);            // type defaults
reverb.preset(AudioEffectVintageReverb::WARM_PLATE);     // named preset
Serial.println(AudioEffectVintageReverb::presetName(reverb.preset()));

// iterate all presets
for (int i = 0; i < AudioEffectVintageReverb::PRESET_COUNT; i++) {
  auto p = (AudioEffectVintageReverb::Preset)i;
  Serial.println(AudioEffectVintageReverb::presetName(p));
}

// save and restore your own settings
AudioEffectVintageReverb::Settings mine = reverb.settings();
reverb.apply(mine);
```

| # | Preset | Type | Size | Decay |
| --- | --- | --- | --- | --- |
| 0 | Tight Ambience | Ambience | 0.25 | 0.3 s |
| 1 | Vocal Ambience | Ambience | 0.35 | 0.55 s |
| 2 | Small Room | Room | 0.30 | 0.7 s |
| 3 | Tiled Bathroom | Room | 0.28 | 1.1 s |
| 4 | Wood Room | Room | 0.40 | 0.8 s |
| 5 | Drum Room | Room | 0.45 | 0.6 s |
| 6 | Large Room | Room | 0.60 | 1.2 s |
| 7 | Small Chamber | Chamber | 0.50 | 1.3 s |
| 8 | Vocal Chamber | Chamber | 0.60 | 1.8 s |
| 9 | Stone Chamber | Chamber | 0.70 | 2.4 s |
| 10 | Warm Plate | Plate | 0.55 | 2.0 s |
| 11 | Bright Plate | Plate | 0.50 | 2.2 s |
| 12 | Vintage Plate | Plate | 0.60 | 2.8 s |
| 13 | Vocal Plate | Plate | 0.55 | 1.7 s |
| 14 | Drum Plate | Plate | 0.45 | 1.3 s |
| 15 | Long Plate | Plate | 0.70 | 5.0 s |
| 16 | Small Hall | Hall | 0.80 | 1.8 s |
| 17 | Concert Hall | Hall | 1.00 | 2.6 s |
| 18 | Bright Hall | Hall | 1.00 | 2.8 s |
| 19 | Warm Hall | Hall | 1.10 | 3.2 s |
| 20 | Vocal Hall | Hall | 0.90 | 2.4 s |
| 21 | Large Hall | Hall | 1.30 | 4.5 s |
| 22 | Dark Hall | Hall | 1.20 | 3.8 s |
| 23 | Arena | Arena | 1.40 | 3.5 s |
| 24 | Stadium | Arena | 1.50 | 5.0 s |
| 25 | Church | Cathedral | 1.20 | 4.5 s |
| 26 | Large Cathedral | Cathedral | 1.50 | 9.0 s |
| 27 | Sanctuary | Cathedral | 1.40 | 14 s |
| 28 | Chorus Space | Chorus Space | 1.10 | 4.0 s |
| 29 | 80s Hall | Chorus Space | 1.00 | 3.5 s |
| 30 | Ambient Wash | Chorus Space | 1.50 | 25 s |
| 31 | Infinite Space | Cathedral | 1.50 | 60 s |

The full parameter set of every preset is the table at the top of
`AudioEffectVintageReverb.cpp`, one line per preset, so they are easy to tune
by ear. In the demo sketch `N` lists them and `n 12` loads number 12.

### Parameters

| Method | Range | Default | Meaning |
| --- | --- | --- | --- |
| `mix(f)` | 0 .. 1 | 1.0 | Dry/wet balance |
| `predelay(ms)` | 0 .. 250 | 0 | Delay before the reverb starts |
| `decay(s)` | 0.1 .. 120 | 2.5 | Mid band RT60 |
| `size(f)` | 0.2 .. 1.5 | 1.0 | Scales all delays. Changes glide (pitch swoop) like vintage units |
| `diffusion(f)` | 0 .. 1 | 0.85 | Allpass coefficients, how fast echoes smear |
| `modRate(hz)` | 0 .. 5 | 1.0 | Delay modulation rate |
| `modDepth(f)` | 0 .. 1 | 0.4 | Delay modulation depth (1.0 = 24 samples) |
| `bassMult(f)` | 0.25 .. 4 | 1.0 | Decay multiplier below the bass crossover |
| `bassXover(hz)` | 50 .. 2000 | 300 | Bass crossover |
| `highMult(f)` | 0.1 .. 1 | 0.5 | Decay multiplier above the high crossover |
| `highXover(hz)` | 1k .. 16k | 4000 | High crossover |
| `lowCut(hz)` | 10 .. 1000 | 20 | Input high-pass |
| `highCut(hz)` | 1k .. 20k | 16000 | Input low-pass (2-pole). 6 to 9 kHz gives the dark 70s character |
| `freeze(bool)` | | off | Infinite hold, input muted |
| `wetGain(f)` | 0 .. 4 | 1.0 | Wet output level |
| `bypass(bool)` | | off | Dry passthrough |
| `reset()` | | | Clears the tail |

All setters are safe to call from `loop()` while audio runs. Parameters are
smoothed inside the effect (40 ms, size 250 ms) so there is no zipper noise.

### Memory

The effect needs about 268 KB of delay memory (float). The default
constructor takes it from the heap, which on Teensy 4.x lives in RAM2. To
keep RAM2 free on a Teensy 4.1 with PSRAM fitted:

```cpp
EXTMEM float reverbMem[AudioEffectVintageReverb::memoryFloats()];
AudioEffectVintageReverb reverb(reverbMem, AudioEffectVintageReverb::memoryFloats());
```

`reverb.ready()` returns false if allocation failed; the effect then passes
audio through unchanged.

### Levels

At 100 % wet the stationary level is calibrated to about 0.7 dB below the dry
level for a 2 s decay at size 1.0. Longer decays accumulate more energy and
come out louder, so leave headroom on hot stationary sources or lower
`wetGain()`. The output saturates cleanly to 16 bit.

## Measured behaviour (extras/test/test_reverb.cpp)

| Test | Result |
| --- | --- |
| RT60 accuracy, size 1.0, 0.5 to 8 s | within 1 % |
| RT60 at size 0.2 / 0.5 / 1.5, target 2 s | 1.91 / 1.90 / 1.92 s broadband; below 4 kHz within 3 %, 8 kHz 1.7 to 1.9 s, 12 kHz 1.5 to 1.6 s (interpolation loss) |
| RT60 at size 1.0 with modulation 0.4, target 2 s | 1.9 s broadband, 1.9 s @8 kHz, 1.7 s @12 kHz |
| Band RT60, bassMult 1.5, highMult 0.4 | 2.7 s @100 Hz, 1.9 s @1 kHz, 1.4 s @4 kHz, 1.1 s @12 kHz |
| Stability: decay 120 s, bassMult 4, size 1.5, full modulation, 60 s | decays monotonically, no NaN |
| Freeze, 30 s | level drifts 1 dB down (interpolation loss), no growth |
| Echo density (1.0 = Gaussian) | room: 0.96 @60 ms, hall: 0.93 @80 ms |
| L/R tail correlation | -0.02 |
| Parameter changes while running | no clicks |

Build and run the harness on a Mac or Linux box:

```bash
cd extras/test && clang++ -O2 -std=c++17 -o test_reverb test_reverb.cpp && ./test_reverb
```

## CPU

The core runs at about 80 ns per stereo sample on an Apple laptop core (the
8 modulated 6-point tank reads are about a third of that). On the 600 MHz
Cortex-M7 expect roughly 10 to 15 % load. The demo prints
`reverb.processorUsage()` every 2 s so you can read the real figure.
