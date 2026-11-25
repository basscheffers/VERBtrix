# VERBtrix - project notes for Claude

Read this first. It is the handoff between Claude sessions (2026-09-08 build,
2026-09-09 interpolation fix, 2026-09-17 library packaging, 2026-09-21 CPU
optimisation, 2026-11-26 lite quality). README.md has the
user-facing documentation; this file has the context, decisions and open items
that are not obvious from the code. Sessions may run under different accounts,
so nothing lives in Claude's memory: everything worth knowing is in here.

## What this is

A high quality stereo reverb for the Teensy Audio Library, built because the
user found the stock Teensy reverbs (effect_reverb, effect_freeverb) metallic
and wanted something on the level of Valhalla VintageVerb. The user's words
after flashing it: "It sounds amazing."

Hardware: Teensy 4.1 + PJRC Audio Shield (SGTL5000), line in -> reverb -> line
out / headphones. Uploaded with the Arduino IDE. Teensy core 1.62 is installed.
`arduino-cli` is NOT on PATH (as of 2026-11-26); it lives inside the IDE bundle:

```bash
CLI="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"
"$CLI" compile --fqbn teensy:avr:teensy41:usb=serial,speed=600,opt=o2std,keys=en-us --warnings more --library . examples/VERBtrix
"$CLI" compile --fqbn teensy:avr:teensy41:usb=serial,speed=600,opt=o2std,keys=en-us --warnings more --library . examples/Minimal
```

The project is laid out as an Arduino 1.5 library (since 2026-09-17) so it can be installed
into other Teensy projects: `library.properties`, `src/`, `examples/`, `extras/`, `keywords.txt`.
Sketches use `#include <VERBtrix.h>`. Only `src/` is compiled by Arduino.

## Git / publishing state

- Remote: `git@github.com:basscheffers/VERBtrix.git`, branch `main`. Identity in this repo is
  `Bas Scheffers <bas@scheffers.net>` (also the author/maintainer in `library.properties`).
- `main` is pushed and in sync with the remote as of 2026-09-21 (the library commit and the
  CLAUDE.md handoff are on GitHub).
- Rendered WAVs (`extras/test/out/`), harness binaries, `.DS_Store` and `.claude/settings.local.json`
  are git-ignored. The harness regenerates the WAVs in seconds.
- Not done yet: a `v1.0.0` tag / GitHub release (Arduino Library Manager and the PlatformIO
  registry key on tags), and the Library Manager submission itself (a PR adding the repo URL to
  github.com/arduino/library-registry). The library name `VERBtrix` looks unique but was not
  checked against the registry.

## Files

- `src/VERBtrix.h` - umbrella include for sketches.
- `src/VintageReverbCore.h` - the float DSP core, no Arduino dependencies, header only.
- `src/AudioEffectVintageReverb.h/.cpp` - the Teensy `AudioStream` object (2 in / 2 out),
  parameter setters, `Type` and `Preset` enums, type-defaults and 32-preset tables (in the .cpp).
  Has a `#error` guard for anything but `__IMXRT1062__` (Teensy 4.x).
- `examples/VERBtrix/VERBtrix.ino` - demo sketch with serial control. Starts on Concert Hall, mix 0.35.
- `examples/Minimal/Minimal.ino` - shortest possible use.
- `extras/test/test_reverb.cpp` - offline measurement harness + WAV renderer (writes `extras/test/out/`).
  Runs the whole suite once per quality (`gQuality`); lite WAVs are written as `out/lite_*.wav`.
- `extras/test/param_sweep.cpp` - click test while changing parameters, both qualities.
- `extras/test/ir_dump.cpp` - early impulse response statistics per 10 ms.
- `library.properties`, `keywords.txt` - Arduino library metadata and IDE syntax colouring.
- `README.md` - documentation, algorithm description, parameter table, preset list, measurements.

Build the harness on the Mac with:
```bash
cd extras/test && clang++ -O2 -std=c++17 -o test_reverb test_reverb.cpp && ./test_reverb
```
The binaries and `out/*.wav` are git-ignored.

## Algorithm (VintageReverbCore.h)

Modulated N-line feedback delay network (Griesinger/Dattorro/Costello lineage):
low cut -> 2-pole high cut -> pre-delay -> 4 series input allpasses per channel
-> N delay lines (primes, scaled by `size`) each with an interpolated modulated
read, bass shelf + high shelf (per-band RT60, Jot), decay gain, in-loop allpass,
then an NxN Hadamard matrix. Outputs are orthogonal weighted sums of the N lines
plus N early taps.

Two qualities (`vrv::Quality`, picked in `init()` / the AudioEffect constructor
because the memory layout depends on it):

| | QUALITY_HIGH | QUALITY_LITE |
| --- | --- | --- |
| lines / matrix | 8, 8x8 | 4, 4x4 |
| tank lengths | 1237..2617 | 1511, 1987, 2503, 3079 |
| in-loop allpasses | 331..1103 | 331, 541, 877, 1103 |
| tank read | `interp6` | `interp4` |
| loop length | 20712 | 11932 |
| `kDecayTrim` / `kOutScale` | 1.045 / 1.5 | 1.062 / 1.61 |
| ARM instr/sample (-O2) | 1570 | 749 |

How it is built: one class, with the per sample loop in a private
`processN<int N>` template that `process()` dispatches to on `quality_`. Every
table, the matrix size and the interpolation order are compile time constants
inside it (`if constexpr`), so neither quality pays for the other at run time;
the state arrays are sized `kMaxLines` and only the first N are used. Both
instantiations are always compiled in (about 6 KB of extra ITCM even if a
sketch uses only one) - that is the price of choosing at construction instead
of templating the user facing class, and it keeps `AudioEffectVintageReverb
reverb;` working unchanged.

QUALITY_LITE was added 2026-11-26 because the user measured 18 % CPU and wanted
under 10 %; everything else (parameters, types, all 32 presets, the smoothing,
freeze, the input stage) is shared and behaves identically.

Decisions that matter:
- Float everywhere. Delay memory is ~268 KB from the heap (RAM2), or user supplied (EXTMEM ok).
- Shelving filters are `x + (G-1)*LP(x)` style first-order shelves, chosen because they are
  provably bounded by max(1, G): the loop cannot go unstable at any setting. `highMult` is
  capped at 1 for that reason; `bassMult` may exceed 1.
- Size, pre-delay and mix ramp per sample; other parameters smooth per block (40 ms, size 250 ms).
  Changing size glides the delays (pitch swoop) on purpose, like the vintage units.
- `kDecayTrim = 1.045` compensates allpass group delay so RT60 measures within 1 % at size 1.0.
- `kOutScale = 1.5` calibrates 100 % wet to about -0.5 dB vs dry for noise at 2 s decay, size 1.
- Every fractional delay in the loop (tank, in-loop allpass, pre-delay) uses the same 6-point
  Lagrange interpolator (`interp6`). Reason (found 2026-09-09): a fractional read is a low pass,
  and in the loop that loss shortens the treble decay. The in-loop allpasses used linear
  interpolation, which at a half sample offset loses 1.5 dB per pass at 8 kHz; at size 0.5 and
  1.5 every delay sits on a half sample, which is why RT60 read 12-15 % short there while size
  1.0 (all integer delays) was exact.
  "0 ms" pre-delay is really 2 samples (`kPreDelayOffset`) because the interpolator needs lead.
- (2026-09-21) The in-loop allpass and pre-delay *targets* are rounded to whole samples
  (`loopApLength` uses `rintf`, `preTarget` too). They are interpolated (`processFrac` /
  `read6`) only while a size or pre-delay glide is in progress and are plain integer reads
  otherwise. A half sample in a 331..1103 sample allpass is inaudible, and integer reads have
  no interpolation loss at all, so treble RT60 at size != 1 got *longer* (8 kHz at size 0.2:
  1.67 -> 1.78 s). Only the 8 LFO modulated tank reads pay `interp6` per sample.
- (2026-09-21) Early tap positions are fixed per block, except when a line moves more than a
  sample within the block (size glide): then they step per sample as before. Fixing them per
  block during a glide made the taps jump ~25 samples at block boundaries (param_sweep slew
  ratio 2.7 -> 5.2 at the glide start), which is why the `tapPerSample` path exists.
- (2026-11-26) `memoryRequired()` / `memoryFloats()` are `constexpr` (via `ceilU`, because
  `ceilf` is not). The README and the header had always documented
  `EXTMEM float mem[AudioEffectVintageReverb::memoryFloats()];` for PSRAM, and that had never
  compiled ("size of array is not an integral constant-expression"); it does now, verified.
- (2026-09-21) `VRV_UNROLL` (`#pragma GCC unroll 8`) on the per-line loops in `process()` and
  `VRV_INLINE` (`always_inline`) on the delay line helpers. Found by disassembling the Teensy
  compiler's output: at -O2 GCC left `DelayLine::read6` as a real function call, 8 per sample,
  and kept `d[]`/`f[]`/`len[]` on the stack. Without `always_inline` GCC still emits calls to
  one shared `read6` whose `bx lr` is an indirect-branch mispredict per call on the M7.
- Presets do not touch mix/wetGain (they belong to the patch).

## Verified (offline, extras/test/test_reverb.cpp)

RT60 within 1 % at size 1.0 (0.5..8 s). At size 0.2/0.5/1.5 broadband reads 1.90-1.92 s for a
2 s target (was 1.69-1.76 before the 6-point interpolator); below 4 kHz within 3 %, 8 kHz 1.7-1.9 s,
12 kHz 1.5-1.6 s. That remaining treble shortfall is the interpolation loss that is left; it is
small next to any real highMult setting and the harness prints the table. Band decay follows
bass/high multipliers. Stable for 60 s at decay 120 s, bassMult 4, size 1.5, full modulation.
Freeze drifts about -1 dB over 30 s (was -2). Echo density reaches Gaussian by ~50 ms (room) /
~80 ms (hall). L/R tail correlation -0.02. No clicks on any parameter change (param_sweep).
Compiles clean for Teensy 4.1 (2026-09-09).

QUALITY_LITE (2026-11-26): RT60 within 1 % at size 1.0 (0.5..8 s) after calibrating
`kDecayTrimLite`; wet level -0.6 dB; band decay tracks the multipliers; stable in the 120 s /
bassMult 4 / size 1.5 worst case; no NaN anywhere. Weaker than QUALITY_HIGH in exactly two
measurable ways: echo density at size 1.0 reaches Gaussian at ~200 ms instead of ~100 ms
(0.58 @60 ms, 0.66 @80 ms against 0.75 / 0.91) - small sizes are nearly as good (0.95 @60 ms) -
and the treble tail is shorter (12 kHz 1.53 s vs 1.68 s, mod 0.4, size 1.0, 2 s target), which
is the 4-point read. Freeze drifts -1.7 dB over 30 s (high: -1.0). param_sweep is clean except
for a 5.31 slew ratio in the window where freeze engages (high: 2.85 there, worst 3.71
elsewhere; the harness calls >> 6 a click, Gaussian noise is 4-5), so there may be a faint tick
on freeze-on at QUALITY_LITE. Deliberately left alone: smoothing the freeze transition would
change QUALITY_HIGH behaviour too, which was not asked for.

CPU history: the user's "13 %" (quoted on 2026-09-21) was from a build flashed BEFORE the
6-point interpolator, i.e. the 2026-09-08 code. The first measurement of the 6-point code,
with the 2026-09-21 tier-1 work already in it, was **18 %** (2026-11-26). That is consistent:
the interpolator raised host cost 49 -> 80 ns/sample. So nothing regressed; 6-point
interpolation is simply what it costs. The emulator counts the tier-1 work saved (2069 ->
1556 instr/sample) are still believed, they were just measured against the wrong baseline
expectation. QUALITY_LITE is 749 vs 1570 instr/sample, so expect ~9 % on the board.
NOT yet measured on the board and NOT listened to. -O3 ("Fastest") gains nothing (1526 vs
1556 at QUALITY_HIGH). Preset values are acoustic judgment, expected to need tuning by ear.

## Measuring CPU without a board

The host harness's ns/sample is a poor proxy: the Mac's out-of-order core hides work behind the
filter recurrences (the 2026-09-21 change moved it 74.6 -> 57.5 ns while removing 25 % of the
M7 instructions). What worked instead: compile a bare-metal benchmark with the Teensy toolchain
(`~/Library/Arduino15/packages/teensy/tools/teensy-compile/15.2.1/arm/bin/arm-none-eabi-g++`,
same `-mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -std=gnu++17` flags as `boards.txt`,
`-nostartfiles --specs=nosys.specs -Wl,-Ttext=0x08000000 -lm`), load the flat binary into the
Unicorn emulator (`pip install unicorn`, `UC_ARCH_ARM`, `UC_MODE_THUMB`, CPU model
`UC_CPU_ARM_MAX` because the Cortex-A15 model lacks `vrintp`; enable the FPU via
`UC_ARM_REG_CP_REG` CPACR write and `UC_ARM_REG_FPEXC = 0x40000000`), call `process()` on a
static core and count executed instructions with a block hook against the objdump address
list. Cortex-M mode in Unicorn could not enable the FPU; A-profile Thumb runs the same code.
The scripts lived in the session scratchpad and are not in the repo; ~60 lines of Python.
Instruction count is not cycles (dual issue, FPU latency, cache) but tracks the in-order M7
far better than host timing. Also useful: `arm-none-eabi-objdump -dr` on the object to see
which helpers are actually inlined (`R_ARM_THM_CALL` relocations inside `process`).

Host ns/sample from the harness is still printed and still misleading: QUALITY_LITE measures
27 ns against 55 ns (0.49x) where the M7 count says 0.48x - that one happens to agree, but the
tier-1 change had them disagreeing badly (74.6 -> 57.5 ns for a 25 % instruction cut).

## Gotchas learned

- `HIGH` is an Arduino macro (`core_pins.h`), so the quality constants cannot be called
  `HIGH`/`LITE`. They are `AudioEffectVintageReverb::QUALITY_HIGH` / `QUALITY_LITE` (aliases of
  `vrv::QUALITY_*`). The error this produces is a confusing
  "expected unqualified-id before numeric constant" pointing INTO core_pins.h.
- Use `if constexpr`, not plain `if`, to pick the 8-line Hadamard inside `processN<N>`: with a
  plain `if` the N=4 instantiation still compiles the branch that indexes `f[4]..f[7]` on
  4-element arrays. The editor's clang warns "constexpr if is a C++17 extension" because it
  does not pick up the project standard; gnu++17 (Teensy) and -std=c++17 (harness) are fine.

- The first upload looked dead: the Arduino IDE serial monitor sends no line ending by default
  and the startup prints came before the monitor attached. The sketch now waits up to 4 s for
  `Serial`, prints `CrashReport`, and executes commands on newline OR 100 ms after the last char.
- `__SSAT` is not available without CMSIS in a sketch; a plain clamp is used instead.
- `reverb.reset()` used to hard fault when the heap allocation had failed (`resetState()`
  memsets `DelayLine::buf`, which is still null then). Guarded with `if (!core_.ready())`
  on 2026-11-26.
- The editor's clang shows "Arduino.h not found" errors in the Teensy files; ignore, arduino-cli
  is the real check.
- Arduino only compiles `src/` of a library, so `extras/test/` is safe from the build.

## Where we left off / possible next steps

Last completed (2026-11-26): QUALITY_LITE, the 4-line/4x4/4-point variant selected at
construction (see Algorithm). Also: `memoryFloats()` made constexpr so the documented EXTMEM
pattern compiles, the harness and param_sweep run both qualities, `VERBTRIX_LITE` switch at
the top of `examples/VERBtrix/VERBtrix.ino` (prints the quality and `processorUsage()` so the
two can be compared on the board), README "Quality" section, keywords.txt.
Verified: the QUALITY_HIGH path is bit-identical to commit ef2ae5d (every harness number
matches), both examples compile in both configurations with `--warnings more` and no warnings,
and the EXTMEM/PSRAM pattern compiles.
NOT flashed and NOT listened to. On hardware:
1. `reverb.processorUsage()` at QUALITY_LITE (expect ~9 %, high reads 18 %).
2. Whether QUALITY_LITE is acceptable by ear, in particular a large hall or cathedral, where
   the density deficit is concentrated (compare `extras/test/out/hall_wet.wav` against
   `out/lite_hall_wet.wav`, and the same for cathedral), and whether freeze-on ticks.

Experiments that did NOT work, so nobody repeats them (2026-11-26):
- Raising the in-loop allpass coefficient for lite (0.55 -> 0.65/0.75/0.85 x diffusion) does
  nothing for echo density: 60 ms stayed at 0.57-0.58. The deficit is the number of echo
  generators, not the strength of the diffusion.
- A SECOND in-loop allpass per lite line (463, 727, 1039, 1279) also did nothing for density
  (0.58 / 0.62 / 0.80 at 60/80/100 ms, i.e. unchanged), while costing CPU, 28 KB and a RT60
  recalibration. Reverted. It did improve L/R correlation (-0.048 -> -0.018), if that ever
  matters.

Before that (2026-09-09, first half): renamed the sketch "presets" to `Type` (8 families) and
added a `Preset` enum with 32 named presets, `settings()`/`apply()` for user presets.

Ideas the user has not decided on yet:
- Tune preset values by ear (serial: `N` lists, `n 12` loads, `t 3` loads a type).
- An early-reflection stage, or a VintageVerb style "color" mode (1970s band-limited/noisy).
- Encoder/display UI for scrolling presets (the enum + `presetName()` are ready for it).
- Reduce memory (int16 pre-delay, smaller max pre-delay) if RAM2 gets tight in a bigger project.
- If CPU is still tight at QUALITY_LITE: a guard region on the delay buffers (one masked base
  index + 5 contiguous loads per interpolated read instead of 6 masked ones) is free in
  quality; dropping the input diffusers from 4 to 3 per channel saves ~16 instr/sample at a
  real density cost. Modulating only half the lines is the other lever (the rest become
  integer reads) but needs a listening test.
- If QUALITY_LITE turns out too thin by ear, the next thing to try is 6 lines rather than
  tuning the 4-line tables: `processN<6>` with a 6x6 matrix is not a Hadamard (use a
  Householder or a block-circulant orthogonal matrix instead), which is why 4 was chosen.
- Preset values were never retuned for QUALITY_LITE. They are shared on purpose, but the
  diffusion and modulation defaults may want to be a little higher there to cover the density
  deficit.
