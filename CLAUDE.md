# VERBtrix - project notes for Claude

Read this first. It is the handoff between Claude sessions (2026-09-08 build,
2026-09-09 interpolation fix, 2026-09-17 library packaging, 2026-09-21 CPU
optimisation). README.md has the
user-facing documentation; this file has the context, decisions and open items
that are not obvious from the code. Sessions may run under different accounts,
so nothing lives in Claude's memory: everything worth knowing is in here.

## What this is

A high quality stereo reverb for the Teensy Audio Library, built because the
user found the stock Teensy reverbs (effect_reverb, effect_freeverb) metallic
and wanted something on the level of Valhalla VintageVerb. The user's words
after flashing it: "It sounds amazing."

Hardware: Teensy 4.1 + PJRC Audio Shield (SGTL5000), line in -> reverb -> line
out / headphones. Uploaded with the Arduino IDE. Teensy core 1.62 is installed
and `arduino-cli` works for compile checks:

```bash
arduino-cli compile --fqbn teensy:avr:teensy41:usb=serial,speed=600,opt=o2std,keys=en-us --warnings more --library . examples/VERBtrix
arduino-cli compile --fqbn teensy:avr:teensy41:usb=serial,speed=600,opt=o2std,keys=en-us --warnings more --library . examples/Minimal
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
- `extras/test/param_sweep.cpp` - click test while changing parameters.
- `extras/test/ir_dump.cpp` - early impulse response statistics per 10 ms.
- `library.properties`, `keywords.txt` - Arduino library metadata and IDE syntax colouring.
- `README.md` - documentation, algorithm description, parameter table, preset list, measurements.

Build the harness on the Mac with:
```bash
cd extras/test && clang++ -O2 -std=c++17 -o test_reverb test_reverb.cpp && ./test_reverb
```
The binaries and `out/*.wav` are git-ignored.

## Algorithm (VintageReverbCore.h)

Modulated 8-line feedback delay network (Griesinger/Dattorro/Costello lineage):
low cut -> 2-pole high cut -> pre-delay -> 4 series input allpasses per channel
-> 8 delay lines (nominal 1237..2617 samples, primes, scaled by `size`) each with
a 6-point Lagrange modulated read, bass shelf + high shelf (per-band RT60, Jot),
decay gain, in-loop allpass (331..1103 samples), then an 8x8 Hadamard matrix.
Outputs are orthogonal weighted sums of the 8 lines plus 8 early taps.

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

CPU: the user measured 13 % on the Teensy 4.1 (600 MHz, "Faster" -O2) with the 2026-09-18
code. The 2026-09-21 changes cut the dynamic instruction count per stereo sample from 2069 to
1556 (-25 %) in steady state and 2408 -> 2107 during a size glide, measured by running the
Teensy toolchain's own output in an emulator (see "Measuring CPU without a board" below), so
expect ~10 %. NOT yet measured on the board, and not listened to. -O3 ("Fastest") gains
nothing further now (1526 vs 1556). Nothing has been evaluated by ear except the user's overall
verdict. Preset values are acoustic judgment, expected to need tuning by ear.

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

## Gotchas learned

- The first upload looked dead: the Arduino IDE serial monitor sends no line ending by default
  and the startup prints came before the monitor attached. The sketch now waits up to 4 s for
  `Serial`, prints `CrashReport`, and executes commands on newline OR 100 ms after the last char.
- `__SSAT` is not available without CMSIS in a sketch; a plain clamp is used instead.
- The editor's clang shows "Arduino.h not found" errors in the Teensy files; ignore, arduino-cli
  is the real check.
- Arduino only compiles `src/` of a library, so `extras/test/` is safe from the build.

## Where we left off / possible next steps

Last completed (2026-09-21): the "tier 1" CPU work in `VintageReverbCore.h` (integer allpass
and pre-delay targets, per-block early taps, unroll + always_inline; see Decisions), README
and this file updated. Harness: RT60 at size 1.0 unchanged, treble RT60 at other sizes longer,
param_sweep clean (worst slew ratio 3.7, same as before), both examples compile for Teensy 4.1
with `--warnings more` and no warnings. The user had already flashed and listened to the
2026-09-18 library version ("13 % CPU"). The 2026-09-21 change is NOT flashed or listened to.
On hardware, check:
1. `reverb.processorUsage()` (expect ~10 %, was 13 %).
2. That it still sounds the same. What could differ by ear: nothing intended; the allpass
   lengths are now within 0.5 sample of before and the treble tail at small sizes is slightly
   longer (closer to the target). During a size glide the allpasses step by whole samples
   between blocks instead of ramping fractionally, which the click test does not see.

Before that (2026-09-09, first half): renamed the sketch "presets" to `Type` (8 families) and
added a `Preset` enum with 32 named presets, `settings()`/`apply()` for user presets.

Ideas the user has not decided on yet:
- Tune preset values by ear (serial: `N` lists, `n 12` loads, `t 3` loads a type).
- An early-reflection stage, or a VintageVerb style "color" mode (1970s band-limited/noisy).
- Encoder/display UI for scrolling presets (the enum + `presetName()` are ready for it).
- Reduce memory (int16 pre-delay, smaller max pre-delay) if RAM2 gets tight in a bigger project.
- If CPU turns out tight ("tier 2"): the 8 tank reads could go to 4-point (cubic/Lagrange) at
  the cost of ~10 % shorter decay at 12 kHz with modulation on (roughly halves the largest
  remaining item), or modulate only 4 of the 8 lines (the other 4 become integer reads; needs a
  listening test). A guard region on the delay buffers (one masked base index + 5 contiguous
  loads per 6-point read) is a smaller, free one. The allpasses should stay 6-point during
  glides.
