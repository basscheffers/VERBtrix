# VERBtrix - project notes for Claude

Read this first. It is the handoff from the session that built the project
(2026-09-08/09). README.md has the user-facing documentation; this file has
the context, decisions and open items that are not obvious from the code.

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
Git remote: github.com/basscheffers/VERBtrix. Rendered WAVs and harness binaries are ignored.

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
  1.0 (all integer delays) was exact. Allpass and pre-delay lengths only move during a glide,
  so their weights are computed once per block (`lagrangeWeights6`) and the per sample
  polynomial is paid only by the 8 LFO modulated tank reads. Host cost went 49 -> ~80 ns/sample.
  "0 ms" pre-delay is really 2 samples (`kPreDelayOffset`) because the interpolator needs lead.
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

NOT verified: actual CPU load on the Teensy (estimate 10-15 % after the interpolator change,
no board was connected; the sketch prints `reverb.processorUsage()` every 2 s), and nothing has
been evaluated by ear except the user's overall verdict. Preset values are acoustic judgment,
expected to need tuning by ear.

## Gotchas learned

- The first upload looked dead: the Arduino IDE serial monitor sends no line ending by default
  and the startup prints came before the monitor attached. The sketch now waits up to 4 s for
  `Serial`, prints `CrashReport`, and executes commands on newline OR 100 ms after the last char.
- `__SSAT` is not available without CMSIS in a sketch; a plain clamp is used instead.
- The editor's clang shows "Arduino.h not found" errors in the Teensy files; ignore, arduino-cli
  is the real check.
- Arduino only compiles `src/` of a library, so `extras/test/` is safe from the build.

## Where we left off / possible next steps

Last completed (2026-09-09, second session): traced the "RT60 short at size != 1" item to
interpolation loss and replaced linear/cubic reads in the loop with 6-point Lagrange (see
Decisions). Harness, README and this file updated. Everything builds. The new build has not been
flashed or listened to yet; first thing to check on hardware is `reverb.processorUsage()`.
Before that: renamed the sketch "presets" to `Type` (8 families) and added a `Preset` enum with
32 named presets, `settings()`/`apply()` for user presets.

Ideas the user has not decided on yet:
- Tune preset values by ear (serial: `N` lists, `n 12` loads, `t 3` loads a type).
- An early-reflection stage, or a VintageVerb style "color" mode (1970s band-limited/noisy).
- Encoder/display UI for scrolling presets (the enum + `presetName()` are ready for it).
- Reduce memory (int16 pre-delay, smaller max pre-delay) if RAM2 gets tight in a bigger project.
- If CPU turns out tight: the 8 tank reads could go back to 4-point (cubic/Lagrange) at the cost
  of ~10 % shorter decay at 12 kHz with modulation on; the allpasses should stay 6-point.
