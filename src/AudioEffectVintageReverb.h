/* AudioEffectVintageReverb
 *
 * High quality stereo reverb for the Teensy Audio Library (Teensy 4.x).
 *
 * A modulated 8-line feedback delay network with input diffusion, in-loop
 * allpass diffusers, frequency dependent decay (bass multiplier, high
 * damping), pre-delay, size, freeze and dry/wet mix. All DSP runs in floating
 * point on the Cortex-M7 FPU. See VintageReverbCore.h for the algorithm.
 *
 * Inputs : 0 = left, 1 = right   (feed the same source to both for mono)
 * Outputs: 0 = left, 1 = right
 *
 * Memory: about 268 KB of delay memory (float). By default it is taken from
 * the heap (RAM2 / OCRAM on Teensy 4.x). To keep RAM2 free, allocate the
 * buffer yourself, for instance in PSRAM on a Teensy 4.1:
 *
 *     EXTMEM float reverbMem[AudioEffectVintageReverb::memoryFloats()];
 *     AudioEffectVintageReverb reverb(reverbMem, AudioEffectVintageReverb::memoryFloats());
 *
 * Usage:
 *     AudioEffectVintageReverb reverb;
 *     AudioConnection c1(source, 0, reverb, 0);
 *     AudioConnection c2(source, 1, reverb, 1);
 *     AudioConnection c3(reverb, 0, out, 0);
 *     AudioConnection c4(reverb, 1, out, 1);
 *     ...
 *     reverb.decay(3.0f);      // seconds
 *     reverb.size(1.0f);       // 0.2 .. 1.5
 *     reverb.mix(0.3f);        // 0 dry .. 1 wet
 *
 * MIT license.
 */

#ifndef effect_vintage_reverb_h_
#define effect_vintage_reverb_h_

#include <Arduino.h>
#include <AudioStream.h>
#include "VintageReverbCore.h"

#if !defined(__IMXRT1062__)
#error "VERBtrix needs a Teensy 4.x (i.MX RT1062): 268 KB of delay memory and the Cortex-M7 FPU."
#endif

class AudioEffectVintageReverb : public AudioStream
{
public:
  static constexpr float kMaxPreDelayMs = 250.0f;

  // Reverb type: the family of space the algorithm is set up to imitate.
  // Selecting a type loads its characteristic diffusion, modulation, damping
  // and bandwidth together with a typical size and decay.
  enum Type : uint8_t {
    AMBIENCE,       // very short, dense, adds air without an audible tail
    ROOM,           // small to medium rooms
    CHAMBER,        // classic echo chamber: medium size, dense, slightly dark
    PLATE,          // EMT style plate: fast build up, bright, thin bass
    HALL,           // concert hall
    CATHEDRAL,      // very large stone space, long dark tail
    ARENA,          // stadium / arena: long pre-delay, sparse, distant
    CHORUS_SPACE,   // heavily modulated 80s digital hall
    TYPE_COUNT
  };

  // Presets: named starting points, each built on one of the types.
  enum Preset : uint8_t {
    TIGHT_AMBIENCE, VOCAL_AMBIENCE,
    SMALL_ROOM, TILED_BATHROOM, WOOD_ROOM, DRUM_ROOM, LARGE_ROOM,
    SMALL_CHAMBER, VOCAL_CHAMBER, STONE_CHAMBER,
    WARM_PLATE, BRIGHT_PLATE, VINTAGE_PLATE, VOCAL_PLATE, DRUM_PLATE, LONG_PLATE,
    SMALL_HALL, CONCERT_HALL, BRIGHT_HALL, WARM_HALL, VOCAL_HALL, LARGE_HALL, DARK_HALL,
    ARENA_PRESET, STADIUM,
    CHURCH, LARGE_CATHEDRAL, SANCTUARY,
    CHORUS_SPACE_PRESET, EIGHTIES_HALL, AMBIENT_WASH, INFINITE_SPACE,
    PRESET_COUNT
  };

  // A complete set of sound-defining parameters (mix and wet gain are left
  // alone, those belong to the patch, not the space).
  struct Settings {
    Type  type;
    float size, decay, predelayMs, diffusion, modRate, modDepth;
    float bassMult, bassXover, highMult, highXover, lowCut, highCut;
  };

  // Allocates delay memory from the heap.
  AudioEffectVintageReverb();
  // Uses caller supplied memory (at least memoryFloats() floats).
  AudioEffectVintageReverb(float *memory, size_t memoryFloats);
  virtual ~AudioEffectVintageReverb();

  virtual void update(void);

  // Number of floats of delay memory the effect needs.
  static size_t memoryFloats() {
    return vrv::VintageReverbCore::memoryRequired(AUDIO_SAMPLE_RATE_EXACT, kMaxPreDelayMs);
  }
  // False if memory allocation failed; the effect then passes audio through.
  bool ready() const { return core_.ready(); }

  // ------------------------------------------------------- types & presets
  // Load a type: its characteristic parameters plus a typical size and decay.
  void type(Type t);
  Type type() const { return type_; }
  // Load a preset. Also sets the type it is built on.
  void preset(Preset p);
  // Last preset loaded (PRESET_COUNT if none was loaded yet).
  Preset preset() const { return preset_; }
  // Apply a complete Settings block (for user defined presets).
  void apply(const Settings &s);
  // Current parameters as a Settings block (to save your own presets).
  Settings settings() const;

  static const char     *typeName(Type t);
  static const Settings &typeDefaults(Type t);
  static const char     *presetName(Preset p);
  static const Settings &presetSettings(Preset p);
  static Type            presetType(Preset p) { return presetSettings(p).type; }

  // ---------------------------------------------------------- parameters
  // Dry/wet balance, 0.0 = dry, 1.0 = wet only (default 1.0).
  void mix(float m)            { core_.setMix(m); }
  // Pre-delay before the reverb starts, 0 .. 250 ms (default 0).
  void predelay(float ms)      { core_.setPreDelayMs(ms); }
  // Mid band decay time RT60 in seconds, 0.1 .. 120 (default 2.5).
  void decay(float seconds)    { core_.setDecaySeconds(seconds); }
  // Room size, 0.2 (small room) .. 1.5 (huge hall) (default 1.0).
  // Changing size while running glides the delays (a pitch swoop), like the
  // vintage units did.
  void size(float s)           { core_.setSize(s); }
  // Diffusion 0 .. 1, how quickly the echoes smear into a smooth wash (default 0.85).
  void diffusion(float d)      { core_.setDiffusion(d); }
  // Delay modulation: rate in Hz 0 .. 5 (default 1.0), depth 0 .. 1 (default 0.4).
  // Modulation is what removes the metallic ring; depth > 0.6 gives an audible chorus.
  void modRate(float hz)       { core_.setModRate(hz); }
  void modDepth(float d)       { core_.setModDepth(d); }
  // Bass decay multiplier 0.25 .. 4 below the bass crossover (default 1.0 @ 300 Hz).
  void bassMult(float m)       { core_.setBassMult(m); }
  void bassXover(float hz)     { core_.setBassXover(hz); }
  // High frequency decay multiplier 0.1 .. 1 above the high crossover (default 0.5 @ 4 kHz).
  void highMult(float m)       { core_.setHighMult(m); }
  void highXover(float hz)     { core_.setHighXover(hz); }
  // Input band limiting: low cut 10 .. 1000 Hz (default 20), high cut 1 .. 20 kHz (default 16k).
  // A high cut around 6-9 kHz gives the darker 1970s digital character.
  void lowCut(float hz)        { core_.setLowCut(hz); }
  void highCut(float hz)       { core_.setHighCut(hz); }
  // Freeze the tail (infinite decay, input muted).
  void freeze(bool on)         { core_.setFreeze(on); }
  // Wet output level, linear 0 .. 4 (default 1.0).
  void wetGain(float g)        { core_.setWetGain(g); }
  // Hard bypass (dry passthrough, delay memory keeps running so no click on return).
  void bypass(bool on)         { bypass_ = on; }
  // Clear the tail immediately.
  void reset()                 { __disable_irq(); core_.resetState(); __enable_irq(); }

  float mix() const       { return core_.mix(); }
  float predelay() const  { return core_.preDelayMs(); }
  float decay() const     { return core_.decay(); }
  float size() const      { return core_.size(); }
  float diffusion() const { return core_.diffusion(); }
  float modRate() const   { return core_.modRate(); }
  float modDepth() const  { return core_.modDepth(); }
  float bassMult() const  { return core_.bassMult(); }
  float bassXover() const { return core_.bassXover(); }
  float highMult() const  { return core_.highMult(); }
  float highXover() const { return core_.highXover(); }
  float lowCut() const    { return core_.lowCut(); }
  float highCut() const   { return core_.highCut(); }
  bool  freeze() const    { return core_.freeze(); }
  bool  bypass() const    { return bypass_; }

private:
  void attach(float *memory, size_t memoryFloats);

  audio_block_t *inputQueueArray_[2];
  vrv::VintageReverbCore core_;
  float *ownedMemory_ = nullptr;
  bool   bypass_ = false;
  Type   type_   = HALL;
  Preset preset_ = PRESET_COUNT;
};

#endif
