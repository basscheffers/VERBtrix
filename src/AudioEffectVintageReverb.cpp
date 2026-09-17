#include "AudioEffectVintageReverb.h"
#include <stdlib.h>

AudioEffectVintageReverb::AudioEffectVintageReverb()
  : AudioStream(2, inputQueueArray_)
{
  size_t n = memoryFloats();
  ownedMemory_ = (float *)malloc(n * sizeof(float));
  attach(ownedMemory_, ownedMemory_ ? n : 0);
}

AudioEffectVintageReverb::AudioEffectVintageReverb(float *memory, size_t memoryFloats)
  : AudioStream(2, inputQueueArray_)
{
  attach(memory, memoryFloats);
}

AudioEffectVintageReverb::~AudioEffectVintageReverb()
{
  if (ownedMemory_) free(ownedMemory_);
}

void AudioEffectVintageReverb::attach(float *memory, size_t memoryFloats)
{
  core_.init(memory, memoryFloats, AUDIO_SAMPLE_RATE_EXACT, kMaxPreDelayMs);
  apply(typeDefaults(HALL));
}

// ------------------------------------------------------------------ types
// Characteristic settings per type. Columns:
//  type, size, decay, predelay, diffusion, modRate, modDepth,
//  bassMult, bassXover, highMult, highXover, lowCut, highCut
static const AudioEffectVintageReverb::Settings kTypeDefaults[AudioEffectVintageReverb::TYPE_COUNT] = {
  { AudioEffectVintageReverb::AMBIENCE,     0.30f,  0.40f,  0.0f, 1.00f, 1.0f, 0.20f, 0.8f, 300.f, 0.50f, 4000.f,  60.f, 14000.f },
  { AudioEffectVintageReverb::ROOM,         0.40f,  0.90f,  5.0f, 0.85f, 1.2f, 0.30f, 1.0f, 300.f, 0.45f, 4000.f,  40.f, 14000.f },
  { AudioEffectVintageReverb::CHAMBER,      0.60f,  1.60f, 10.0f, 0.90f, 1.0f, 0.35f, 1.1f, 250.f, 0.40f, 3500.f,  30.f, 12000.f },
  { AudioEffectVintageReverb::PLATE,        0.55f,  2.20f,  0.0f, 1.00f, 1.5f, 0.55f, 0.8f, 300.f, 0.60f, 6000.f,  80.f, 16000.f },
  { AudioEffectVintageReverb::HALL,         1.00f,  3.00f, 20.0f, 0.85f, 0.9f, 0.40f, 1.3f, 300.f, 0.40f, 4000.f,  20.f, 12000.f },
  { AudioEffectVintageReverb::CATHEDRAL,    1.50f,  8.00f, 40.0f, 0.80f, 0.6f, 0.45f, 1.6f, 300.f, 0.30f, 3000.f,  20.f,  9000.f },
  { AudioEffectVintageReverb::ARENA,        1.50f,  5.00f, 80.0f, 0.60f, 0.5f, 0.30f, 1.4f, 300.f, 0.35f, 3000.f,  40.f,  8000.f },
  { AudioEffectVintageReverb::CHORUS_SPACE, 1.10f,  4.00f, 15.0f, 0.85f, 2.0f, 0.90f, 1.2f, 300.f, 0.45f, 4000.f,  20.f, 12000.f },
};

static const char *const kTypeNames[AudioEffectVintageReverb::TYPE_COUNT] = {
  "Ambience", "Room", "Chamber", "Plate", "Hall", "Cathedral", "Arena", "Chorus Space"
};

// ---------------------------------------------------------------- presets
struct PresetDef {
  const char *name;
  AudioEffectVintageReverb::Settings s;
};

#define T(x) AudioEffectVintageReverb::x
static const PresetDef kPresets[AudioEffectVintageReverb::PRESET_COUNT] = {
  //                     type             size  decay   pre   diff  mRate mDepth bass  bassXo high  highXo lowCut highCut
  { "Tight Ambience",  { T(AMBIENCE),     0.25f,  0.30f,  0.f, 1.00f, 1.0f, 0.15f, 0.7f, 300.f, 0.50f, 4000.f,  80.f, 14000.f } },
  { "Vocal Ambience",  { T(AMBIENCE),     0.35f,  0.55f,  8.f, 1.00f, 1.0f, 0.20f, 0.8f, 300.f, 0.50f, 5000.f, 100.f, 15000.f } },

  { "Small Room",      { T(ROOM),         0.30f,  0.70f,  3.f, 0.85f, 1.2f, 0.30f, 1.0f, 300.f, 0.45f, 4000.f,  40.f, 14000.f } },
  { "Tiled Bathroom",  { T(ROOM),         0.28f,  1.10f,  2.f, 0.70f, 1.5f, 0.25f, 0.6f, 250.f, 0.80f, 8000.f, 120.f, 18000.f } },
  { "Wood Room",       { T(ROOM),         0.40f,  0.80f,  5.f, 0.90f, 1.0f, 0.30f, 1.0f, 300.f, 0.30f, 2500.f,  40.f, 10000.f } },
  { "Drum Room",       { T(ROOM),         0.45f,  0.60f,  0.f, 0.80f, 1.3f, 0.25f, 0.9f, 300.f, 0.50f, 5000.f,  60.f, 16000.f } },
  { "Large Room",      { T(ROOM),         0.60f,  1.20f, 12.f, 0.85f, 1.1f, 0.35f, 1.1f, 300.f, 0.45f, 4000.f,  30.f, 13000.f } },

  { "Small Chamber",   { T(CHAMBER),      0.50f,  1.30f,  8.f, 0.90f, 1.0f, 0.35f, 1.1f, 250.f, 0.40f, 3500.f,  30.f, 12000.f } },
  { "Vocal Chamber",   { T(CHAMBER),      0.60f,  1.80f, 25.f, 0.95f, 0.9f, 0.35f, 0.9f, 300.f, 0.45f, 4500.f, 120.f, 12000.f } },
  { "Stone Chamber",   { T(CHAMBER),      0.70f,  2.40f, 15.f, 0.80f, 0.8f, 0.30f, 1.3f, 250.f, 0.55f, 5000.f,  30.f, 14000.f } },

  { "Warm Plate",      { T(PLATE),        0.55f,  2.00f,  0.f, 1.00f, 1.4f, 0.50f, 0.9f, 300.f, 0.35f, 3000.f,  80.f, 10000.f } },
  { "Bright Plate",    { T(PLATE),        0.50f,  2.20f,  0.f, 1.00f, 1.6f, 0.50f, 0.7f, 300.f, 0.80f, 8000.f, 100.f, 18000.f } },
  { "Vintage Plate",   { T(PLATE),        0.60f,  2.80f,  0.f, 1.00f, 1.2f, 0.60f, 0.8f, 300.f, 0.45f, 4000.f,  80.f,  8000.f } },
  { "Vocal Plate",     { T(PLATE),        0.55f,  1.70f, 30.f, 1.00f, 1.3f, 0.45f, 0.7f, 350.f, 0.55f, 5000.f, 150.f, 14000.f } },
  { "Drum Plate",      { T(PLATE),        0.45f,  1.30f,  0.f, 1.00f, 1.8f, 0.40f, 0.8f, 300.f, 0.60f, 6000.f, 100.f, 16000.f } },
  { "Long Plate",      { T(PLATE),        0.70f,  5.00f, 10.f, 1.00f, 1.0f, 0.55f, 0.9f, 300.f, 0.40f, 4000.f,  60.f, 12000.f } },

  { "Small Hall",      { T(HALL),         0.80f,  1.80f, 15.f, 0.85f, 1.0f, 0.35f, 1.2f, 300.f, 0.45f, 4000.f,  30.f, 13000.f } },
  { "Concert Hall",    { T(HALL),         1.00f,  2.60f, 20.f, 0.85f, 0.9f, 0.40f, 1.3f, 300.f, 0.40f, 4000.f,  20.f, 12000.f } },
  { "Bright Hall",     { T(HALL),         1.00f,  2.80f, 20.f, 0.85f, 1.0f, 0.40f, 1.1f, 300.f, 0.75f, 7000.f,  30.f, 20000.f } },
  { "Warm Hall",       { T(HALL),         1.10f,  3.20f, 25.f, 0.90f, 0.8f, 0.40f, 1.6f, 350.f, 0.30f, 3000.f,  20.f,  9000.f } },
  { "Vocal Hall",      { T(HALL),         0.90f,  2.40f, 40.f, 0.90f, 0.9f, 0.35f, 0.9f, 300.f, 0.45f, 4500.f, 120.f, 12000.f } },
  { "Large Hall",      { T(HALL),         1.30f,  4.50f, 30.f, 0.85f, 0.7f, 0.45f, 1.4f, 300.f, 0.35f, 3500.f,  20.f, 11000.f } },
  { "Dark Hall",       { T(HALL),         1.20f,  3.80f, 25.f, 0.90f, 0.7f, 0.40f, 1.8f, 400.f, 0.20f, 2500.f,  20.f,  6000.f } },

  { "Arena",           { T(ARENA),        1.40f,  3.50f, 60.f, 0.65f, 0.6f, 0.30f, 1.2f, 300.f, 0.35f, 3500.f,  40.f,  9000.f } },
  { "Stadium",         { T(ARENA),        1.50f,  5.00f, 90.f, 0.55f, 0.5f, 0.30f, 1.3f, 300.f, 0.30f, 3000.f,  40.f,  8000.f } },

  { "Church",          { T(CATHEDRAL),    1.20f,  4.50f, 30.f, 0.85f, 0.7f, 0.40f, 1.4f, 300.f, 0.35f, 3500.f,  20.f, 10000.f } },
  { "Large Cathedral", { T(CATHEDRAL),    1.50f,  9.00f, 45.f, 0.80f, 0.5f, 0.45f, 1.6f, 300.f, 0.30f, 3000.f,  20.f,  9000.f } },
  { "Sanctuary",       { T(CATHEDRAL),    1.40f, 14.00f, 40.f, 0.90f, 0.4f, 0.50f, 1.5f, 300.f, 0.25f, 2500.f,  30.f,  8000.f } },

  { "Chorus Space",    { T(CHORUS_SPACE), 1.10f,  4.00f, 15.f, 0.85f, 2.0f, 0.90f, 1.2f, 300.f, 0.45f, 4000.f,  20.f, 12000.f } },
  { "80s Hall",        { T(CHORUS_SPACE), 1.00f,  3.50f, 30.f, 0.80f, 1.2f, 0.70f, 1.1f, 300.f, 0.40f, 4000.f,  40.f,  8000.f } },
  { "Ambient Wash",    { T(CHORUS_SPACE), 1.50f, 25.00f,  0.f, 0.90f, 0.5f, 0.60f, 1.4f, 300.f, 0.35f, 3000.f,  40.f,  9000.f } },
  { "Infinite Space",  { T(CATHEDRAL),    1.50f, 60.00f,  0.f, 0.90f, 0.3f, 0.50f, 1.0f, 300.f, 0.50f, 4000.f,  30.f, 10000.f } },
};
#undef T

const char *AudioEffectVintageReverb::typeName(Type t)
{
  return t < TYPE_COUNT ? kTypeNames[t] : "?";
}

const AudioEffectVintageReverb::Settings &AudioEffectVintageReverb::typeDefaults(Type t)
{
  return kTypeDefaults[t < TYPE_COUNT ? t : HALL];
}

const char *AudioEffectVintageReverb::presetName(Preset p)
{
  return p < PRESET_COUNT ? kPresets[p].name : "(none)";
}

const AudioEffectVintageReverb::Settings &AudioEffectVintageReverb::presetSettings(Preset p)
{
  return kPresets[p < PRESET_COUNT ? p : CONCERT_HALL].s;
}

void AudioEffectVintageReverb::type(Type t)
{
  if (t >= TYPE_COUNT) return;
  apply(typeDefaults(t));
  preset_ = PRESET_COUNT;
}

void AudioEffectVintageReverb::preset(Preset p)
{
  if (p >= PRESET_COUNT) return;
  apply(kPresets[p].s);
  preset_ = p;
}

void AudioEffectVintageReverb::apply(const Settings &s)
{
  type_ = s.type < TYPE_COUNT ? s.type : HALL;
  preset_ = PRESET_COUNT;
  core_.setSize(s.size);
  core_.setDecaySeconds(s.decay);
  core_.setPreDelayMs(s.predelayMs);
  core_.setDiffusion(s.diffusion);
  core_.setModRate(s.modRate);
  core_.setModDepth(s.modDepth);
  core_.setBassMult(s.bassMult);
  core_.setBassXover(s.bassXover);
  core_.setHighMult(s.highMult);
  core_.setHighXover(s.highXover);
  core_.setLowCut(s.lowCut);
  core_.setHighCut(s.highCut);
}

AudioEffectVintageReverb::Settings AudioEffectVintageReverb::settings() const
{
  Settings s;
  s.type = type_;
  s.size = core_.size();           s.decay = core_.decay();
  s.predelayMs = core_.preDelayMs(); s.diffusion = core_.diffusion();
  s.modRate = core_.modRate();     s.modDepth = core_.modDepth();
  s.bassMult = core_.bassMult();   s.bassXover = core_.bassXover();
  s.highMult = core_.highMult();   s.highXover = core_.highXover();
  s.lowCut = core_.lowCut();       s.highCut = core_.highCut();
  return s;
}

void AudioEffectVintageReverb::update(void)
{
  audio_block_t *inL = receiveReadOnly(0);
  audio_block_t *inR = receiveReadOnly(1);

  if (!core_.ready() || bypass_) {
    // pass through
    if (inL) { transmit(inL, 0); release(inL); }
    if (inR) { transmit(inR, 1); release(inR); }
    return;
  }

  float bufL[AUDIO_BLOCK_SAMPLES];
  float bufR[AUDIO_BLOCK_SAMPLES];
  const float toFloat = 1.0f / 32768.0f;

  if (inL) {
    const int16_t *s = inL->data;
    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) bufL[i] = (float)s[i] * toFloat;
    release(inL);
  } else {
    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) bufL[i] = 0.0f;
  }
  if (inR) {
    const int16_t *s = inR->data;
    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) bufR[i] = (float)s[i] * toFloat;
    release(inR);
  } else {
    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) bufR[i] = 0.0f;
  }

  // The tail must keep running even when no input blocks arrive.
  core_.process(bufL, bufR, bufL, bufR, AUDIO_BLOCK_SAMPLES);

  audio_block_t *outL = allocate();
  audio_block_t *outR = allocate();

  if (outL) {
    int16_t *d = outL->data;
    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
      int32_t v = (int32_t)lrintf(bufL[i] * 32767.0f);
      d[i] = (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
    }
    transmit(outL, 0);
    release(outL);
  }
  if (outR) {
    int16_t *d = outR->data;
    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
      int32_t v = (int32_t)lrintf(bufR[i] * 32767.0f);
      d[i] = (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
    }
    transmit(outR, 1);
    release(outR);
  }
}
