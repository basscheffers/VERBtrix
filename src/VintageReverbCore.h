// VintageReverbCore.h
//
// Platform independent floating point stereo reverb core.
//
// Topology (Griesinger / Dattorro / Costello lineage, the family of designs
// the classic late-70s / 80s digital reverbs and their modern software
// descendants are built on):
//
//   in L/R -> low cut -> high cut -> pre-delay -> 4 x series allpass diffusers
//          -> injected into an N line feedback delay network (FDN)
//
//   FDN loop, per line:  modulated delay (Lagrange interpolation)
//                        -> bass shelf (bass multiplier) -> high shelf (HF damping)
//                        -> in-loop allpass diffuser -> decay gain
//                        -> NxN Hadamard mixing matrix -> back into the delays
//
//   Outputs are decorrelated weighted sums of the N delay outputs plus N early
//   taps read from inside the delay lines.
//
// Two qualities, chosen when init() is called (see Quality below):
//   QUALITY_HIGH: 8 lines, 8x8 Hadamard, 6-point interpolated tank reads.
//   QUALITY_LITE: 4 lines, 4x4 Hadamard, 4-point interpolated tank reads.
//                 About 40 % of the CPU and 76 % of the memory, at the cost of
//                 roughly half the modal density and a slightly shorter
//                 treble tail. Same parameters and presets.
//
// Why this sounds better than a Schroeder comb/allpass reverb:
//   * The orthogonal Hadamard feedback matrix couples all lines, so the modal
//     density is that of the whole network (~20000 samples of loop length at
//     QUALITY_HIGH) instead of one comb at a time. No ringing "metallic" combs.
//   * Slow, incommensurate sine modulation of every delay smears the modes so
//     no single mode ever settles into a stationary ring.
//   * Decay is frequency dependent via first order shelving filters designed
//     from the desired RT60 per band (Jot), so the tail darkens naturally.
//   * Allpass diffusers inside the loop keep raising echo density each pass.
//
// All processing is float. Memory for the delay lines is supplied by the caller
// (see memoryRequired()).
//
// This file has no Arduino/Teensy dependencies so it can be unit tested on a PC.

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

// Fully unroll the per line loops in the audio path so the per line arrays stay
// in FPU registers instead of on the stack (GCC only unrolls at -O3, and the
// Teensy default is -O2). Clang understands the same pragma.
// The small read/write helpers are forced inline for the same reason: at -O2
// GCC leaves the interpolated tank read as a function call, once per line and
// sample. Define VRV_NO_UNROLL to compare without either (see README, CPU).
#if defined(__GNUC__) && !defined(VRV_NO_UNROLL)
#define VRV_UNROLL _Pragma("GCC unroll 8")
#define VRV_INLINE inline __attribute__((always_inline))
#else
#define VRV_UNROLL
#define VRV_INLINE inline
#endif

namespace vrv {

// Quality / cost of the network. Fixed when init() is called because the
// delay memory layout depends on it.
enum Quality : uint8_t {
  QUALITY_HIGH = 0,   // 8 lines, 6-point interpolation (the original VERBtrix)
  QUALITY_LITE = 1    // 4 lines, 4-point interpolation, ~40 % of the CPU
};

static constexpr uint32_t nextPow2(uint32_t v) {
  if (v < 2) return 2;
  v--;
  v |= v >> 1; v |= v >> 2; v |= v >> 4; v |= v >> 8; v |= v >> 16;
  return v + 1;
}

// Integer ceil of a non-negative float, so the memory size calculation can be
// a constant expression (ceilf is not constexpr). Lets the caller write
// EXTMEM float mem[VintageReverbCore::memoryRequired()];
static constexpr uint32_t ceilU(float x) {
  uint32_t i = (uint32_t)x;
  return ((float)i < x) ? i + 1 : i;
}

static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// 6-point, 5th order Lagrange interpolation between y0 and y1 at fraction t
// (0..1); ym2, ym1 lead y0 and y2, y3 follow y1. Chosen over 4-point cubic
// because a fractional delay inside a feedback loop acts as a low pass, and
// that loss is what limits the high frequency decay time: at a half sample
// offset linear interpolation loses 1.5 dB per pass at 8 kHz, cubic Hermite
// 0.3 dB, this one 0.08 dB.
static VRV_INLINE float interp6(float ym2, float ym1, float y0, float y1, float y2, float y3, float t) {
  float ym1py1 = ym1 + y1;
  float ym2py2 = (1.0f / 24.0f) * (ym2 + y2);
  float c1 = (1.0f / 20.0f) * ym2 - 0.5f * ym1 - (1.0f / 3.0f) * y0 + y1 - 0.25f * y2 + (1.0f / 30.0f) * y3;
  float c2 = (2.0f / 3.0f) * ym1py1 - 1.25f * y0 - ym2py2;
  float c3 = (5.0f / 12.0f) * y0 - (7.0f / 12.0f) * y1 + (7.0f / 24.0f) * y2 - (1.0f / 24.0f) * (ym2 + ym1 + y3);
  float c4 = 0.25f * y0 - (1.0f / 6.0f) * ym1py1 + ym2py2;
  float c5 = (1.0f / 120.0f) * (y3 - ym2) + (1.0f / 24.0f) * (ym1 - y2) + (1.0f / 12.0f) * (y1 - y0);
  return ((((c5 * t + c4) * t + c3) * t + c2) * t + c1) * t + y0;
}

// 4-point, 3rd order Lagrange interpolation between y0 and y1 at fraction t,
// used for the tank reads at QUALITY_LITE: about half the arithmetic and four
// loads instead of six, at 0.3 dB loss per pass at 8 kHz on a half sample
// offset (see interp6) and therefore a slightly shorter treble tail.
static VRV_INLINE float interp4(float ym1, float y0, float y1, float y2, float t) {
  float c1 = y1 - (1.0f / 3.0f) * ym1 - 0.5f * y0 - (1.0f / 6.0f) * y2;
  float c2 = 0.5f * (ym1 + y1) - y0;
  float c3 = (1.0f / 6.0f) * (y2 - ym1) + 0.5f * (y0 - y1);
  return ((c3 * t + c2) * t + c1) * t + y0;
}

// Power-of-two circular delay line with integer and fractional reads.
struct DelayLine {
  float   *buf  = nullptr;
  uint32_t mask = 0;
  uint32_t w    = 0;

  void init(float *b, uint32_t sizePow2) {
    buf = b; mask = sizePow2 - 1; w = 0;
    memset(buf, 0, sizeof(float) * sizePow2);
  }
  void clear() { memset(buf, 0, sizeof(float) * (mask + 1)); }

  VRV_INLINE float readInt(uint32_t d) const { return buf[(w - d) & mask]; }

  // Fractional read, 6-point Lagrange. d must be >= 2 and <= mask - 3.
  VRV_INLINE float read6(float d) const {
    int32_t  i = (int32_t)d;
    float    t = d - (float)i;
    uint32_t p = w - (uint32_t)i;
    return interp6(buf[(p + 2) & mask], buf[(p + 1) & mask], buf[p & mask],
                   buf[(p - 1) & mask], buf[(p - 2) & mask], buf[(p - 3) & mask], t);
  }

  // Fractional read, 4-point Lagrange. d must be >= 1 and <= mask - 2.
  VRV_INLINE float read4(float d) const {
    int32_t  i = (int32_t)d;
    float    t = d - (float)i;
    uint32_t p = w - (uint32_t)i;
    return interp4(buf[(p + 1) & mask], buf[p & mask],
                   buf[(p - 1) & mask], buf[(p - 2) & mask], t);
  }

  VRV_INLINE void write(float x) { buf[w & mask] = x; w++; }
};

// Schroeder allpass built on a DelayLine.
struct Allpass {
  DelayLine d;
  VRV_INLINE float processInt(float x, uint32_t len, float g) {
    float dl = d.readInt(len);
    float v  = x + g * dl;
    d.write(v);
    return dl - g * v;
  }
  VRV_INLINE float processFrac(float x, float len, float g) {
    float dl = d.read6(len);
    float v  = x + g * dl;
    d.write(v);
    return dl - g * v;
  }
};

// Non power-of-two delay (pre-delay), 6-point Lagrange interpolation.
struct PreDelay {
  float   *buf  = nullptr;
  uint32_t size = 0;
  uint32_t w    = 0;
  void init(float *b, uint32_t n) { buf = b; size = n; w = 0; memset(b, 0, sizeof(float) * n); }
  void clear() { memset(buf, 0, sizeof(float) * size); }
  // Call after write(): d = 0 would be the sample just written. The two
  // leading interpolation points must exist, so d must be >= 2 (and <= size - 4).
  VRV_INLINE float read6(float d) const {
    int32_t i = (int32_t)d;
    float   t = d - (float)i;
    int32_t p = (int32_t)w - 1 - i;
    if (p < 0) p += (int32_t)size;
    return interp6(at(p + 2), at(p + 1), buf[p], at(p - 1), at(p - 2), at(p - 3), t);
  }
  VRV_INLINE float readInt(uint32_t i) const {
    int32_t p = (int32_t)w - 1 - (int32_t)i;
    if (p < 0) p += (int32_t)size;
    return buf[p];
  }
  VRV_INLINE void write(float x) { buf[w] = x; if (++w >= size) w = 0; }
private:
  VRV_INLINE float at(int32_t k) const {
    if (k < 0) k += (int32_t)size; else if (k >= (int32_t)size) k -= (int32_t)size;
    return buf[k];
  }
};

class VintageReverbCore {
public:
  static constexpr int   kMaxLines     = 8;
  static constexpr int   kLinesHigh    = 8;
  static constexpr int   kLinesLite    = 4;
  static constexpr float kMinSize      = 0.20f;
  static constexpr float kMaxSize      = 1.50f;
  static constexpr float kMaxModSamples = 24.0f;    // modulation depth at modDepth = 1
  static constexpr float kMinDecay     = 0.10f;
  static constexpr float kMaxDecay     = 120.0f;

  // Nominal tank delay lengths in samples at 44.1 kHz (all prime).
  static constexpr uint32_t kTankNomHigh[kLinesHigh] = { 1237, 1381, 1553, 1747, 1949, 2161, 2389, 2617 };
  // Nominal in-loop allpass lengths at 44.1 kHz (all prime).
  static constexpr uint32_t kLoopApNomHigh[kLinesHigh] = { 331, 431, 541, 653, 761, 877, 991, 1103 };
  // QUALITY_LITE: 4 lines spread over a wider range than any 4 of the above,
  // so the total loop length (and with it the modal density) stays as high as
  // 4 lines allow: 9080 + 2852 samples against 15024 + 5688 at QUALITY_HIGH.
  static constexpr uint32_t kTankNomLite[kLinesLite]   = { 1511, 1987, 2503, 3079 };
  static constexpr uint32_t kLoopApNomLite[kLinesLite] = {  331,  541,  877, 1103 };
  // Input diffuser lengths at 44.1 kHz (Dattorro's ratios rescaled, made prime).
  // The same four per channel at both qualities: they cost little and they are
  // what builds the initial echo density.
  static constexpr uint32_t kInApNomL[4] = { 211, 157, 563, 409 };
  static constexpr uint32_t kInApNomR[4] = { 223, 149, 587, 397 };

  static constexpr int lineCount(Quality q) { return q == QUALITY_LITE ? kLinesLite : kLinesHigh; }

  // ---------------------------------------------------------------- memory
  // Number of floats the caller must supply for a given sample rate, maximum
  // pre-delay and quality.
  static constexpr size_t memoryRequired(float sampleRate = 44100.0f, float maxPreDelayMs = 250.0f,
                                         Quality q = QUALITY_HIGH) {
    float scale = sampleRate / 44100.0f;
    const uint32_t *tankNom = tankTable(q);
    const uint32_t *apNom   = loopApTable(q);
    size_t total = 0;
    for (int i = 0; i < lineCount(q); i++) {
      total += nextPow2(ceilU(tankNom[i] * kMaxSize * scale + kMaxModSamples + 8.0f));
      total += nextPow2(ceilU(apNom[i] * kMaxSize * scale + 8.0f));
    }
    for (int i = 0; i < 4; i++) {
      total += nextPow2(ceilU(kInApNomL[i] * scale + 2.0f));
      total += nextPow2(ceilU(kInApNomR[i] * scale + 2.0f));
    }
    total += 2 * preDelaySamples(sampleRate, maxPreDelayMs);
    return total;
  }

  // Attach memory. Returns false if the buffer is too small.
  bool init(float *mem, size_t memFloats, float sampleRate = 44100.0f, float maxPreDelayMs = 250.0f,
            Quality q = QUALITY_HIGH) {
    fs_ = sampleRate;
    fsScale_ = sampleRate / 44100.0f;
    maxPreDelayMs_ = maxPreDelayMs;
    quality_   = q;
    lines_     = lineCount(q);
    tankNom_   = tankTable(q);
    loopApNom_ = loopApTable(q);
    decayTrim_ = (q == QUALITY_LITE) ? kDecayTrimLite : kDecayTrimHigh;
    if (!mem || memFloats < memoryRequired(sampleRate, maxPreDelayMs, q)) { ready_ = false; return false; }
    float *p = mem;
    for (int i = 0; i < lines_; i++) {
      uint32_t n = nextPow2((uint32_t)ceilf(tankNom_[i] * kMaxSize * fsScale_ + kMaxModSamples + 8.0f));
      tank_[i].init(p, n); p += n;
      n = nextPow2((uint32_t)ceilf(loopApNom_[i] * kMaxSize * fsScale_ + 8.0f));
      loopAp_[i].d.init(p, n); p += n;
    }
    for (int i = 0; i < 4; i++) {
      uint32_t n = nextPow2((uint32_t)ceilf(kInApNomL[i] * fsScale_ + 2.0f));
      inApL_[i].d.init(p, n); p += n;
      inApLenL_[i] = (uint32_t)lrintf(kInApNomL[i] * fsScale_);
      n = nextPow2((uint32_t)ceilf(kInApNomR[i] * fsScale_ + 2.0f));
      inApR_[i].d.init(p, n); p += n;
      inApLenR_[i] = (uint32_t)lrintf(kInApNomR[i] * fsScale_);
    }
    uint32_t pd = preDelaySamples(sampleRate, maxPreDelayMs);
    preL_.init(p, pd); p += pd;
    preR_.init(p, pd); p += pd;

    const float *phase0 = (quality_ == QUALITY_LITE) ? kLfoPhase0Lite : kLfoPhase0High;
    for (int i = 0; i < lines_; i++) {
      lfoPhase_[i] = phase0[i];
    }
    resetState();
    // Snap smoothed parameters to their targets so there is no glide at start.
    sSize_ = tSize_; sDecay_ = tDecay_; sPre_ = tPre_; sMix_ = tMix_;
    sBassMult_ = tBassMult_; sHighMult_ = tHighMult_;
    for (int i = 0; i < lines_; i++) {
      float len = tankLength(i, sSize_);
      curLen_[i] = len;
      curApLen_[i] = loopApLength(i, sSize_);
    }
    updateCoefficients(1);
    ready_ = true;
    return true;
  }

  bool ready() const { return ready_; }
  Quality quality() const { return quality_; }
  int lines() const { return lines_; }

  // Clear all delay memory and filter state (kills the tail instantly).
  void resetState() {
    for (int i = 0; i < lines_; i++) {
      tank_[i].clear(); loopAp_[i].d.clear();
      lpBass_[i] = 0.0f; lpHigh_[i] = 0.0f;
    }
    for (int i = 0; i < 4; i++) { inApL_[i].d.clear(); inApR_[i].d.clear(); }
    preL_.clear(); preR_.clear();
    hpInL_ = hpInR_ = 0.0f;
    lp1InL_ = lp2InL_ = lp1InR_ = lp2InR_ = 0.0f;
  }

  // ------------------------------------------------------------ parameters
  // Dry/wet, 0 = dry only, 1 = wet only.
  void setMix(float m)                 { tMix_ = clampf(m, 0.0f, 1.0f); }
  // Pre-delay in milliseconds.
  void setPreDelayMs(float ms)         { tPre_ = clampf(ms, 0.0f, maxPreDelayMs_); }
  // Mid band RT60 in seconds.
  void setDecaySeconds(float s)        { tDecay_ = clampf(s, kMinDecay, kMaxDecay); }
  // Room size, scales every delay in the tank. 0.2 (small room) .. 1.5 (huge hall).
  void setSize(float s)                { tSize_ = clampf(s, kMinSize, kMaxSize); }
  // Diffusion 0..1, sets the allpass coefficients (input and in-loop).
  void setDiffusion(float d)           { diffusion_ = clampf(d, 0.0f, 1.0f); }
  // Modulation rate in Hz (0.05 .. 5) and depth 0..1.
  void setModRate(float hz)            { modRate_ = clampf(hz, 0.0f, 5.0f); }
  void setModDepth(float d)            { modDepth_ = clampf(d, 0.0f, 1.0f); }
  // Bass decay multiplier (0.25 .. 4) below bassXover Hz.
  void setBassMult(float m)            { tBassMult_ = clampf(m, 0.25f, 4.0f); }
  void setBassXover(float hz)          { bassXover_ = clampf(hz, 50.0f, 2000.0f); }
  // High frequency decay multiplier (0.1 .. 1) above highXover Hz.
  void setHighMult(float m)            { tHighMult_ = clampf(m, 0.1f, 1.0f); }
  void setHighXover(float hz)          { highXover_ = clampf(hz, 1000.0f, 16000.0f); }
  // Input band limiting.
  void setLowCut(float hz)             { lowCut_ = clampf(hz, 10.0f, 1000.0f); }
  void setHighCut(float hz)            { highCut_ = clampf(hz, 1000.0f, 20000.0f); }
  // Freeze: infinite decay, input muted.
  void setFreeze(bool f)               { freeze_ = f; }
  // Output level of the wet signal (linear).
  void setWetGain(float g)             { wetGain_ = clampf(g, 0.0f, 4.0f); }

  float mix() const        { return tMix_; }
  float preDelayMs() const { return tPre_; }
  float decay() const      { return tDecay_; }
  float size() const       { return tSize_; }
  float diffusion() const  { return diffusion_; }
  float modRate() const    { return modRate_; }
  float modDepth() const   { return modDepth_; }
  float bassMult() const   { return tBassMult_; }
  float bassXover() const  { return bassXover_; }
  float highMult() const   { return tHighMult_; }
  float highXover() const  { return highXover_; }
  float lowCut() const     { return lowCut_; }
  float highCut() const    { return highCut_; }
  bool  freeze() const     { return freeze_; }

  // -------------------------------------------------------------- process
  // Processes n samples. Output buffers may alias the inputs.
  void process(const float *inL, const float *inR, float *outL, float *outR, int n) {
    if (!ready_ || n <= 0) return;
    if (quality_ == QUALITY_LITE) processN<kLinesLite>(inL, inR, outL, outR, n);
    else                          processN<kLinesHigh>(inL, inR, outL, outR, n);
  }

private:
  // The per sample loop, instantiated once per quality so every table lookup,
  // the matrix and the interpolation order are compile time constants.
  template <int N>
  void processN(const float *inL, const float *inR, float *outL, float *outR, int n) {
    static_assert(N == kLinesHigh || N == kLinesLite, "unsupported line count");
    constexpr bool  kLite      = (N == kLinesLite);
    constexpr const float *kOutL  = kLite ? kOutLLite  : kOutLHigh;
    constexpr const float *kOutR  = kLite ? kOutRLite  : kOutRHigh;
    constexpr const float *kEarlyFrac = kLite ? kEarlyFracLite : kEarlyFracHigh;
    constexpr const float *kEarlyW    = kLite ? kEarlyWLite    : kEarlyWHigh;
    constexpr const float *kMixSign   = kLite ? kMixSignLite   : kMixSignHigh;
    constexpr float kOutScale  = kLite ? kOutScaleLite : kOutScaleHigh;
    // Hadamard normalisation, 1/sqrt(N).
    constexpr float kHadScale  = kLite ? 0.5f : 0.35355339059327373f;

    updateCoefficients(n);

    // Per-sample ramps for the delay lengths (size glide + LFO) and mix.
    const float invN = 1.0f / (float)n;
    float len[N], lenInc[N], apLen[N], apInc[N];
    for (int i = 0; i < N; i++) {
      float target = tankLength(i, sSize_) + lfoValue_[i];
      len[i] = curLen_[i];
      lenInc[i] = (target - curLen_[i]) * invN;
      curLen_[i] = target;
      float apTarget = loopApLength(i, sSize_);
      apLen[i] = curApLen_[i];
      apInc[i] = (apTarget - curApLen_[i]) * invN;
      curApLen_[i] = apTarget;
    }
    float pre = curPre_ + (float)kPreDelayOffset;
    const float preTarget = rintf(sPre_ * 0.001f * fs_);
    float preInc = (preTarget - curPre_) * invN;
    curPre_ = preTarget;
    float mixv = curMix_;
    float mixInc = (sMix_ - curMix_) * invN;
    curMix_ = sMix_;

    // The in-loop allpass and pre-delay targets are whole samples (a half
    // sample in a 331..1103 sample allpass or in the pre-delay is inaudible),
    // so they only need interpolation while gliding to a new size or
    // pre-delay. Once settled they are plain integer reads; only the LFO
    // modulated tank reads pay the per sample polynomial.
    uint32_t apInt[N];
    bool     apStatic[N];
    for (int i = 0; i < N; i++) {
      apStatic[i] = (apInc[i] == 0.0f);
      apInt[i] = (uint32_t)apLen[i];
    }
    const bool     preStatic = (preInc == 0.0f);
    const uint32_t preInt    = (uint32_t)pre;

    // Early tap positions are integer reads. In steady state the LFO moves a
    // line by well under a sample per block, so the positions are fixed per
    // block (saves a multiply and a float->int conversion per line and
    // sample). During a size glide a line can move tens of samples per block
    // and the taps must keep stepping per sample, or they would jump.
    uint32_t tapPos[N];
    bool     tapPerSample = false;
    for (int i = 0; i < N; i++) {
      tapPos[i] = (uint32_t)(len[i] * kEarlyFrac[i]);
      if (fabsf(lenInc[i]) * (float)n * kEarlyFrac[i] >= 1.0f) tapPerSample = true;
    }

    const float gIn   = gInDiff_;
    const float gIn2  = gInDiff2_;
    const float gLoop = gLoopDiff_;
    const float aLC   = aLowCut_;
    const float aHC   = aHighCut_;
    const float aB    = aBass_;
    const float aH    = aHigh_;
    const float inj   = freeze_ ? 0.0f : kInjectGain;
    const float wg    = wetGain_ * kOutScale;

    for (int s = 0; s < n; s++) {
      float dryL = inL[s], dryR = inR[s];

      // ---- input conditioning: one-pole low cut, two-pole high cut
      hpInL_ += aLC * (dryL - hpInL_);   float l = dryL - hpInL_;
      hpInR_ += aLC * (dryR - hpInR_);   float r = dryR - hpInR_;
      lp1InL_ += aHC * (l - lp1InL_);  lp2InL_ += aHC * (lp1InL_ - lp2InL_);  l = lp2InL_;
      lp1InR_ += aHC * (r - lp1InR_);  lp2InR_ += aHC * (lp1InR_ - lp2InR_);  r = lp2InR_;

      // ---- pre-delay
      preL_.write(l); preR_.write(r);
      if (preStatic) { l = preL_.readInt(preInt); r = preR_.readInt(preInt); }
      else           { l = preL_.read6(pre);      r = preR_.read6(pre);      pre += preInc; }

      // ---- input diffusion (4 series allpasses per channel)
      l = inApL_[0].processInt(l, inApLenL_[0], gIn);
      l = inApL_[1].processInt(l, inApLenL_[1], gIn);
      l = inApL_[2].processInt(l, inApLenL_[2], gIn2);
      l = inApL_[3].processInt(l, inApLenL_[3], gIn2);
      r = inApR_[0].processInt(r, inApLenR_[0], gIn);
      r = inApR_[1].processInt(r, inApLenR_[1], gIn);
      r = inApR_[2].processInt(r, inApLenR_[2], gIn2);
      r = inApR_[3].processInt(r, inApLenR_[3], gIn2);

      // ---- tank: read the N modulated delay outputs
      float d[N];
      VRV_UNROLL
      for (int i = 0; i < N; i++) {
        if constexpr (kLite) d[i] = tank_[i].read4(len[i]);
        else                 d[i] = tank_[i].read6(len[i]);
      }

      // ---- outputs: decorrelated sums of the line outputs + early taps
      float wetL = 0.0f, wetR = 0.0f;
      VRV_UNROLL
      for (int i = 0; i < N; i++) {
        wetL += kOutL[i] * d[i];
        wetR += kOutR[i] * d[i];
      }
      if (tapPerSample) {
        VRV_UNROLL
        for (int i = 0; i < N; i++) {
          float e = tank_[i].readInt((uint32_t)(len[i] * kEarlyFrac[i]));
          if (i & 1) wetR += kEarlyW[i] * e; else wetL += kEarlyW[i] * e;
        }
      } else {
        VRV_UNROLL
        for (int i = 0; i < N; i++) {
          float e = tank_[i].readInt(tapPos[i]);
          if (i & 1) wetR += kEarlyW[i] * e; else wetL += kEarlyW[i] * e;
        }
      }

      // ---- per line: shelving decay filters, decay gain, in-loop allpass
      float f[N];
      VRV_UNROLL
      for (int i = 0; i < N; i++) {
        float x = d[i];
        lpBass_[i] += aB * (x - lpBass_[i]);
        x += shelfBass_[i] * lpBass_[i];               // bass shelf: 1 + (GL-1)*LP
        lpHigh_[i] += aH * (x - lpHigh_[i]);
        x += shelfHigh_[i] * (x - lpHigh_[i]);         // high shelf: 1 + (GH-1)*HP
        x *= gMid_[i];
        x = apStatic[i] ? loopAp_[i].processInt(x, apInt[i], gLoop)
                        : loopAp_[i].processFrac(x, apLen[i], gLoop);
        f[i] = kMixSign[i] * x;
      }

      // ---- NxN Hadamard (fast Walsh-Hadamard), scaled by 1/sqrt(N)
      float m[N];
      if constexpr (kLite) {
        // 4x4, 8 adds
        float a0 = f[0] + f[1], a1 = f[0] - f[1], a2 = f[2] + f[3], a3 = f[2] - f[3];
        m[0] = (a0 + a2) * kHadScale; m[1] = (a1 + a3) * kHadScale;
        m[2] = (a0 - a2) * kHadScale; m[3] = (a1 - a3) * kHadScale;
      } else {
        // 8x8, 24 adds
        float a0 = f[0] + f[1], a1 = f[0] - f[1], a2 = f[2] + f[3], a3 = f[2] - f[3];
        float a4 = f[4] + f[5], a5 = f[4] - f[5], a6 = f[6] + f[7], a7 = f[6] - f[7];
        float b0 = a0 + a2, b1 = a1 + a3, b2 = a0 - a2, b3 = a1 - a3;
        float b4 = a4 + a6, b5 = a5 + a7, b6 = a4 - a6, b7 = a5 - a7;
        m[0] = (b0 + b4) * kHadScale; m[1] = (b1 + b5) * kHadScale;
        m[2] = (b2 + b6) * kHadScale; m[3] = (b3 + b7) * kHadScale;
        m[4] = (b0 - b4) * kHadScale; m[5] = (b1 - b5) * kHadScale;
        m[6] = (b2 - b6) * kHadScale; m[7] = (b3 - b7) * kHadScale;
      }

      // ---- inject diffused input (L into even lines, R into odd) and write
      float il = inj * l, ir = inj * r;
      if constexpr (kLite) {
        tank_[0].write(m[0] + il);  tank_[1].write(m[1] + ir);
        tank_[2].write(m[2] - il);  tank_[3].write(m[3] - ir);
      } else {
        tank_[0].write(m[0] + il);  tank_[1].write(m[1] + ir);
        tank_[2].write(m[2] - il);  tank_[3].write(m[3] - ir);
        tank_[4].write(m[4] + il);  tank_[5].write(m[5] + ir);
        tank_[6].write(m[6] - il);  tank_[7].write(m[7] - ir);
      }

      // ---- advance ramps
      VRV_UNROLL
      for (int i = 0; i < N; i++) { len[i] += lenInc[i]; apLen[i] += apInc[i]; }

      // ---- dry/wet mix
      wetL *= wg; wetR *= wg;
      outL[s] = dryL + mixv * (wetL - dryL);
      outR[s] = dryR + mixv * (wetR - dryR);
      mixv += mixInc;
    }
  }

  // Output weight vectors. kOutL . kOutR = 0 so left and right are
  // decorrelated; even lines (fed from L) weigh more on the left and odd
  // lines (fed from R) on the right to keep some of the source image. Both
  // vectors have unit norm at either quality.
  static constexpr float kOutLHigh[kLinesHigh] = { 0.4472f,  0.2236f, -0.4472f, -0.2236f,  0.4472f,  0.2236f, -0.4472f, -0.2236f };
  static constexpr float kOutRHigh[kLinesHigh] = { 0.2236f, -0.4472f, -0.2236f,  0.4472f,  0.2236f, -0.4472f, -0.2236f,  0.4472f };
  static constexpr float kOutLLite[kLinesLite] = { 0.6325f,  0.3162f, -0.6325f, -0.3162f };
  static constexpr float kOutRLite[kLinesLite] = { 0.3162f, -0.6325f, -0.3162f,  0.6325f };
  // Early tap positions as fraction of the line length and their weights.
  // Two taps per channel either way, so the lite weights are larger to keep
  // the same early energy (4 x 0.25^2 = 2 x 0.3536^2).
  static constexpr float kEarlyFracHigh[kLinesHigh] = { 0.31f, 0.57f, 0.23f, 0.71f, 0.43f, 0.83f, 0.37f, 0.63f };
  static constexpr float kEarlyWHigh[kLinesHigh]    = { 0.25f, -0.25f, -0.25f, 0.25f, 0.25f, -0.25f, -0.25f, 0.25f };
  static constexpr float kEarlyFracLite[kLinesLite] = { 0.31f, 0.57f, 0.43f, 0.83f };
  static constexpr float kEarlyWLite[kLinesLite]    = { 0.3536f, 0.3536f, -0.3536f, -0.3536f };
  // Sign flips ahead of the Hadamard to break its regular structure.
  static constexpr float kMixSignHigh[kLinesHigh] = { 1.0f, -1.0f, -1.0f, 1.0f, -1.0f, 1.0f, 1.0f, -1.0f };
  static constexpr float kMixSignLite[kLinesLite] = { 1.0f, -1.0f, -1.0f, 1.0f };
  // LFO rate ratios (incommensurate) and start phases.
  static constexpr float kLfoRatioHigh[kLinesHigh]  = { 1.000f, 1.127f, 0.871f, 1.313f, 0.793f, 1.531f, 0.709f, 1.211f };
  static constexpr float kLfoPhase0High[kLinesHigh] = { 0.00f, 0.83f, 1.71f, 2.53f, 3.37f, 4.19f, 5.02f, 5.87f };
  static constexpr float kLfoRatioLite[kLinesLite]  = { 1.000f, 0.793f, 1.313f, 1.531f };
  static constexpr float kLfoPhase0Lite[kLinesLite] = { 0.00f, 1.71f, 3.37f, 5.02f };
  static constexpr float kInjectGain = 0.5f;
  // The pre-delay read needs two samples of lead for its interpolator, so
  // "0 ms" is really 2 samples (45 us). Nobody can hear that.
  static constexpr uint32_t kPreDelayOffset = 2;
  // Compensates the extra group delay of the in-loop allpasses so RT60 comes
  // out right; calibrated per quality (the allpasses are a larger share of
  // the loop at QUALITY_LITE).
  static constexpr float kDecayTrimHigh = 1.045f;
  static constexpr float kDecayTrimLite = 1.062f;
  // Output scale: calibrated so stationary noise at decay 2 s / size 1.0 comes
  // out about 0.5 dB below the dry level at 100 % wet. Half the lines are fed
  // and summed at QUALITY_LITE, hence the larger value.
  static constexpr float kOutScaleHigh = 1.5f;
  static constexpr float kOutScaleLite = 1.61f;

  static constexpr const uint32_t *tankTable(Quality q) {
    return q == QUALITY_LITE ? kTankNomLite : kTankNomHigh;
  }
  static constexpr const uint32_t *loopApTable(Quality q) {
    return q == QUALITY_LITE ? kLoopApNomLite : kLoopApNomHigh;
  }

  static constexpr uint32_t preDelaySamples(float fs, float maxMs) {
    return ceilU(maxMs * 0.001f * fs) + kPreDelayOffset + 8;
  }
  inline float tankLength(int i, float size) const {
    return (float)tankNom_[i] * size * fsScale_;
  }
  // Rounded to a whole sample: the allpass only has to be interpolated while
  // the size glides, and a half sample here is inaudible.
  inline float loopApLength(int i, float size) const {
    return rintf((float)loopApNom_[i] * size * fsScale_);
  }
  static inline float onePoleCoef(float hz, float fs) {
    return 1.0f - expf(-6.283185307f * hz / fs);
  }

  // Block-rate parameter smoothing and coefficient calculation.
  void updateCoefficients(int n) {
    // smoothing: ~40 ms for most, ~250 ms for size (glide instead of zipper)
    const float aFast = 1.0f - expf(-(float)n / (0.040f * fs_));
    const float aSize = 1.0f - expf(-(float)n / (0.250f * fs_));
    sSize_     += aSize * (tSize_ - sSize_);
    sDecay_    += aFast * (tDecay_ - sDecay_);
    sPre_      += aFast * (tPre_ - sPre_);
    sMix_      += aFast * (tMix_ - sMix_);
    sBassMult_ += aFast * (tBassMult_ - sBassMult_);
    sHighMult_ += aFast * (tHighMult_ - sHighMult_);

    gInDiff_   = 0.75f * diffusion_;
    gInDiff2_  = 0.625f * diffusion_;
    gLoopDiff_ = 0.55f * diffusion_;
    aLowCut_   = onePoleCoef(lowCut_, fs_);
    aHighCut_  = onePoleCoef(highCut_, fs_);
    aBass_     = onePoleCoef(bassXover_, fs_);
    aHigh_     = onePoleCoef(highXover_, fs_);

    // Decay gains from RT60 (Jot): g = 10^(-3 L / (fs T60)) for the total
    // loop length of each line (delay + average group delay of its allpass).
    const float lnk = -6.907755279f; // ln(1e-3)
    for (int i = 0; i < lines_; i++) {
      float L = decayTrim_ * (tankLength(i, sSize_) + loopApLength(i, sSize_)) / fs_;
      float gM, gL, gH;
      if (freeze_) {
        gM = gL = gH = 1.0f;
      } else {
        gM = expf(lnk * L / sDecay_);
        gL = expf(lnk * L / (sDecay_ * sBassMult_));
        gH = expf(lnk * L / (sDecay_ * sHighMult_));
        gM = clampf(gM, 0.0f, 0.99995f);
        gL = clampf(gL, 0.0f, 0.99995f);
        gH = clampf(gH, 0.0f, gM);
      }
      gMid_[i]      = gM;
      shelfBass_[i] = (gM > 0.0f) ? (gL / gM - 1.0f) : 0.0f;   // (GL - 1)
      shelfHigh_[i] = (gM > 0.0f) ? (gH / gM - 1.0f) : 0.0f;   // (GH - 1)
    }

    // LFOs, advanced once per block
    const float *ratio = (quality_ == QUALITY_LITE) ? kLfoRatioLite : kLfoRatioHigh;
    const float depth = modDepth_ * kMaxModSamples;
    const float twoPi = 6.283185307f;
    for (int i = 0; i < lines_; i++) {
      lfoPhase_[i] += twoPi * modRate_ * ratio[i] * (float)n / fs_;
      if (lfoPhase_[i] > twoPi) lfoPhase_[i] -= twoPi;
      // depth limited so tiny rooms never read across the write pointer
      float maxD = 0.2f * tankLength(i, sSize_);
      float dep = depth < maxD ? depth : maxD;
      lfoValue_[i] = dep * sinf(lfoPhase_[i]);
    }
  }

  // ---------------------------------------------------------------- state
  bool  ready_ = false;
  float fs_ = 44100.0f, fsScale_ = 1.0f, maxPreDelayMs_ = 250.0f;

  Quality quality_ = QUALITY_HIGH;
  int     lines_   = kLinesHigh;
  const uint32_t *tankNom_   = kTankNomHigh;
  const uint32_t *loopApNom_ = kLoopApNomHigh;
  float   decayTrim_ = kDecayTrimHigh;

  DelayLine tank_[kMaxLines];
  Allpass   loopAp_[kMaxLines];
  Allpass   inApL_[4], inApR_[4];
  uint32_t  inApLenL_[4] = {0}, inApLenR_[4] = {0};
  PreDelay  preL_, preR_;

  float lpBass_[kMaxLines] = {0}, lpHigh_[kMaxLines] = {0};
  float hpInL_ = 0, hpInR_ = 0, lp1InL_ = 0, lp2InL_ = 0, lp1InR_ = 0, lp2InR_ = 0;

  float curLen_[kMaxLines] = {0}, curApLen_[kMaxLines] = {0};
  float lfoPhase_[kMaxLines] = {0}, lfoValue_[kMaxLines] = {0};
  float curPre_ = 0.0f, curMix_ = 1.0f;

  // targets (set from the control thread) and smoothed values
  float tMix_ = 1.0f,  sMix_ = 1.0f;
  float tPre_ = 0.0f,  sPre_ = 0.0f;
  float tDecay_ = 2.5f, sDecay_ = 2.5f;
  float tSize_ = 1.0f, sSize_ = 1.0f;
  float tBassMult_ = 1.0f, sBassMult_ = 1.0f;
  float tHighMult_ = 0.5f, sHighMult_ = 0.5f;
  float diffusion_ = 0.85f;
  float modRate_ = 1.0f, modDepth_ = 0.4f;
  float bassXover_ = 300.0f, highXover_ = 4000.0f;
  float lowCut_ = 20.0f, highCut_ = 16000.0f;
  float wetGain_ = 1.0f;
  bool  freeze_ = false;

  // derived coefficients
  float gInDiff_ = 0, gInDiff2_ = 0, gLoopDiff_ = 0;
  float aLowCut_ = 0, aHighCut_ = 1, aBass_ = 0, aHigh_ = 0;
  float gMid_[kMaxLines] = {0}, shelfBass_[kMaxLines] = {0}, shelfHigh_[kMaxLines] = {0};
};

} // namespace vrv
