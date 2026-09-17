// Offline test harness for VintageReverbCore.
// Build:  clang++ -O2 -std=c++17 -o test_reverb test_reverb.cpp && ./test_reverb
// Writes WAV files into ./out for listening and prints measurements.

#include "../../src/VintageReverbCore.h"
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <chrono>
#include <sys/stat.h>

using vrv::VintageReverbCore;

static const float FS = 44100.0f;
static const int BLOCK = 128;

struct Rev {
  std::vector<float> mem;
  VintageReverbCore core;
  Rev() {
    mem.resize(VintageReverbCore::memoryRequired(FS, 250.0f));
    if (!core.init(mem.data(), mem.size(), FS, 250.0f)) { fprintf(stderr, "init failed\n"); exit(1); }
  }
  void run(const std::vector<float>& inL, const std::vector<float>& inR,
           std::vector<float>& outL, std::vector<float>& outR) {
    size_t n = inL.size();
    outL.assign(n, 0.0f); outR.assign(n, 0.0f);
    for (size_t p = 0; p < n; p += BLOCK) {
      int len = (int)std::min((size_t)BLOCK, n - p);
      core.process(&inL[p], &inR[p], &outL[p], &outR[p], len);
    }
  }
};

static void writeWav(const std::string& path, const std::vector<float>& L, const std::vector<float>& R) {
  FILE* f = fopen(path.c_str(), "wb");
  if (!f) { fprintf(stderr, "cannot write %s\n", path.c_str()); return; }
  uint32_t n = (uint32_t)L.size();
  uint32_t dataBytes = n * 4;
  auto w32 = [&](uint32_t v) { fwrite(&v, 4, 1, f); };
  auto w16 = [&](uint16_t v) { fwrite(&v, 2, 1, f); };
  fwrite("RIFF", 1, 4, f); w32(36 + dataBytes); fwrite("WAVE", 1, 4, f);
  fwrite("fmt ", 1, 4, f); w32(16); w16(1); w16(2); w32(44100); w32(44100 * 4); w16(4); w16(16);
  fwrite("data", 1, 4, f); w32(dataBytes);
  for (uint32_t i = 0; i < n; i++) {
    float l = std::max(-1.0f, std::min(1.0f, L[i]));
    float r = std::max(-1.0f, std::min(1.0f, R[i]));
    int16_t sl = (int16_t)lrintf(l * 32767.0f), sr = (int16_t)lrintf(r * 32767.0f);
    fwrite(&sl, 2, 1, f); fwrite(&sr, 2, 1, f);
  }
  fclose(f);
}

// Schroeder backward integration -> RT60 estimated from the -5..-35 dB slope.
static float measureT60(const std::vector<float>& ir) {
  size_t n = ir.size();
  std::vector<double> edc(n);
  double acc = 0;
  for (size_t i = n; i-- > 0;) { acc += (double)ir[i] * ir[i]; edc[i] = acc; }
  double e0 = edc[0];
  size_t i5 = 0, i35 = 0;
  for (size_t i = 0; i < n; i++) { if (10 * log10(edc[i] / e0) < -5)  { i5 = i;  break; } }
  for (size_t i = 0; i < n; i++) { if (10 * log10(edc[i] / e0) < -35) { i35 = i; break; } }
  if (i35 <= i5) return -1.0f;
  double slope = 30.0 / ((i35 - i5) / FS);  // dB per second
  return (float)(60.0 / slope);
}

// Band limited T60 via a simple 2nd order Butterworth-ish bandpass (biquad) around fc.
static std::vector<float> bandpass(const std::vector<float>& x, float fc, float Q) {
  float w0 = 2 * M_PI * fc / FS, alpha = sinf(w0) / (2 * Q);
  float b0 = alpha, b1 = 0, b2 = -alpha, a0 = 1 + alpha, a1 = -2 * cosf(w0), a2 = 1 - alpha;
  b0 /= a0; b1 /= a0; b2 /= a0; a1 /= a0; a2 /= a0;
  std::vector<float> y(x.size());
  float x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  for (size_t i = 0; i < x.size(); i++) {
    float v = b0 * x[i] + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1; x1 = x[i]; y2 = y1; y1 = v; y[i] = v;
  }
  return y;
}

// Normalized echo density (Abel & Huang 2006). 1.0 = Gaussian, fully mixed.
static float echoDensityAt(const std::vector<float>& ir, size_t center, size_t halfWin) {
  size_t a = center > halfWin ? center - halfWin : 0;
  size_t b = std::min(ir.size(), center + halfWin);
  double sq = 0; for (size_t i = a; i < b; i++) sq += (double)ir[i] * ir[i];
  double sd = sqrt(sq / (b - a));
  size_t cnt = 0; for (size_t i = a; i < b; i++) if (fabs(ir[i]) > sd) cnt++;
  return (float)((cnt / (double)(b - a)) / erfc(1.0 / sqrt(2.0)));
}

static float rms(const std::vector<float>& x, size_t a, size_t b) {
  double s = 0; for (size_t i = a; i < b; i++) s += (double)x[i] * x[i];
  return (float)sqrt(s / (b - a));
}
static float peak(const std::vector<float>& x) {
  float p = 0; for (float v : x) p = std::max(p, fabsf(v)); return p;
}
static bool hasNaN(const std::vector<float>& x) { for (float v : x) if (!std::isfinite(v)) return true; return false; }

// ------------------------------------------------------------------ signals
static void addNoiseBurst(std::vector<float>& L, std::vector<float>& R, size_t at, float ms, float amp, uint32_t seed) {
  uint32_t s = seed;
  size_t n = (size_t)(ms * 0.001f * FS);
  for (size_t i = 0; i < n && at + i < L.size(); i++) {
    s = s * 1664525u + 1013904223u; float v = ((s >> 8) / 8388608.0f - 1.0f);
    float env = expf(-4.0f * i / (float)n);
    L[at + i] += amp * env * v;
    s = s * 1664525u + 1013904223u; v = ((s >> 8) / 8388608.0f - 1.0f);
    R[at + i] += amp * env * v;
  }
}
static void addPluck(std::vector<float>& L, std::vector<float>& R, size_t at, float freq, float ms, float amp, float pan) {
  size_t n = (size_t)(ms * 0.001f * FS);
  float ph = 0;
  for (size_t i = 0; i < n && at + i < L.size(); i++) {
    float env = expf(-5.0f * i / (float)n);
    // band limited saw-ish: sum of 8 harmonics with 1/k amplitude
    float v = 0; for (int k = 1; k <= 8; k++) v += sinf(2 * M_PI * k * ph) / k;
    ph += freq / FS; if (ph >= 1) ph -= 1;
    L[at + i] += amp * env * v * (1 - pan);
    R[at + i] += amp * env * v * pan;
  }
}

struct Preset { const char* name; float size, decay, pre, diff, modRate, modDepth, bassMult, highMult, highXover, highCut; };

int main() {
  mkdir("out", 0755);
  printf("memory required: %zu floats = %.1f KB\n\n",
         VintageReverbCore::memoryRequired(FS, 250.0f), VintageReverbCore::memoryRequired(FS, 250.0f) * 4 / 1024.0);

  // ---------------------------------------------------------- 1. RT60 accuracy
  printf("== RT60 calibration (mod off, size 1.0, bassMult 1, highMult 1) ==\n");
  for (float t60 : {0.5f, 1.0f, 2.0f, 4.0f, 8.0f}) {
    Rev r;
    r.core.setModDepth(0); r.core.setBassMult(1); r.core.setHighMult(1); r.core.setHighCut(20000); r.core.setLowCut(10);
    r.core.setDecaySeconds(t60); r.core.setSize(1.0f);
    // re-init to snap smoothing
    r.core.init(r.mem.data(), r.mem.size(), FS, 250.0f);
    size_t n = (size_t)(FS * (t60 * 1.5f + 1.0f));
    std::vector<float> iL(n, 0), iR(n, 0), oL, oR;
    iL[0] = 1.0f; iR[0] = 1.0f;
    r.run(iL, iR, oL, oR);
    float m = measureT60(oL);
    printf("  target %5.2f s  measured %5.2f s  (%+.1f%%)  peak %.3f  nan:%d\n", t60, m, 100 * (m / t60 - 1), peak(oL), hasNaN(oL));
  }

  // Fractional delay interpolation inside the loop is a low pass, so the high
  // bands decay faster than asked. Size 1.0 with modulation off has integer
  // delays everywhere and is the reference; 0.5 and 1.5 put every delay on a
  // half sample, the worst case.
  printf("\n== RT60 vs size and modulation (target 2.0 s, bassMult 1, highMult 1) ==\n");
  printf("   mod   size   broad  250Hz   1kHz   4kHz   8kHz  12kHz\n");
  for (float md : {0.0f, 0.4f}) {
    for (float sz : {0.2f, 0.5f, 1.0f, 1.5f}) {
      Rev r;
      r.core.setModDepth(md); r.core.setBassMult(1); r.core.setHighMult(1); r.core.setHighCut(20000); r.core.setLowCut(10);
      r.core.setDecaySeconds(2.0f); r.core.setSize(sz);
      r.core.init(r.mem.data(), r.mem.size(), FS, 250.0f);
      size_t n = (size_t)(FS * 5);
      std::vector<float> iL(n, 0), iR(n, 0), oL, oR; iL[0] = 1; iR[0] = 1;
      r.run(iL, iR, oL, oR);
      printf("  %4.1f   %.2f   %5.2f", md, sz, measureT60(oL));
      for (float fc : {250.f, 1000.f, 4000.f, 8000.f, 12000.f}) printf("  %5.2f", measureT60(bandpass(oL, fc, 2.0f)));
      printf("\n");
    }
  }

  // ---------------------------------------------------------- 2. frequency dependent decay
  printf("\n== Band RT60 (target mid 2.0 s, bassMult 1.5 @300Hz, highMult 0.4 @4kHz) ==\n");
  {
    Rev r;
    r.core.setModDepth(0); r.core.setBassMult(1.5f); r.core.setBassXover(300); r.core.setHighMult(0.4f); r.core.setHighXover(4000);
    r.core.setHighCut(20000); r.core.setLowCut(10); r.core.setDecaySeconds(2.0f);
    r.core.init(r.mem.data(), r.mem.size(), FS, 250.0f);
    size_t n = (size_t)(FS * 5);
    std::vector<float> iL(n, 0), iR(n, 0), oL, oR; iL[0] = 1; iR[0] = 1;
    r.run(iL, iR, oL, oR);
    for (float fc : {100.f, 250.f, 500.f, 1000.f, 2000.f, 4000.f, 8000.f, 12000.f}) {
      printf("  %6.0f Hz : %5.2f s\n", fc, measureT60(bandpass(oL, fc, 2.0f)));
    }
  }

  // ---------------------------------------------------------- 3. stability worst case
  printf("\n== Stability: decay 120 s, bassMult 4, size 1.5, full mod, 60 s of silence after a burst ==\n");
  {
    Rev r;
    r.core.setDecaySeconds(120); r.core.setBassMult(4); r.core.setHighMult(1); r.core.setSize(1.5f); r.core.setModDepth(1); r.core.setModRate(5);
    r.core.init(r.mem.data(), r.mem.size(), FS, 250.0f);
    size_t n = (size_t)(FS * 60);
    std::vector<float> iL(n, 0), iR(n, 0), oL, oR;
    addNoiseBurst(iL, iR, 0, 50, 1.0f, 1);
    r.run(iL, iR, oL, oR);
    size_t sec = (size_t)FS;
    printf("  rms @1s %.4f  @10s %.4f  @30s %.4f  @59s %.4f   peak %.3f  nan:%d\n",
           rms(oL, 1 * sec, 2 * sec), rms(oL, 10 * sec, 11 * sec), rms(oL, 30 * sec, 31 * sec), rms(oL, 58 * sec, 59 * sec), peak(oL), hasNaN(oL));
  }
  printf("\n== Freeze: level should stay constant ==\n");
  {
    Rev r;
    r.core.setDecaySeconds(2); r.core.setSize(1.0f);
    r.core.init(r.mem.data(), r.mem.size(), FS, 250.0f);
    size_t n = (size_t)(FS * 30);
    std::vector<float> iL(n, 0), iR(n, 0), oL(n), oR(n);
    addNoiseBurst(iL, iR, 0, 50, 1.0f, 2);
    size_t p = 0;
    for (; p < (size_t)(FS * 0.3f); p += BLOCK) r.core.process(&iL[p], &iR[p], &oL[p], &oR[p], BLOCK);
    r.core.setFreeze(true);
    for (; p + BLOCK <= n; p += BLOCK) r.core.process(&iL[p], &iR[p], &oL[p], &oR[p], BLOCK);
    size_t sec = (size_t)FS;
    printf("  rms @2s %.4f  @10s %.4f  @29s %.4f  nan:%d\n", rms(oL, 2 * sec, 3 * sec), rms(oL, 10 * sec, 11 * sec), rms(oL, 28 * sec, 29 * sec), hasNaN(oL));
  }

  // ---------------------------------------------------------- 4. echo density / mixing time
  printf("\n== Normalized echo density of the impulse response (1.0 = Gaussian tail) ==\n");
  for (float sz : {0.35f, 1.0f}) {
    Rev r;
    r.core.setDecaySeconds(3); r.core.setSize(sz); r.core.setPreDelayMs(0);
    r.core.init(r.mem.data(), r.mem.size(), FS, 250.0f);
    size_t n = (size_t)(FS * 2);
    std::vector<float> iL(n, 0), iR(n, 0), oL, oR; iL[0] = 1; iR[0] = 1;
    r.run(iL, iR, oL, oR);
    printf("  size %.2f :", sz);
    for (float ms : {20.f, 40.f, 60.f, 80.f, 100.f, 150.f, 200.f, 300.f}) {
      printf("  %3.0fms=%.2f", ms, echoDensityAt(oL, (size_t)(ms * 0.001f * FS), (size_t)(0.010f * FS)));
    }
    printf("\n");
  }

  // ---------------------------------------------------------- 5. stereo decorrelation
  printf("\n== L/R correlation of the tail (0 = fully decorrelated) ==\n");
  {
    Rev r;
    r.core.setDecaySeconds(3); r.core.setSize(1.0f);
    r.core.init(r.mem.data(), r.mem.size(), FS, 250.0f);
    size_t n = (size_t)(FS * 3);
    std::vector<float> iL(n, 0), iR(n, 0), oL, oR; iL[0] = 1; iR[0] = 1;
    r.run(iL, iR, oL, oR);
    size_t a = (size_t)(0.2f * FS), b = (size_t)(2.0f * FS);
    double sll = 0, srr = 0, slr = 0;
    for (size_t i = a; i < b; i++) { sll += (double)oL[i] * oL[i]; srr += (double)oR[i] * oR[i]; slr += (double)oL[i] * oR[i]; }
    printf("  corr = %.3f   (L rms %.4f, R rms %.4f)\n", slr / sqrt(sll * srr), sqrt(sll / (b - a)), sqrt(srr / (b - a)));
  }

  // ---------------------------------------------------------- 6. level calibration
  printf("\n== Wet level: stationary white noise in, decay 2 s, size 1.0, 100%% wet ==\n");
  {
    Rev r;
    r.core.setDecaySeconds(2); r.core.setSize(1.0f);
    r.core.init(r.mem.data(), r.mem.size(), FS, 250.0f);
    size_t n = (size_t)(FS * 8);
    std::vector<float> iL(n), iR(n), oL, oR;
    uint32_t s = 7;
    for (size_t i = 0; i < n; i++) {
      s = s * 1664525u + 1013904223u; iL[i] = 0.25f * ((s >> 8) / 8388608.0f - 1.0f);
      s = s * 1664525u + 1013904223u; iR[i] = 0.25f * ((s >> 8) / 8388608.0f - 1.0f);
    }
    r.run(iL, iR, oL, oR);
    size_t a = (size_t)(5 * FS), b = n;
    printf("  dry rms %.4f  wet rms %.4f  ratio %+.1f dB  wet peak %.3f\n", rms(iL, a, b), rms(oL, a, b), 20 * log10(rms(oL, a, b) / rms(iL, a, b)), peak(oL));
  }

  // ---------------------------------------------------------- 7. speed (host CPU, indicative only)
  {
    Rev r;
    size_t n = (size_t)(FS * 20);
    std::vector<float> iL(n, 0.01f), iR(n, 0.01f), oL, oR;
    auto t0 = std::chrono::high_resolution_clock::now();
    r.run(iL, iR, oL, oR);
    auto t1 = std::chrono::high_resolution_clock::now();
    double sec = std::chrono::duration<double>(t1 - t0).count();
    printf("\nhost speed: %.1f ns/sample (%.2fx realtime on this machine)\n", sec / n * 1e9, 20.0 / sec);
  }

  // ---------------------------------------------------------- 8. listening files
  printf("\n== Writing listening files to ./out ==\n");
  Preset presets[] = {
    { "room",      0.35f, 0.9f,  5.0f, 0.85f, 1.2f, 0.30f, 1.0f, 0.45f, 4000.f, 14000.f },
    { "plate",     0.55f, 2.2f,  0.0f, 1.00f, 1.5f, 0.55f, 0.8f, 0.60f, 6000.f, 16000.f },
    { "hall",      1.00f, 3.0f, 20.0f, 0.85f, 0.9f, 0.40f, 1.3f, 0.40f, 4000.f, 12000.f },
    { "cathedral", 1.50f, 8.0f, 40.0f, 0.80f, 0.6f, 0.45f, 1.6f, 0.30f, 3000.f,  9000.f },
  };
  for (const Preset& p : presets) {
    Rev r;
    r.core.setSize(p.size); r.core.setDecaySeconds(p.decay); r.core.setPreDelayMs(p.pre); r.core.setDiffusion(p.diff);
    r.core.setModRate(p.modRate); r.core.setModDepth(p.modDepth); r.core.setBassMult(p.bassMult); r.core.setHighMult(p.highMult);
    r.core.setHighXover(p.highXover); r.core.setHighCut(p.highCut);
    r.core.init(r.mem.data(), r.mem.size(), FS, 250.0f);
    r.core.setSize(p.size); r.core.setDecaySeconds(p.decay); r.core.setPreDelayMs(p.pre); r.core.setDiffusion(p.diff);
    r.core.setModRate(p.modRate); r.core.setModDepth(p.modDepth); r.core.setBassMult(p.bassMult); r.core.setHighMult(p.highMult);
    r.core.setHighXover(p.highXover); r.core.setHighCut(p.highCut);
    float total = 4.0f + p.decay * 1.2f + 4.0f;
    size_t n = (size_t)(FS * total);
    std::vector<float> iL(n, 0), iR(n, 0), oL, oR;
    // a click, a snare-ish burst, then a few plucked notes, then a chord
    iL[(size_t)(0.2f * FS)] = 0.5f; iR[(size_t)(0.2f * FS)] = 0.5f;
    addNoiseBurst(iL, iR, (size_t)(1.5f * FS), 12.0f, 0.7f, 11);
    addPluck(iL, iR, (size_t)(2.8f * FS), 220.0f, 900, 0.35f, 0.3f);
    addPluck(iL, iR, (size_t)(3.4f * FS), 329.6f, 900, 0.35f, 0.7f);
    addPluck(iL, iR, (size_t)(4.0f * FS), 440.0f, 900, 0.35f, 0.5f);
    addPluck(iL, iR, (size_t)(4.8f * FS), 261.6f, 1500, 0.25f, 0.4f);
    addPluck(iL, iR, (size_t)(4.8f * FS), 392.0f, 1500, 0.25f, 0.6f);
    addPluck(iL, iR, (size_t)(4.8f * FS), 523.3f, 1500, 0.25f, 0.5f);
    r.core.setMix(1.0f);
    r.run(iL, iR, oL, oR);
    writeWav(std::string("out/") + p.name + "_wet.wav", oL, oR);
    // 35% mix version
    r.core.resetState();
    r.core.setMix(0.35f);
    r.run(iL, iR, oL, oR);
    writeWav(std::string("out/") + p.name + "_mix35.wav", oL, oR);
    // impulse response
    r.core.resetState();
    r.core.setMix(1.0f);
    std::vector<float> imL(n, 0), imR(n, 0); imL[100] = 0.5f; imR[100] = 0.5f;
    r.run(imL, imR, oL, oR);
    writeWav(std::string("out/") + p.name + "_ir.wav", oL, oR);
    printf("  %s (size %.2f, decay %.1f s): wet peak %.3f, nan:%d\n", p.name, p.size, p.decay, peak(oL), hasNaN(oL));
  }
  {
    // also write the dry test signal for A/B
    size_t n = (size_t)(FS * 8);
    std::vector<float> iL(n, 0), iR(n, 0);
    iL[(size_t)(0.2f * FS)] = 0.5f; iR[(size_t)(0.2f * FS)] = 0.5f;
    addNoiseBurst(iL, iR, (size_t)(1.5f * FS), 12.0f, 0.7f, 11);
    addPluck(iL, iR, (size_t)(2.8f * FS), 220.0f, 900, 0.35f, 0.3f);
    addPluck(iL, iR, (size_t)(3.4f * FS), 329.6f, 900, 0.35f, 0.7f);
    addPluck(iL, iR, (size_t)(4.0f * FS), 440.0f, 900, 0.35f, 0.5f);
    addPluck(iL, iR, (size_t)(4.8f * FS), 261.6f, 1500, 0.25f, 0.4f);
    addPluck(iL, iR, (size_t)(4.8f * FS), 392.0f, 1500, 0.25f, 0.6f);
    addPluck(iL, iR, (size_t)(4.8f * FS), 523.3f, 1500, 0.25f, 0.5f);
    writeWav("out/dry.wav", iL, iR);
  }
  printf("done\n");
  return 0;
}
