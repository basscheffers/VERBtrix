// Changes parameters while audio runs and checks for clicks (large
// sample-to-sample jumps relative to the signal's own slew).
// Build: clang++ -O2 -std=c++17 -o param_sweep param_sweep.cpp && ./param_sweep
#include "../../src/VintageReverbCore.h"
#include <vector>
#include <cstdio>
#include <cmath>
using vrv::VintageReverbCore;
static const float FS = 44100.0f;

static void writeWav(const char* path, const std::vector<float>& L, const std::vector<float>& R) {
  FILE* f = fopen(path, "wb"); if (!f) return;
  uint32_t n = (uint32_t)L.size(), dataBytes = n * 4;
  auto w32 = [&](uint32_t v) { fwrite(&v, 4, 1, f); }; auto w16 = [&](uint16_t v) { fwrite(&v, 2, 1, f); };
  fwrite("RIFF", 1, 4, f); w32(36 + dataBytes); fwrite("WAVE", 1, 4, f);
  fwrite("fmt ", 1, 4, f); w32(16); w16(1); w16(2); w32(44100); w32(44100 * 4); w16(4); w16(16);
  fwrite("data", 1, 4, f); w32(dataBytes);
  for (uint32_t i = 0; i < n; i++) {
    int16_t l = (int16_t)lrintf(fmaxf(-1, fminf(1, L[i])) * 32767), r = (int16_t)lrintf(fmaxf(-1, fminf(1, R[i])) * 32767);
    fwrite(&l, 2, 1, f); fwrite(&r, 2, 1, f);
  }
  fclose(f);
}

int main() {
  std::vector<float> mem(VintageReverbCore::memoryRequired(FS));
  VintageReverbCore c;
  c.setDecaySeconds(3); c.setSize(0.5f); c.setMix(1.0f);
  c.init(mem.data(), mem.size(), FS);

  size_t n = ((size_t)(FS * 12) / 128) * 128;
  std::vector<float> iL(n), iR(n), oL(n), oR(n);
  // sustained input: a soft chord of sines (worst case for pitch artifacts)
  for (size_t i = 0; i < n; i++) {
    float t = i / FS;
    float v = 0.15f * (sinf(2 * M_PI * 220 * t) + sinf(2 * M_PI * 277.2f * t) + sinf(2 * M_PI * 329.6f * t));
    iL[i] = v; iR[i] = v * 0.8f;
  }
  // schedule: t=2 size 0.5->1.5, t=5 predelay 0->120ms, t=7 decay 3->0.5, t=9 mix 1->0.3, t=10 freeze on
  for (size_t p = 0; p + 128 <= n; p += 128) {
    float t = p / FS;
    if (fabsf(t - 2.0f) < 0.002f)  c.setSize(1.5f);
    if (fabsf(t - 5.0f) < 0.002f)  c.setPreDelayMs(120);
    if (fabsf(t - 7.0f) < 0.002f)  c.setDecaySeconds(0.5f);
    if (fabsf(t - 9.0f) < 0.002f)  c.setMix(0.3f);
    if (fabsf(t - 10.0f) < 0.002f) c.setFreeze(true);
    c.process(&iL[p], &iR[p], &oL[p], &oR[p], 128);
  }
  // click detector: per 100 ms window, max |x[n]-x[n-1]| vs the window's rms slew
  printf("window  rms      maxSlew/rmsSlew  (spikes >> 6 indicate a click)\n");
  float worst = 0; float worstT = 0;
  for (size_t w = 0; w < n / 4410; w++) {
    size_t a = std::max<size_t>(1, w * 4410), b = (w + 1) * 4410;
    double sq = 0, sl = 0; float mx = 0;
    for (size_t i = a; i < b; i++) { float d = oL[i] - oL[i - 1]; sq += (double)oL[i] * oL[i]; sl += (double)d * d; mx = fmaxf(mx, fabsf(d)); }
    float ratio = mx / (sqrt(sl / (b - a)) + 1e-9f);
    if (ratio > worst) { worst = ratio; worstT = w * 0.1f; }
    if (w % 5 == 0) printf("%5.1fs  %.4f   %.2f\n", w * 0.1f, sqrt(sq / (b - a)), ratio);
  }
  printf("worst maxSlew/rmsSlew = %.2f at t=%.1fs   (a sine has ~1.4, Gaussian noise ~4-5)\n", worst, worstT);
  bool nan = false; for (float v : oL) if (!std::isfinite(v)) nan = true;
  printf("nan: %d\n", nan);
  writeWav("out/param_sweep.wav", oL, oR);
  return 0;
}
