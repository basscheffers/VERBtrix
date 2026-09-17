// Dumps early impulse response statistics per 10 ms window.
#include "../../src/VintageReverbCore.h"
#include <vector>
#include <cstdio>
#include <cmath>
using vrv::VintageReverbCore;
static const float FS = 44100.0f;
int main(int argc, char** argv) {
  float size = argc > 1 ? atof(argv[1]) : 1.0f;
  float diff = argc > 2 ? atof(argv[2]) : 0.85f;
  float modDepth = argc > 3 ? atof(argv[3]) : 0.4f;
  std::vector<float> mem(VintageReverbCore::memoryRequired(FS));
  VintageReverbCore c;
  c.setSize(size); c.setDiffusion(diff); c.setDecaySeconds(3); c.setModDepth(modDepth); c.setPreDelayMs(0);
  c.init(mem.data(), mem.size(), FS);
  size_t n = (size_t)(FS * 1.0f);
  std::vector<float> iL(n, 0), iR(n, 0), oL(n), oR(n);
  iL[0] = 1; iR[0] = 1;
  for (size_t p = 0; p < n; p += 128) c.process(&iL[p], &iR[p], &oL[p], &oR[p], 128);
  printf("window(ms)  peak     rms      n>peak/10  n>1e-3  echoDensity\n");
  for (int w = 0; w < 40; w++) {
    size_t a = (size_t)(w * 0.010f * FS), b = (size_t)((w + 1) * 0.010f * FS);
    float pk = 0; double sq = 0;
    for (size_t i = a; i < b; i++) { pk = std::max(pk, fabsf(oL[i])); sq += (double)oL[i] * oL[i]; }
    float sd = sqrt(sq / (b - a));
    int c1 = 0, c2 = 0, c3 = 0;
    for (size_t i = a; i < b; i++) { if (fabsf(oL[i]) > pk / 10) c1++; if (fabsf(oL[i]) > 1e-3f) c2++; if (fabsf(oL[i]) > sd) c3++; }
    printf("%3d-%3d   %.4f   %.4f   %4d      %4d    %.2f\n", w * 10, w * 10 + 10, pk, sd, c1, c2, c3 / (double)(b - a) / erfc(1 / sqrt(2.0)));
  }
  // first 40 non-trivial samples
  printf("\nfirst samples > 1e-3:\n");
  int shown = 0;
  for (size_t i = 0; i < n && shown < 30; i++) if (fabsf(oL[i]) > 1e-3f) { printf("  t=%.2fms  %.4f\n", i / FS * 1000, oL[i]); shown++; }
  return 0;
}
