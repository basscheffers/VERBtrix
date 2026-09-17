// VERBtrix - stereo reverb demo for Teensy 4.1 + Audio Shield (SGTL5000)
//
// Line in (stereo) -> AudioEffectVintageReverb -> headphones / line out
//
// Serial monitor commands (115200 baud), each a letter followed by a number:
//   d 3.0    decay (seconds)            s 1.0    size (0.2 .. 1.5)
//   m 0.3    mix (0 dry .. 1 wet)       p 20     predelay (ms)
//   f 0.85   diffusion (0 .. 1)         r 1.0    mod rate (Hz)
//   e 0.4    mod depth (0 .. 1)         b 1.2    bass multiplier (0.25 .. 4)
//   x 300    bass crossover (Hz)        h 0.5    high multiplier (0.1 .. 1)
//   y 4000   high crossover (Hz)        c 12000  high cut (Hz)
//   l 20     low cut (Hz)               z 0/1    freeze
//   t 4      type 0..7 (Ambience, Room, Chamber, Plate, Hall, Cathedral, Arena, Chorus Space)
//   n 17     preset 0..31 by number     N        list all presets
//   ?        print current settings

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <VERBtrix.h>

AudioInputI2S            i2sIn;
AudioEffectVintageReverb reverb;   // ~268 KB of delay memory from the heap (RAM2)
AudioOutputI2S           i2sOut;
AudioControlSGTL5000     sgtl5000;

AudioConnection patchInL(i2sIn, 0, reverb, 0);
AudioConnection patchInR(i2sIn, 1, reverb, 1);
AudioConnection patchOutL(reverb, 0, i2sOut, 0);
AudioConnection patchOutR(reverb, 1, i2sOut, 1);

// If you would rather keep RAM2 free, put the delay memory in PSRAM instead:
//   EXTMEM float reverbMem[AudioEffectVintageReverb::memoryFloats()];
//   AudioEffectVintageReverb reverb(reverbMem, AudioEffectVintageReverb::memoryFloats());

void listPresets() {
  for (int i = 0; i < AudioEffectVintageReverb::PRESET_COUNT; i++) {
    auto p = (AudioEffectVintageReverb::Preset)i;
    const auto &s = AudioEffectVintageReverb::presetSettings(p);
    Serial.printf("  n %2d  %-16s (%s, size %.2f, decay %.1f s)\n", i, AudioEffectVintageReverb::presetName(p),
                  AudioEffectVintageReverb::typeName(s.type), s.size, s.decay);
  }
}

void printSettings() {
  Serial.printf("type: %s   preset: %s\n", AudioEffectVintageReverb::typeName(reverb.type()),
                AudioEffectVintageReverb::presetName(reverb.preset()));
  Serial.printf("decay %.2f s  size %.2f  mix %.2f  predelay %.0f ms  diffusion %.2f\n",
                reverb.decay(), reverb.size(), reverb.mix(), reverb.predelay(), reverb.diffusion());
  Serial.printf("mod %.2f Hz / %.2f  bass x%.2f @ %.0f Hz  high x%.2f @ %.0f Hz  lowcut %.0f  highcut %.0f  freeze %d\n",
                reverb.modRate(), reverb.modDepth(), reverb.bassMult(), reverb.bassXover(),
                reverb.highMult(), reverb.highXover(), reverb.lowCut(), reverb.highCut(), reverb.freeze());
  Serial.printf("reverb CPU %.1f%% (max %.1f%%), total CPU %.1f%%, audio memory %d/%d blocks\n",
                reverb.processorUsage(), reverb.processorUsageMax(), AudioProcessorUsage(),
                AudioMemoryUsage(), AudioMemoryUsageMax());
}

void setup() {
  Serial.begin(115200);
  // Wait (max 4 s) for the serial monitor so the startup messages are not lost.
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 4000) ;

  Serial.println("VERBtrix starting");
  if (CrashReport) {
    Serial.println("---- previous run crashed ----");
    Serial.print(CrashReport);
    Serial.println("------------------------------");
  }
  Serial.printf("reverb memory: %u floats (%u KB), ready=%d\n",
                (unsigned)AudioEffectVintageReverb::memoryFloats(),
                (unsigned)(AudioEffectVintageReverb::memoryFloats() * 4 / 1024), reverb.ready());

  AudioMemory(20);

  Serial.println("enabling SGTL5000...");
  bool codec = sgtl5000.enable();
  Serial.printf("SGTL5000 enable: %s\n", codec ? "ok" : "FAILED (no audio shield found on I2C)");
  sgtl5000.inputSelect(AUDIO_INPUT_LINEIN);
  sgtl5000.lineInLevel(5);
  sgtl5000.volume(0.6f);

  if (!reverb.ready()) {
    Serial.println("Reverb: memory allocation failed, passing audio through");
  }
  reverb.preset(AudioEffectVintageReverb::CONCERT_HALL);
  reverb.mix(0.35f);

  Serial.println("VERBtrix ready. Type ? for settings, N to list presets (commands run on newline or after 100 ms idle).");
}

void handleSerial() {
  static char line[32];
  static uint8_t len = 0;
  static elapsedMillis sinceChar;
  // Execute on newline, or 100 ms after the last character so the command
  // also works when the serial monitor sends no line ending.
  while (Serial.available() || (len > 0 && sinceChar > 100)) {
    char c = Serial.available() ? Serial.read() : '\n';
    sinceChar = 0;
    if (c == '\n' || c == '\r') {
      line[len] = 0;
      if (len > 0) {
        char cmd = line[0];
        float v = atof(line + 1);
        switch (cmd) {
          case 'd': reverb.decay(v);     break;
          case 's': reverb.size(v);      break;
          case 'm': reverb.mix(v);       break;
          case 'p': reverb.predelay(v);  break;
          case 'f': reverb.diffusion(v); break;
          case 'r': reverb.modRate(v);   break;
          case 'e': reverb.modDepth(v);  break;
          case 'b': reverb.bassMult(v);  break;
          case 'x': reverb.bassXover(v); break;
          case 'h': reverb.highMult(v);  break;
          case 'y': reverb.highXover(v); break;
          case 'c': reverb.highCut(v);   break;
          case 'l': reverb.lowCut(v);    break;
          case 'z': reverb.freeze(v > 0.5f); break;
          case 't': reverb.type((AudioEffectVintageReverb::Type)(int)v); break;
          case 'n': reverb.preset((AudioEffectVintageReverb::Preset)(int)v); break;
          case 'N': listPresets(); break;
          case '?': break;
          default: Serial.println("unknown command"); break;
        }
        printSettings();
      }
      len = 0;
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    }
  }
}

void loop() {
  handleSerial();

  static elapsedMillis report;
  if (report > 2000) {
    report = 0;
    Serial.printf("[%lu s] alive. reverb CPU %.1f%% (max %.1f%%), total %.1f%%, audio mem %d/%d\n",
                  millis() / 1000, reverb.processorUsage(), reverb.processorUsageMax(),
                  AudioProcessorUsage(), AudioMemoryUsage(), AudioMemoryUsageMax());
  }
}
