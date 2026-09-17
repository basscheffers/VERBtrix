// VERBtrix minimal example: Audio Shield line in -> reverb -> line out / headphones.
// Concert Hall preset at 30 % wet. See examples/VERBtrix for serial control of
// every parameter.

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

void setup() {
  AudioMemory(20);
  sgtl5000.enable();
  sgtl5000.inputSelect(AUDIO_INPUT_LINEIN);
  sgtl5000.volume(0.6f);

  reverb.preset(AudioEffectVintageReverb::CONCERT_HALL);
  reverb.mix(0.3f);
}

void loop() {
}
